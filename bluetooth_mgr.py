"""MicroPython BLE Robot Control"""
import gc
import struct
import sys
import bluetooth
import micropython
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

from .app import REMOTE_CMD_LINE_FOLLOW_TOGGLE, REMOTE_CMD_LINE_FOLLOW_DIRECTION, STATE_BLUETOOTH

try:
    from machine import mem32
except ImportError:
    class _Mem32Shim:
        def __getitem__(self, _addr: int) -> int:
            return 0

        def __setitem__(self, _addr: int, _value: int) -> None:
            return None

    # Simulator fallback: keep imports working even when direct register access
    # and IRQ controls are not exposed by the simulated machine module.
    mem32 = _Mem32Shim()


try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment


#Micropython Bluetooth is the crashiest thing I've ever seen.
# Have tried using aioble - same result.
# Have tried to avoid using irq at all but then you can't get the connection handle,
# but I don't think that is the problem, as the crash happens when there is no data beign sent or received.
# it is more likley if you load the transmissions up.
# its core C code gets stuck due to memory issues with micropython and the ESP core wdt kicks in
# disabling the garbage collector didn't help. (NB miicropython does seem to churn a huge amount of memory that needs to be garbage collected)



# --- BLE Constants for Nordic UART Service (NUS) ---
_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_TX_UUID   = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_RX_UUID   = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

_ADV_INTERVAL_US = const(500000)

# --- CRITICAL: Declare flat, unboxed global storage primitives ---
# Keeping these outside the class guarantees no complex lookup structures
_BLE_EVENT_FLAG = 0
_BLE_PENDING_EVENT = 0
_BLE_PENDING_DATA = 0


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):  #pylint: disable=invalid-name
    """Register bluetooth-specific settings in the shared settings dict."""
    _ = s
    _ = MySetting

class RobotBLE:
    """Handles BLE communication with the Bluefruit Connect app on a phone."""
    __slots__ = (
        "_ble", "_write_callback", "_name", "_logging", "_enabled", "_connected", "_connection_handle", "_handle_tx", "_handle_rx",
        "_adv_data",
    )

    def __init__(self, ble, name="BBot", logging: bool = False):
        self._ble = ble
        self._write_callback = None
        self._name: str = name
        self._logging: bool = logging
        self._enabled: bool = False  # Master control flag for the hardware loop
        self._connected: bool = False
        self._connection_handle = None
        self._handle_tx = None
        self._handle_rx = None
        self._adv_data = None
        self.init()


    def init(self):
        """ Register service nodes using safe async declarations """
        print("B:BLE:Initializing BLE")

        self._connected = False
        self._connection_handle = None
        #gc.enable()  # Enable garbage collection while there is no connection

        self._enabled = True
        self._ble.active(True)  # Ensure the hardware radio is awake before registering services
        self._ble.irq(self._irq)

        _UART_SERVICE = (_UART_UUID, (
            (_TX_UUID, bluetooth.FLAG_NOTIFY),
            (_RX_UUID, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE),
        ))

        # Register the profile structure and capture the direct hardware register handles
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))

        self._adv_data = self._advertising_payload(name=self._name, services=[_UART_UUID])

        # Configure local device name parameter
        self._ble.config(gap_name=self._name)

        self.start_advertising()


    @micropython.native
    def _irq(self, event, data):
        """
        PERFECT STATE: Zero memory lookups, zero allocations, zero object referencing.
        """
        global _BLE_EVENT_FLAG, _BLE_PENDING_EVENT, _BLE_PENDING_DATA

        _BLE_PENDING_EVENT = event
        _BLE_PENDING_DATA = data
        _BLE_EVENT_FLAG = 1


    def start_advertising(self):
        """Start advertising the BLE service."""

        self._ble.gap_advertise(_ADV_INTERVAL_US, adv_data=self._adv_data)
        print(f"B:BLE:Advertising as {self._name}")


    def update(self):
        """
        Call this function regularly inside main application loop.
        """
        #if self._connected:
        #    # garbage collection is disabled while a BLE central is connected to avoid memory fragmentation during active connection
        #    # if it looks like we are low on memory then we can force a garbage collection cycle to free up memory, but this will cause a brief pause in the main loop while the garbage collection runs.
        #    if gc.mem_free() < 60000:  # Check if free memory is below a threshold (e.g., 20KB)
        #        print("B:BLE:Low memory detected, forcing garbage collection...")
        #        try:
        #            gc.collect()  # Force garbage collection to free up memory
        #        except Exception as e:  # pylint: disable=broad-except
        #            print(f"B:BLE:Error during garbage collection: {e}")
        #        finally:
        #            print(f"B:BLE:Memory after GC: {gc.mem_free()} bytes free")

        global _BLE_EVENT_FLAG, _BLE_PENDING_EVENT, _BLE_PENDING_DATA
        if not _BLE_EVENT_FLAG:
            return
        _BLE_EVENT_FLAG = 0

        event = _BLE_PENDING_EVENT
        data  = _BLE_PENDING_DATA

        if event == 1:    # _IRQ_CENTRAL_CONNECT
            if self._enabled:
                self._connected = True
                self._connection_handle = data[0] if data else 0
                #print(f"B:BLE:Central connected, handle={self._connection_handle}")
                #gc.disable()  # Disable garbage collection to avoid memory fragmentation during active connection
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            self._connected = False
            self._connection_handle = None
            if self._enabled:
                self.start_advertising()
            #gc.enable()  # Re-enable garbage collection after disconnection
        elif event == 3:  # _IRQ_GATTS_WRITE
            if self._enabled and self._connected:
                handle = data[1] if data else 0
                value = self._ble.gatts_read(handle)
                #print(f"B:BLE:Received data, handle={handle}: {value}")
                if handle == self._handle_rx and self._write_callback:
                    try:
                        self._write_callback(value)
                    except Exception as e:      # pylint: disable=broad-except
                        print(f"B:BLE:Callback Error: {e}")


    def _advertising_payload(self, name=None, services=None):
        # name is limited to 8 characters as the total packet is only 31 bytes.
        # services is a list of UUID objects.
        payload = bytearray()

        def _append(adv_type, value):
            nonlocal payload
            payload.append(len(value) + 1)
            payload.append(adv_type)
            payload.extend(value)

        _append(_ADV_TYPE_FLAGS, struct.pack("B", 0x06))
        if name:
            _append(_ADV_TYPE_NAME, name.encode('utf-8'))
        if services:
            for s in services:
                _append(_ADV_TYPE_UUID128_COMPLETE, bytes(s))

        return payload


    def send_telemetry(self, text):
        """Sends data out via Bluetooth"""
        if not self._connected:
            return
        try:
            # Transmit data via the TX characteristic
            self._ble.gatts_notify(self._connection_handle, self._handle_tx, text + "\n")
        except Exception as e:       # pylint: disable=broad-except
            print(f"B:BLE:Error sending telemetry: {e}")


    def on_write(self, callback):
        """Registers a callback function to be called when the phone app sends data."""
        self._write_callback = callback


    @property
    def name(self) -> str:
        """Returns the current Bluetooth device name."""
        return self._name

# NB to change name you need to shutdown and restart the BLE controller with the new name, which will update the advertising name.

    @property
    def is_connected(self) -> bool:
        """Returns True if at least one BLE central is connected."""
        return self._connected


    @property
    def is_enabled(self) -> bool:
        """Returns True if the BLE hardware is currently enabled and advertising or connected."""
        return self._enabled


    # --- DISABLING & ENABLING METHODS ---

    def disconnect_client(self):
        """
        Forces the remote smartphone to disconnect instantly over the air.
        """
        if self._connected:
            print("B:BLE:Issuing hardware disconnection command...")
            # Native low-level command to drop a connection handle explicitly
            try:
                self._ble.gap_disconnect(self._connection_handle)
            except Exception as e:  # pylint: disable=broad-except
                print(f"B:BLE:Error disconnecting client: {e}")
            self._connected = False
            self._connection_handle = None
            #gc.enable()  # Re-enable garbage collection after disconnection



    def stop_advertising(self):
        """
        Stops the over-the-air beacon broadcasts completely, preventing
        new devices from finding the badge, but keeps the BLE stack alive.
        """
        # To stop advertising natively in MicroPython, pass None into the interval argument
        self._ble.gap_advertise(None)


    def shutdown(self):
        """
        Performs a full shutdown: severs active clients, stops advertisements,
        and cuts the physical power to the 2.4 GHz radio transceiver to save power.
        """
        self._enabled = False

        self.disconnect_client()

        self.stop_advertising()

        # Pull the physical power down from the ESP32-S3 silicon transceiver block.
        # This stops peripheral scanning clocks completely to optimize battery metrics.
        self._ble.active(False)
        print("B:BLE:Peripheral Hardware Stack Powered Down.")


    def restart(self):
        """
        Powers up the physical radio antenna, reinitializes configuration blocks,
        and resumes over-the-air pairing advertisements.
        """
        self.init()
        print("B:BLE:Hardware core awake. Advertising active.")


# --- Robot Logic ---

# Direction buttons that can override motor output from the current state.
# '5' = forward, '6' = backward, '7' = left, '8' = right.
_DRIVE_BUTTONS = frozenset('45678') # 5/6/7/8 form a control keypad, but 4 is used as emergency stop which will override other apps trying to drive the robot.
_CONTROL_BUTTONS = frozenset('0123')

# Map phone control-pad buttons to transport-agnostic app remote commands.
# Button presses are translated to these and posted to the app, which owns all
# state control.  Add entries here to expose more remote features.
_CONTROL_BUTTON_COMMANDS = {
    '1': REMOTE_CMD_LINE_FOLLOW_TOGGLE,     # activate Line Follower / toggle start-stop
    '2': REMOTE_CMD_LINE_FOLLOW_DIRECTION,  # toggle Line Follower direction
}

# Currently-held BLE drive button, or None when no button is pressed.
_ble_active_button = None

# Queue of pending control-button presses ('0'-'3') from the phone.  Appended in
# ble_process_command (runs on the main thread via schedule) and drained by
# BluetoothMgr.update() so the app can action them.
_ble_pending_buttons = []
_logging = False

def ble_process_command(data):
    """
    Bluefruit Connect Control Pad sends data in the format:
    !B <button_number> <1=pressed/0=released> <checksum>
    Example: b'!B516' is Up Button Pressed
    """
    global _ble_active_button, _logging

    if _logging:
        print(f"B:BLE:Processing command: {data}")
        
    command = data.decode().strip()
    if not command.startswith("!B"):
        print(f"B:BLE:Invalid command format: {command}")
        return

    # Check button number and press state
    button = command[2]
    action = command[3] # '1' for press, '0' for release

    if button in _CONTROL_BUTTONS:
        #print(f"BLE:Control Button {button} {'Pressed' if action == '1' else 'Released'}")
        if action == '1':
            # Queue the press; BluetoothMgr.update() translates and forwards it to the app.
            _ble_pending_buttons.append(button)
        return

    if button in _DRIVE_BUTTONS:
        if action == '1': # Button pressed
            _ble_active_button = button
            #if button == '4':
            #    print("BLE:Stop")
            #elif button == '5':
            #    print("BLE: Forward")
            #elif button == '6':
            #    print("BLE: Backward")
            #elif button == '7':
            #    print("BLE: Left")
            #elif button == '8':
            #    print("BLE: Right")
        else: # Button released — clear override only if it's the button we're tracking
            if _ble_active_button == button:
                _ble_active_button = None
                #print("BLE: Release")



# ---------------------------------------------------------------------------
# BLE Logging - redirect sys.stdout so all print() calls are also forwarded
# over BLE when explicitly enabled.  The rest of the codebase is untouched.
# ---------------------------------------------------------------------------

class BleLogStream:
    """Proxy for sys.stdout that tees complete log lines to a BluetoothMgr instance."""

    def __init__(self, ble_controller, original_stdout):
        self._ble = ble_controller
        self._orig = original_stdout
        self._line_buf = []

    def write(self, text):
        """Write text to the original stdout and also send complete lines to BLE."""
        # Always write to the original stdout (USB/UART serial)
        self._orig.write(text)
        # Buffer characters and send each complete line to BLE
        if '\n' in text:
            parts = text.split('\n')
            # First segment completes whatever is already in the buffer
            self._line_buf.append(parts[0])
            line = ''.join(self._line_buf)
            if line:
                self._ble.send_telemetry(line)
            # Any middle segments are self-contained complete lines
            for part in parts[1:-1]:
                if part:
                    self._ble.send_telemetry(part)
            # The trailing segment starts a new partial line
            self._line_buf = [parts[-1]] if parts[-1] else []
        else:
            self._line_buf.append(text)


    def flush(self):
        """Flush the original stdout if it has a flush() method."""
        try:
            self._orig.flush()
        except AttributeError:
            pass


_ble_log_stream = None
_orig_stdout = None

### THIS DOES NOT WORK!!!
def enable_ble_logging(ble_controller):
    """Redirect sys.stdout through BleLogStream so every print() is also sent via BLE."""
    global _ble_log_stream, _orig_stdout
    if _ble_log_stream is None:
        _orig_stdout = sys.stdout
        _ble_log_stream = BleLogStream(ble_controller, _orig_stdout)
        #sys.stdout = _ble_log_stream


def disable_ble_logging():
    """Restore sys.stdout to serial-only output."""
    global _ble_log_stream, _orig_stdout
    if _ble_log_stream is not None:
        #sys.stdout = _orig_stdout
        _ble_log_stream = None
        _orig_stdout = None


class BluetoothMgr:
    """Manages the Bluetooth functionality.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """
    __slots__ = ("_app", "_logging", "_ble", "_ble_controller", "_is_connected", "_is_enabled", "_name")

    def __init__(self, main_app, logging=True):
        self._app = main_app
        self._logging: bool = logging
        self._ble = bluetooth.BLE()
        self._ble_controller: RobotBLE | None = None
        self._is_connected: bool = False
        self._is_enabled: bool = False
        self._name: str = ""


    @property
    def logging(self) -> bool:
        """Whether to print debug logs from the bluetooth manager."""
        return self._logging

    @logging.setter
    def logging(self, value: bool):
        """Enable or disable debug logging for the bluetooth manager."""
        self._logging = value


    @property
    def is_active(self) -> bool:
        """Returns True if the Bluetooth manager is currently active (started)."""
        return self._ble_controller is not None and self._ble_controller.is_enabled


    @property
    def is_connected(self) -> bool:
        """Returns True if a BLE central is connected."""
        return self._is_connected


    def motor_override(self, max_power: int):
        """Return a (left, right) motor override tuple if a BLE drive button is
        currently held, or None to let the current state control the motors.

        max_power should be the full-scale PWM value (0-65535).
        """
        btn = _ble_active_button
        if btn is None:
            return None
        if btn == '4':                          # Stop
            return (0, 0)
        if btn == '5':                          # Forward
            return (max_power, max_power)
        if btn == '6':                          # Backward
            return (-max_power, -max_power)
        if btn == '7':                          # Left
            return (-max_power, max_power)
        if btn == '8':                          # Right
            return (max_power, -max_power)
        return None


    def start(self, name: str = "BBot") -> bool:
        """Start the Bluetooth manager and begin advertising."""
        app = self._app
        self._name = name

        if self._logging:
            print(f"B:Initialising Bluetooth LE with name = {name}")

        if self._ble_controller is not None:
            if self._name != self._ble_controller.name:
                # Name has changed, so we need to restart the BLE controller to update the advertising name.
                if self._ble_controller.is_connected:
                    self._ble_controller.disconnect_client()

                self._ble_controller.shutdown()

                if self._is_enabled:
                    # Re-enable the BLE hardware and resume advertising - IF it was previously enabled
                    self._ble_controller.restart()
        else:
            # Initialize the BLE controller
            try:
                # Create a new RobotBLE instance and start advertising
                self._ble_controller = RobotBLE(self._ble, name=self._name, logging=self._logging)

            except Exception as e:      # pylint: disable=broad-except
                print(f"B:Failed to initialize BLE controller: {e}")
                self._ble_controller = None
                return False

            # Register the command processor
            self._ble_controller.on_write(ble_process_command)
            global _logging
            _logging = self._logging

            if self._logging:
                print("B:BluetoothMgr initialised")

        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()
        return True


    def background_update(self, delta: int) -> None:
        """Update the Bluetooth manager's state in the background.  This should be called in the main loop's background update phase."""
        _ = delta
        if self._ble_controller is not None and self._ble_controller.is_enabled:
            self._ble_controller.update()


    def update(self, delta: int) -> None:
        """Update the Bluetooth manager's state.  This should be called in the main loop's update phase."""
        _ = delta
        app = self._app

        if self._ble_controller is not None:
            if self._ble_controller.is_enabled and not self._is_enabled:
                self._is_enabled = True
                app.refresh = True
            elif not self._ble_controller.is_enabled and self._is_enabled:
                self._is_enabled = False
                app.refresh = True
            if self._ble_controller.is_connected and not self._is_connected:
                self._is_connected = True
                app.notification = Notification("BLE Connected")
                app.refresh = True
            elif not self._ble_controller.is_connected and self._is_connected:
                self._is_connected = False
                app.notification = Notification("BLE Disconnected")
                app.refresh = True

        # Forward any queued phone control-button presses to the app as remote commands.
        self._process_control_buttons()

        # only process button presses if the app is in the Bluetooth State
        if app.current_state != STATE_BLUETOOTH:
            return

        # Attempts to have a "Disconnect" button - caused issues, Bluefruit would immediately reconnect, but the messages were not received by the app until it was forced to disconnect and reconnect again.

        if app.button_states.get(BUTTON_TYPES["CANCEL"]): # Back
            app.button_states.clear()
            # Return to menu - leaving BLE connection state as is (i.e. currently the same as "OK" button)
            app.return_to_menu()
        elif app.button_states.get(BUTTON_TYPES["LEFT"]): # "BLE Off"
            app.button_states.clear()
            if self._ble_controller is not None and self._ble_controller.is_enabled:
                self._ble_controller.shutdown()
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]): # "BLE On" / "OK"
            app.button_states.clear()
            if self._ble_controller is not None and not self._ble_controller.is_enabled:
                # Enable the BLE hardware and resume advertising
                self._ble_controller.restart()
            else: # "OK"
                # return to menu leaving the BLE connection active so the user can continue to control the robot from the phone app
                app.return_to_menu()


    def _process_control_buttons(self):
        """Translate any pending phone control-button presses into transport-agnostic
        remote commands and post them to the app, which owns all state control."""
        while _ble_pending_buttons:
            button = _ble_pending_buttons.pop(0)
            command = _CONTROL_BUTTON_COMMANDS.get(button)
            if command is not None:
                self._app.post_remote_command(command)


    def draw(self, ctx) -> bool:
        """Draw the Bluetooth manager's UI elements.  This should be called in the main loop's draw phase."""
        app = self._app
        # if connected then show a message to the user that they can control the robot with the Bluefruit LE app
        if self._is_connected:
            app.draw_message(ctx, ["BLE connected", "use Bluefruit", "Connect to", "control."], [(0.5, 0.5, 1), (0.5, 1, 0.5), (0.5, 1, 0.5), (0.5, 1, 0.5)], label_font_size)
        elif self._ble_controller is not None and self._ble_controller.is_enabled:
            app.draw_message(ctx, ["BLE enabled:", "On Phone", "use Bluefruit", "Connect with", f"{self._name}"], [(0.5, 0.5, 1), (1, 1, 0), (1, 1, 0), (1, 1, 0), (0.5, 1, 1)], label_font_size)
        else:
            app.draw_message(ctx, ["BLE disabled"], [(1, 0.5, 0.5)], label_font_size)

        if self._ble_controller is not None and self._ble_controller.is_enabled:
            left_label = "BLE Off"
            confirm_label = "OK"
        else:
            left_label = ""
            confirm_label = "BLE On"
        button_labels(ctx, confirm_label=confirm_label, cancel_label="Back", left_label=left_label)
        return True


    # Method to send Chartable data to the phone app via BLE
    def send_plotter_data(self, data: list[int]):
        """Send chartable data to the phone app via BLE."""
        if self._ble_controller is not None and self._ble_controller.is_connected:
            # Join numbers with commas (the mandatory newline is appended by the send_telemetry method)
            plot_string = ",".join(str(val) for val in data)
            self._ble_controller.send_telemetry(plot_string)
