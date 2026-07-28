"""MicroPython BLE Robot Control"""
import asyncio
import sys
import time
import aioble  # Built-in async wrapper module for MicroPython BLE
import bluetooth
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

from .app import REMOTE_CMD_LINE_FOLLOW_TOGGLE, REMOTE_CMD_LINE_FOLLOW_DIRECTION, STATE_BLUETOOTH

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment


# --- BLE Constants for Nordic UART Service (NUS) ---
_ADV_TYPE_FLAGS = const(0x01)
_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_TX_UUID   = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
_RX_UUID   = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):  #pylint: disable=invalid-name
    """Register motor-moves-specific settings in the shared settings dict."""
    _ = s
    _ = MySetting

class RobotBLE:
    """Handles BLE communication with the Bluefruit Connect app on a phone."""

    __slots__ = "_ble", "_write_callback", "_name", "_logging", "_is_enabled", "_active_connection", "_uart_service", "_rx_characteristic", "_tx_characteristic"

    def __init__(self, ble, name="Robot", logging: bool = False):
        self._ble = ble
        self._write_callback = None
        self._name: str = name
        self._logging = logging
        self._is_enabled = False  # Master control flag for the hardware loop
        self._active_connection = None
        self._uart_service = None
        self._rx_characteristic = None
        self._tx_characteristic = None
        self.init()


    def init(self):
        """ Register service nodes using safe async declarations """
        self._ble.active(True)  # Ensure the hardware radio is awake before registering services
        self._uart_service = aioble.Service(_UART_UUID)
        self._tx_characteristic = aioble.Characteristic(
            self._uart_service, _TX_UUID, notify=True
        )
        self._rx_characteristic = aioble.Characteristic(
            self._uart_service, _RX_UUID, write=True, write_no_response=True, capture=True
        )

        aioble.register_services(self._uart_service)
        print("B:BLE:Services registered, tx handle:", self._tx_characteristic._value_handle)
        print("B:BLE:Services registered, rx handle:", self._rx_characteristic._value_handle)


    async def run_loop(self):
        """
        Main application task loop.
        Spawns background routines completely within the asyncio framework.
        """
        # Set the Bluetooth Device Name cleanly
        self._ble.config(gap_name=self._name)

        self._is_enabled = True  # Enable the BLE loop

        # Concurrent processing structures: manages advertising and read tasks simultaneously
        await asyncio.gather(
            self._advertising_task(),
            self._read_handler_task()
        )


    async def _advertising_task(self):
        """Safely cycles advertising without blocking hardware registers."""

        while True:
            if not self._is_enabled:
                await asyncio.sleep_ms(500)
                continue
            try:
                print(f"B:BLE:Advertising {self._name}")

                # Pass raw bytearrays directly to override aioble's layout picker
                connection = await aioble.advertise(
                    500000,
                    name=self._name,
                    services=[_UART_UUID] # High-level binding required by discovery
                )

                if connection is None:
                    print("B:BLE:Advertising timed out, restarting...")
                else:
                    print("B:BLE:Connected to central device")
                    # Code blocks here until a smartphone connects
                    self._active_connection = connection

                    # Wait until the phone disconnects
                    await connection.disconnected()
                    print("B:BLE:Central device disconnected")

            except Exception as e:   # pylint: disable=broad-except
                print(f"B:BLE:Advertising error: {e}")
                await asyncio.sleep_ms(2000) # Breathing room before restarting
            finally:
                # Clean up connection flags if a disconnection event occurs
                self._active_connection = None
                await asyncio.sleep_ms(500) # Breathing room before restarting


    async def _read_handler_task(self):
        """Safely processes incoming data streams using async queues."""
        while True:
            if not self._is_enabled or self._active_connection is None:
                # nothing to try to receive as no connection is active
                await asyncio.sleep_ms(500)
                continue

            # Code pauses here cleanly until data arrives in the RX register
            # written by a central application (like Bluefruit Connect)
            try:
                _, value = await self._rx_characteristic.written(timeout_ms=1000)
                print(f"B:BLE:RX characteristic written: {value}")
                if value and self._write_callback:
                    self._write_callback(value)
            except asyncio.TimeoutError:
                if self._active_connection is None:
                    # Connection was dropped while waiting for data
                    print("B:BLE:Connection lost while waiting for RX data")
                pass  # No data received in this interval, loop back to check connection state
            except Exception as e:   # pylint: disable=broad-except
                print(f"B:BLE:Error reading RX characteristic data: {e}")
                await asyncio.sleep_ms(50)


    def send_telemetry(self, text):
        """Sends data out via Bluetooth"""
        if not self._active_connection:
            return
        try:
            # Writes data directly into the notification register structure
            self._tx_characteristic.notify(self._active_connection, (text + "\n").encode())
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
        return self._active_connection is not None


    @property
    def is_enabled(self) -> bool:
        """Returns True if the BLE hardware is currently enabled and advertising or connected."""
        return self._is_enabled


    async def disconnect_client(self):
        """
        Forces the badge to sever the link with the smartphone application.
        """
        if self._active_connection:
            try:
                print("B:BLE:Soft disconnect triggered...")
                # Instruct the hardware radio stack to drop the central device
                await self._active_connection.disconnect()
            except Exception as e:          # pylint: disable=broad-except
                print(f"B:BLE:Error during disconnect execution: {e}")
            finally:
                # Clear references immediately in case the hardware link takes a moment to drop
                self._active_connection = None


    # --- DISABLING & ENABLING METHODS ---

    def shutdown_ble(self):
        """
        Completely powers down the physical Bluetooth hardware antenna
        and halts the internal processing tasks.
        """
        print("B:BLE:Global shutdown triggered. Terminating capability...")
        self._is_enabled = False

        # 1. Pull the physical power down from the hardware layer
        # This completely frees the internal transceiver peripheral and radio stack.
        self._ble.active(False)

        # 2. Reclaim state registers
        self._active_connection = None


    def restart_ble(self, name: str | None = None):
        """
        Powers up the physical Bluetooth radio and automatically
        resumes background broadcasting.
        """
        if self._is_enabled:
            print("B:BLE:Bluetooth is already running.")
            return

        print("B:BLE:Global restart triggered. Initialising hardware stack...")

        # 1. Re-initialize the low-level silicon transceiver registers
        self.init() # re-register services and characteristics

        if name is not None:
            self._name = name
        self._ble.config(gap_name=self._name)

        # 2. Release the async loops from their sleep states
        self._is_enabled = True



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


def ble_process_command(data):
    """
    Bluefruit Connect Control Pad sends data in the format:
    !B <button_number> <1=pressed/0=released> <checksum>
    Example: b'!B516' is Up Button Pressed
    """
    global _ble_active_button

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
        self._name: str | None = None


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
        self._name: str = name

        if self._logging:
            print(f"B:Initialising Bluetooth LE with name = {name}")

        if self._ble_controller is not None:
            if self._name != self._ble_controller.name:
                # Name has changed, so we need to restart the BLE controller to update the advertising name.
                if self._ble_controller.is_connected:
                    # Create it as a task on the running loop context
                    asyncio.get_event_loop().create_task(self._ble_controller.disconnect_client())
                    time.sleep_ms(20)  # Give the disconnect time to propagate before powering down the radio
                    self._is_connected = False

                # Disable the BLE hardware and stop advertising
                self._ble_controller.shutdown_ble()

                if self._is_enabled:
                    # Re-enable the BLE hardware and resume advertising - IF it was previously enabled
                    self._ble_controller.restart_ble(name=self._name)
        else:
            # Initialize the BLE controller
            try:
                # Create a new RobotBLE instance and start advertising
                self._ble_controller = RobotBLE(self._ble, name=self._name, logging=self._logging)

                # Add the safe BLE tracking loop to the global scheduler
                asyncio.get_event_loop().create_task(self._ble_controller.run_loop())

            except Exception as e:      # pylint: disable=broad-except
                print(f"B:Failed to initialize BLE controller: {e}")
                self._ble_controller = None
                return False

            # Register the command processor
            self._ble_controller.on_write(ble_process_command)

            if self._logging:
                print("B:BluetoothMgr initialised")

        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()
        return True


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
                # Create it as a task on the running loop context
                asyncio.get_event_loop().create_task(self._ble_controller.disconnect_client())
                time.sleep_ms(20)  # Give the disconnect time to propagate before powering down the radio
                # Disable the BLE hardware and stop advertising
                self._ble_controller.shutdown_ble()
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]): # "BLE On" / "OK"
            app.button_states.clear()
            if self._ble_controller is not None and not self._ble_controller.is_enabled:
                # Enable the BLE hardware and resume advertising
                self._ble_controller.restart_ble(name=self._name)
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
