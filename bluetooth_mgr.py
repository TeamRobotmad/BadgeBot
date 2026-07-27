"""MicroPython BLE Robot Control"""

import struct
import sys
import bluetooth
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
from micropython import schedule
from .app import REMOTE_CMD_LINE_FOLLOW_TOGGLE, REMOTE_CMD_LINE_FOLLOW_DIRECTION

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment

# --- BLE Constants for Nordic UART Service (NUS) ---
_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    bluetooth.FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE,
)

_UART_SERVICE = (_UART_UUID, (_UART_TX, _UART_RX))

# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):  #pylint: disable=invalid-name
    """Register motor-moves-specific settings in the shared settings dict."""
    _ = s
    _ = MySetting

class RobotBLE:
    """Handles BLE communication with the Bluefruit Connect app on a phone."""
    __slots__ = ("_ble", "_connections", "_write_callback", "_payload", "_handle_tx", "_handle_rx", "_name", "_logging","_ble_event_triggered", "_pending_event", "_pending_data")

    def __init__(self, ble, name="Robot", logging: bool = False):
        self._ble = ble
        self._connections = set()
        self._write_callback = None
        self._payload = None
        self._handle_tx = None
        self._handle_rx = None
        self._name = name
        self._logging = logging

        # Pre-allocate thread event tracking properties
        self._ble_event_triggered = False
        self._pending_event = 0
        self._pending_data = None

        self.init()


    def init(self):
        """Re-initialize the BLE controller if it was deinitialized."""
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections.clear()
        self._write_callback = None
        self._payload = self._advertising_payload(name=self._name, services=[_UART_UUID])
        # Override the internal firmware name string - This stops the phone app from falling back to "MPY ESP32" upon connection
        self._ble.config(gap_name=self._name)
        self._advertise()


    def _irq(self, event, data):
        # 1. Catch the hardware event immediately
        # 2. Bounce it out to the main execution thread safely
        # SAFE REWRITE: Performs zero variable copying or block moves.
        # It stores integers and handles references instantly, bypassing the ROM.

        self._pending_event = event
        self._pending_data = data
        self._ble_event_triggered = True
        schedule(self._safe_irq_handler, None)


    def _safe_irq_handler(self, _):
        # This runs safely on the main thread.
        # Printing and memory allocation here will NEVER trigger an Interupt WDT crash.

        if not self._ble_event_triggered:
            return

        # Reset event flag
        self._ble_event_triggered = False
        event = self._pending_event
        data = self._pending_data

        if event == 1:  # _IRQ_CENTRAL_CONNECT
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            #if self._logging:
            #    print("B:BLE:Connected")

        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            self._advertise()
            #if self._logging:
            #    print("B:BLE:Disconnected")

        elif event == 3:  # _IRQ_GATTS_WRITE
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            #if self._logging:
            #    print(f"B:BLE:RX: {value.decode().strip()}")
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)


    def _advertise(self, interval_us=500000):
        #print("BLE:Advertising...")
        try:
            self._ble.gap_advertise(interval_us, adv_data=self._payload)
        except OSError as e:
            print(f"BLE:Advertising failed: {e}")


    def send_telemetry(self, text):
        """Sends sensor data or diagnostic logs back to the phone app."""
        for conn_handle in self._connections:
            try:
                # Transmit data via the TX characteristic
                self._ble.gatts_notify(conn_handle, self._handle_tx, text + "\n")
            except Exception:  # pylint: disable=broad-except
                pass


    def on_write(self, callback):
        """Registers a callback function to be called when the phone app sends data."""
        self._write_callback = callback


    def is_connected(self):
        """Returns True if at least one BLE central is connected."""
        return len(self._connections) > 0


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


# DO NOT USE WITHOUT CARE AND SOME DEBUGGING - seems to cause crashes
    def deinit(self):
        """Deinitializes the BLE controller and stops advertising."""
        self._ble.active(False)
        self._connections.clear()
        self._write_callback = None
        if self._logging:
            print("B:BLE:Deinitialized")


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

    command = data.decode().strip()
    if not command.startswith("!B"):
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
    __slots__ = ("_app", "_logging", "_ble", "_ble_controller", "_is_connected", "_name")

    def __init__(self, main_app, logging=True):
        self._app = main_app
        self._logging: bool = logging
        self._ble = bluetooth.BLE()
        self._ble_controller: RobotBLE | None = None
        self._is_connected: bool = False
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
        return self._ble_controller is not None


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


    def start(self, name: str = "RobotXXX") -> bool:
        """Start the Bluetooth manager and begin advertising."""
        self._name = name
        app = self._app

        if self._ble_controller is None:
            if self._logging:
                print(f"B:Initialising Bluetooth LE with name = {name}")

            # BLE doesn't seem to like being deinitisalised - perhpaps somethign wrong in the sequence...
            # If the BLE controller is already initialized, deinitialize it first
            #if self._ble_controller is not None:
            #    print("B:Deinitializing existing BLE controller...")
            #    self._ble_controller.deinit()
            #    self._ble_controller = None

            # Initialize the BLE controller
            try:
                self._ble_controller = RobotBLE(self._ble, name=name, logging=self._logging)
                if self._logging:
                    print("B:BLE controller initialized")
            except Exception as e:      # pylint: disable=broad-except
                print(f"B:Failed to initialize BLE controller: {e}")
                self._ble_controller = None

            if self._ble_controller is None:
                print("B:BLE controller is None, cannot start Bluetooth manager")
                return False

            # Register the command processor
            if self._logging:
                print("B:Registering BLE command processor...")
            self._ble_controller.on_write(ble_process_command)
            if self._logging:
                print("B:BluetoothMgr initialised")
        else:
            if self._logging:
                print("B:BLE controller already initialized")

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
            if self._ble_controller.is_connected() and not self._is_connected:
                self._is_connected = True
                app.notification = Notification("BLE Connected")
                app.refresh = True
            elif not self._ble_controller.is_connected() and self._is_connected:
                self._is_connected = False
                app.notification = Notification("BLE Disconnected")
                app.refresh = True

        # Forward any queued phone control-button presses to the app as remote commands.
        self._process_control_buttons()

        if app.button_states.get(BUTTON_TYPES["CANCEL"]): # Exit
            app.button_states.clear()
            # return to menu and disconnect from BLE if connected
            #if self._ble_controller is not None:
            #    self._ble_controller.deinit()
            #    self._ble_controller = None
            app.return_to_menu()
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]): # OK
            app.button_states.clear()
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
            app.draw_message(ctx, ["BLE connected", "use Bluefruit", "Connect to", "control."], [(0.5, 1, 0.5), (0.5, 1, 0.5), (0.5, 1, 0.5), (0.5, 1, 0.5)], label_font_size)
        else:
            app.draw_message(ctx, ["BLE enabled:", "on Phone", "use Bluefruit", "Connect with", f"{self._name}"], [(1, 1, 0), (1, 1, 0), (1, 1, 0), (1, 1, 0), (0.5, 1, 1)], label_font_size)

        button_labels(ctx, confirm_label="OK", cancel_label="Exit")
        return True


# TODO - some sort of pairing / bonding / security mechanism to prevent random phones from controlling the robot.
