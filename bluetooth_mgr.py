"""MicroPython BLE Robot Control"""

import struct
import sys
import bluetooth
from micropython import schedule
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

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


class RobotBLE:
    """Handles BLE communication with the Bluefruit Connect app on a phone."""
    def __init__(self, ble, name="Robot-Control"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = self._advertising_payload(name=name, services=[_UART_UUID])
        self._advertise()


    def _irq(self, event, data):
        # 1. Catch the hardware event immediately
        # 2. Bounce it out to the main execution thread safely
        schedule(self._safe_irq_handler, (event, data))


    def _safe_irq_handler(self, params):
        # This runs safely on the main thread.
        # Printing and memory allocation here will NEVER trigger an Interupt WDT crash.
        event, data = params

        if event == 1:  # _IRQ_CENTRAL_CONNECT
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("B:BLE:Connected")

        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.remove(conn_handle)
            self._advertise()
            print("B:BLE:Disconnected")

        elif event == 3:  # _IRQ_GATTS_WRITE
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)


    def _advertise(self, interval_us=500000):
        print("BLE:Advertising...")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)


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

    def deinit(self):
        """Deinitializes the BLE controller and stops advertising."""
        self._ble.active(False)
        self._connections.clear()
        self._write_callback = None
        print("BLE:Deinitialized")


# --- Robot Logic ---

# Direction buttons that can override motor output from the current state.
# '4' = stop, '5' = forward, '6' = backward, '7' = left, '8' = right.
_DRIVE_BUTTONS = frozenset('45678')
_CONTROL_BUTTONS = frozenset('0123')

# Currently-held BLE drive button, or None when no button is pressed.
_ble_active_button = None


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
        print(f"BLE:Control Button {button} {'Pressed' if action == '1' else 'Released'}")
        if action == '1':
            if button == '0':
                # toggle Flood LED state
                print("BLE:Toggle Flood LED")
        return

    if button in _DRIVE_BUTTONS:
        if action == '1': # Button pressed
            _ble_active_button = button
            if button == '4':
                print("BLE: Stop")
            elif button == '5':
                print("BLE: Forward")
            elif button == '6':
                print("BLE: Backward")
            elif button == '7':
                print("BLE: Left")
            elif button == '8':
                print("BLE: Right")
        else: # Button released — clear override only if it's the button we're tracking
            if _ble_active_button == button:
                _ble_active_button = None
                print("BLE: Release")



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
    __slots__ = ("_app", "_logging", "_ble", "_ble_controller", "_is_connected")

    def __init__(self, app, logging=False):
        self._app = app
        self._logging: bool = logging
        self._ble = bluetooth.BLE()
        self._ble_controller = None
        self._is_connected = False


    @property
    def logging(self) -> bool:
        """Whether to print debug logs from the bluetooth manager."""
        return self._logging

    @logging.setter
    def logging(self, value: bool):
        """Enable or disable debug logging for the bluetooth manager."""
        self._logging = value


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


    def start(self, name: str = "RobotXXX"):
        print(f"B:Initialising Bluetooth LE with name = {name}")

        # If the BLE controller is already initialized, deinitialize it first
        if self._ble_controller is not None:
            print("B:Deinitializing existing BLE controller...")
            self._ble_controller.deinit()
            self._ble_controller = None

        # Initialize the BLE controller
        try:
            self._ble_controller = RobotBLE(self._ble, name=name)
            if self.logging:
                print("B:BLE controller initialized")
        except Exception as e:
            print(f"B:Failed to initialize BLE controller: {e}")
            self._ble_controller = None

        # Register the command processor
        if self._ble_controller is not None:
            self._ble_controller.on_write(ble_process_command)

        if self.logging:
            print("B:BluetoothMgr initialised")


    def update(self, delta: int) -> None:
        app = self._app

        if self._ble_controller is not None:
            if self._ble_controller.is_connected() and not self._is_connected:
                self._is_connected = True
                app.show_notification(Notification("BLE Connected"))
            elif not self._ble_controller.is_connected() and self._is_connected:
                self._is_connected = False
                app.show_notification(Notification("BLE Disconnected"))

        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            # return to menu and disconnect from BLE if connected
            if self._ble_controller is not None:
                self._ble_controller.deinit()
                self._ble_controller = None
            app.return_to_menu()
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            # return to menu leaving the BLE connection active so the user can continue to control the robot from the phone app
            app.return_to_menu()


    def draw(self, ctx) -> bool:
        """Draw the Bluetooth manager's UI elements.  This should be called in the main loop's draw phase."""
        app = self._app
        # if connected then show a message to the user that they can control the robot with the Bluefruit LE app
        if self._is_connected:
            app.draw_message(ctx, ["BLE connected", "use Bluefruit LE", "to control"], [(0.5, 1, 0.5), (0.5, 1, 0.5), (0.5, 1, 0.5)], label_font_size)
        else:
            app.draw_message(ctx, ["BLE enabled", "use Bluefruit LE", "to connect"], [(1, 1, 0), (1, 1, 0), (1, 1, 0)], label_font_size)

        button_labels(ctx, confirm_label="OK", cancel_label="Exit")
        return True


# TODO - some sort of pairing / bonding / security mechanism to prevent random phones from controlling the robot.
