# MicroPython BLE Robot Control

import bluetooth
import struct
import time
from micropython import const

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


class RobotBLE:
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
        # Track connections and handle data reception
        if event == 1:  # _IRQ_CENTRAL_CONNECT
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("BLE:Connected")
        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            self._advertise()
            print("BLE:Disconnected")
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
            except Exception:
                pass
            
    def on_write(self, callback):
        self._write_callback = callback

    def _advertising_payload(self, name=None, services=None):

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


# --- Robot Logic ---
def ble_process_command(data):
    """
    Bluefruit Connect Control Pad sends data in the format:
    !B <button_number> <1=pressed/0=released> <checksum>
    Example: b'!B516' is Up Button Pressed
    """
    command = data.decode().strip()
    if not command.startswith("!B"):
        return

    # Check button number and press state
    button = command[2]
    action = command[3] # '1' for press, '0' for release

    if action == '1': # Only act on button press
        if button == '5':
            print("ACTION: Moving Forward")
        elif button == '6':
            print("ACTION: Moving Backward")
        elif button == '7':
            print("ACTION: Turning Left")
        elif button == '8':
            print("ACTION: Turning Right")
    else:
        print("ACTION: Stopping Motors")
