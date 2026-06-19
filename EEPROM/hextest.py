"""HexTest Hexpansion App for BadgeBot."""

# This is the app to be installed from the HexTest Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import asyncio
import time

try:
    from typing import TYPE_CHECKING
except ImportError:
    TYPE_CHECKING = False

import ota
import vfs
from machine import I2C, Pin, mem32, disable_irq, enable_irq

import settings as platform_settings
from app_components import Menu, button_labels, clear_background, label_font_size
from app_components.notification import Notification
from events.input import BUTTON_TYPES, Buttons

from system.eventbus import eventbus
from system.hexpansion import app as hexpansion_app
from system.hexpansion.config import HexpansionConfig
from system.hexpansion.events import HexpansionMountedEvent, HexpansionRemovalEvent
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent,
                                     RequestStopAppEvent)
from system.hexpansion.header import HexpansionHeader
from system.scheduler import scheduler
from system.hexpansion.util import (
    detect_eeprom_addr,
    get_hexpansion_block_devices,
)
try:
    from system.hexpansion.util import get_app_by_slot, get_slots_by_vid_pid
except ImportError:
    def get_app_by_slot(slot):
        """Find the app instance running from the hexpansion on the given port, if any.  Returns the app instance if found, None otherwise."""
        for an_app in scheduler.apps:
            if hasattr(an_app, "config"):
                if hasattr(an_app.config, "port") and an_app.config.port == slot:
                    return an_app
        return None

    def get_slots_by_vid_pid(vid, pid):
        """Find all hexpansion ports with the given VID/PID. Returns a list of port numbers."""
        ports_with_hexpansion = []
        for port in range(1, 1+_SLOTS):
            try:
                i2c = I2C(port)   # port is 0-indexed in code but 1-indexed in hardware
                # Autodetect eeprom addr
                eeprom_addr, addr_len = detect_eeprom_addr(i2c)
                if eeprom_addr is None or addr_len is None:
                    continue
                # Do we have a header?
                header_bytes = i2c.readfrom_mem(eeprom_addr, 0x00, 32, addrsize=8*addr_len)
                hexpansion_header = HexpansionHeader.from_bytes(header_bytes)
            except OSError:
                # OSError just means there is no hexpansion EEPROM on this port
                continue
            except RuntimeError:
                # RuntimeError means there is a blank EEPROM
                continue
            except Exception:      # pylint: disable=broad-except
                continue
            if hexpansion_header is None:
                continue
            if hexpansion_header.vid == vid and hexpansion_header.pid == pid:
                ports_with_hexpansion.append(port)
        return ports_with_hexpansion



import app
from tildagon import Pin as ePin
try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is an identity function on MicroPython
    const = lambda x: x  # noqa: E731

if TYPE_CHECKING:
    from typing import cast
    from hexdrive_protocol import HexDriveLike

    def _as_hexdrive_app(value: object) -> HexDriveLike:
        return cast(HexDriveLike, value)
else:
    HexDriveLike = object

    def _as_hexdrive_app(value):
        return value


# Define the minimum BadgeOS version required to run this app (e.g. if we need features that are only available in a certain version of BadgeOS)
_MIN_BADGEOS_VERSION = [2, 0, 0]     # v2.0.0 is required to be able to read the EEPROM with 16-bit addressing

_PRE = "hextest."  # Prefix for settings keys in EEPROM

# HexTest Hexpansion constants
# Hardware defintions:
_SLOTS = const(6)

# I2S pin to signal mapping for each known I2S capable hexpansion by VID/PID
# ==========================================================================
# Different hexpansions use different mappings of I2S signals to the HS pins.
# We can identify them from the hexpansion VID/PID and use the correct mapping for each one.
# This defines the mapping of I2S signals to the HS pins for each known I2S capable hexpansion by VID/PID.
_I2S_PIN_MAPPINGS = {
    (0xCBCB, 0x2000): (2,3,0),  # HexAudio: SCK=HS_H, WS=HS_I, SD=HS_F
}
_I2S_RATE = const(22050)  # I2S sample rate in Hz


# Constants for rotation rate measurement and motor test mode.
_ROTATION_RATE_MEASUREMENT_PERIOD_MS = const(3000)     # how often to update the displayed rotation rate measurement in ms (tradeoff between display responsiveness and stability of the reading)
_DEFAULT_ROTATION_RATE_EMITTER_DUTY = const(10)        # default duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
_DEFAULT_SPOKES_PER_ROTATION = const(10)               # number of times the photodiode will be triggered per full rotation of the wheel
_ROTATION_RATE_EMITTER_PINS = [const(1), const(2)]            # LS_B & LS_C pins used to drive the IR emitter for rotation rate testing
_ROTATION_RATE_SENSOR_PINS = [const(0), const(1)]             # HS_F & HS_G pins used to read the phototransistors for rotation rate testing
_ROTATION_RATE_SENSOR_ENABLE_PINS = [const(3), const(4)]      # LS_D & LS_E pins used to enable the phototransistors for rotation rate testing (set to output and high to enable, input to disable)
_IR_EMITTER_PWM_STEP_SIZE = const(2)                   # Step size for adjusting IR emitter brightness in manual mode, 0-255 (0=off, 255=full on)
_MAX_POWER = const(65535)                              # maximum power level to apply to motors, corresponds to 100% on the HexDrive
_MOTOR_PWM_FREQUENCY = const(20000)                    # Default PWM frequency to set on the HexDrive for testing, in Hz.

# Rotation Rate Auto scan configuration
_AUTO_SCAN_STEPS       = const(45)     # Number of power levels to test during auto scan
_AUTO_SCAN_SETTLE_MS   = const(800)    # ms to wait after setting power before starting actual measurement period
_AUTO_SCAN_MEASURE_MS  = const(10000)  # ms measurement window per step (maximum)
_AUTO_SCAN_MIN_POWER   = const(35000)  # Minimum power level to test during auto scan (0-65535)
_FILE = "motor"
_EXT  = "csv"
_FILE_DEST_LABELS = ("Badge FS", "Hex FS")

# App states
STATE_MENU = const(0)
STATE_MESSAGE = const(1)         # Message display
STATE_SETTINGS = const(2)        # Edit Settings
STATE_SENSOR = const(3)          # Sensor Test
STATE_MOTOR_TEST = const(4)      # Motor Test
STATE_WAITING_FOR_HEXDRIVE = const(5)  # Waiting for HexDrive to be mounted so we can start motor test

# App states where user can minimise app (Menu, Message, Logo)
MINIMISE_VALID_STATES = [STATE_MENU, STATE_MESSAGE, STATE_WAITING_FOR_HEXDRIVE]

# Main Menu Items
MAIN_MENU_ITEMS = ["Sensor Test", "Motor Test", "Settings", "About","Exit"]
MENU_ITEM_SENSOR_TEST = const(0)
MENU_ITEM_MOTOR_TEST = const(1)
MENU_ITEM_SETTINGS = const(2)
MENU_ITEM_ABOUT = const(3)
MENU_ITEM_EXIT = const(4)

DEFAULT_BACKGROUND_UPDATE_PERIOD = const(100)    # mS when not moving
_LOGGING = True
_AUTO_REPEAT_MS = const(200)       # Time between auto-repeats, in ms
_AUTO_REPEAT_COUNT_THRES = const(10) # Number of auto-repeats before increasing level
_AUTO_REPEAT_SPEED_LEVEL_MAX = const(4)  # Maximum level of auto-repeat speed increases
_AUTO_REPEAT_LEVEL_MAX = const(3)  # Maximum level of auto-repeat digit increases


# Pages of information to show for each sensor (can be switched with up/down buttons)
_PAGE_RAW = const(0)
_PAGE_STATS = const(1)
_PAGE_DATA = const(2)
_PAGE_CAL = const(3)
_PAGE_NAMES = {
    _PAGE_RAW: "Raw",
    _PAGE_STATS: "Stats",
    _PAGE_DATA: "Data",
    _PAGE_CAL: "Cal",
}

# Badge hexpansion HS pin to ESP32-S3 GPIO number mapping
# HexpansionConfig(port).pin[i] is Pin(gpio) where gpio = _HS_PIN_TO_GPIO[port][i]
_HS_PIN_TO_GPIO = {
    1: (39, 40, 41, 42),
    2: (35, 36, 37, 38),
    3: (34, 33, 47, 48),
    4: (11, 14, 13, 12),
    5: (18, 16, 15, 17),
    6: ( 3,  4,  5,  6),
}


class HexTestApp(app.App):         # pylint: disable=no-member
    """ HexTest Hexpansion App for BadgeBot."""
    VERSION = 2         # Increment this when making changes to the app that require the hexpansion EEPROM app to be re-flashed with the new code.

    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()
        if config is None:
            raise ValueError("HexTestApp requires a HexpansionConfig on initialisation")

        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            #print(f"HT:S/W {ver}")
            # e.g. v1.9.0-beta.1
            if ver >= _MIN_BADGEOS_VERSION:
                pass
            else:
                raise RuntimeError("HexTestApp requires BadgeOS Upgrade")
        except Exception as e:      # pylint: disable=broad-except
            print(f"HT:Ver check failed {e}!")

        self.config: HexpansionConfig = config
        self._logging: bool = True
        self._foreground = False

        # UI Button Controls
        self.button_states = Buttons(self)
        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_MENU
        self.previous_state = self.current_state
        self.update_period: int = DEFAULT_BACKGROUND_UPDATE_PERIOD   # mS

        # UI Feature Controls
        self.refresh: bool = True            # True so that we draw initial screen on first loop, then set to True whenever we want to trigger a screen update
        self.notification: Notification | None = None
        self.message: list = []
        self.message_colours: list = []
        self.message_type: str | None = None
        self.message_return_state: int | None = None
        self.current_menu: str | None = None
        self.menu: Menu | None = None

        # Settings
        self.settings: dict = {}
        self.edit_setting = None
        self.edit_setting_value = None
        self._auto_repeat_intervals = [ _AUTO_REPEAT_MS, _AUTO_REPEAT_MS//2, _AUTO_REPEAT_MS//4, _AUTO_REPEAT_MS//8, _AUTO_REPEAT_MS//16] # at the top end the loop is unlikley to cycle this fast
        self._auto_repeat: int = 0
        self._auto_repeat_count: int = 0
        self.auto_repeat_level: int = 0

        self._hexdrive_ports: list[int] = []
        self._hexdrive_in_use_port: int | None = None
        self._hexdrive_app: HexDriveLike | None = None

        self._rotation_rate_emitter_duty: int = _DEFAULT_ROTATION_RATE_EMITTER_DUTY # duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
        self._rotation_rate_counters: list[Counter] = []                           # hardware counters used to count photodiode pulses for rate testing
        self._rotation_rate_rpms: list[int] = []                    # computed RPM values derived from counter deltas
        self._rotation_rate_measurement_period: int = _ROTATION_RATE_MEASUREMENT_PERIOD_MS
        self._rotation_rate_measurement_period_elapsed: int = 0     # ticks since last rate check, used to compute pulse rate in Hz based on the change in the counter value
        self._rotation_rate_motor_power: int = 0                    # Power applied to motors in TEST mode

        # Auto scan test
        self._scan_mode: bool = False             # True = auto scanning, False = manual
        self._unsaved_data = False
        self._scan_direction: int = 1             # 1 = forwards, -1 = reverse
        self._scan_decrease_power: bool = False   # True = decreasing power, False = increasing power
        self._scan_step: int = 0                  # current step index (0.._AUTO_SCAN_STEPS-1)
        self._capture_settling: bool = True          # True = in settle phase, False = in measure phase
        self._rotation_detected: bool = False     # True once motion has been observed during auto scan
        self._capture_data: list[tuple[int, list[int], int | None, int]] = []   # list of (power, rpm list, current mA, decreasing)
        self._max_rpm: int = 0               # max rpm seen during scan
        self._max_current_ma: int = 0        # max current seen during scan
        self._last_current_ma: int = 0       # latest current sampled in auto mode
        self._scan_done: bool = False             # True = scan complete
        self._motor_calibration_fit: list[tuple[float, float] | None] = []  # list of (slope, intercept) fits, indexed by motor number
        self._hut_id: int  = 0              # ID of the hexpansion under test (HUT) to include in the auto scan results file name
        self._hut_id_seeded_ports = set()   # ports already seen this mount cycle, so we only seed from UID once per detection

        self._ina226 = None
        self._ina226_sensor_mgr = None  # SensorManager used exclusively for motor-test INA226 discovery
        self._ina226_reading: dict[str, int] = {}
        self._ina226_sum_current_ma: int = 0
        self._ina226_sum_bus_mv: int = 0
        self._ina226_sample_count: int = 0

        self._sensor_data: dict = {}
        self._display_data: dict = {}
        self._page_selected: int = _PAGE_RAW
        self._page_count: int = 3
        self._read_timer: int = 0    # ms since last sensor read
        self._sample_count: int = 0
        self._count_timer: int = 0  # ms
        self._sample_rate: int = 0  # Hz
        self._new_sample: bool = False

        # Audio
        self._hexaudio_ports: list[int] = []
        self._hexaudio_in_use_port: int | None = None
        self._sequencer: AsyncI2SSequencer | None = None

        # Settings
        if MySetting is not None:
            # General settings
            self.settings['logging']       = MySetting(self.settings, _LOGGING, False, True)
            self.settings['path']          = MySetting(self.settings, 0, 0, len(_FILE_DEST_LABELS) - 1, labels=_FILE_DEST_LABELS)
            self.settings['serialise']     = MySetting(self.settings, True, False, True)
            self.settings['ir_pwm']        = MySetting(self.settings, _DEFAULT_ROTATION_RATE_EMITTER_DUTY, 0, 255)
            self.settings['spokes']        = MySetting(self.settings, _DEFAULT_SPOKES_PER_ROTATION, 1, 20)
            self.settings['volume']        = MySetting(self.settings, 4, 0, 10)

            self.update_settings()

        self.HEXDRIVE_TYPES = [HexpansionType(0xCBCB, "HexDrive",                motors=2, servos=4, sub_type="Uncommitted" ),
                                 HexpansionType(0xCBCA, "HexDrive",                motors=2,           sub_type="2 Motor" ),
                                 HexpansionType(0xCBCC, "HexDrive",                          servos=4, sub_type="4 Servo" ),
                                 HexpansionType(0xCBCD, "HexDrive",                motors=1, servos=2, sub_type="1 Mot 2 Srvo" ),
                                 HexpansionType(0x10C8, "HexDrive2",   vid=0xCBCB, motors=2, servos=2, sub_type="Uncommitted" ),
                                 HexpansionType(0x10C9, "HexDrive2",   vid=0xCBCB,           servos=2, sub_type="2 Servo" ),
                                 HexpansionType(0x10CA, "HexDrive2",   vid=0xCBCB, motors=2,           sub_type="2 Motor" ),
                                 HexpansionType(0x11CE, "HexDrive2",   vid=0xCBCB, motors=1,           sub_type="Left Motor" ),
                                 HexpansionType(0x12CE, "HexDrive2",   vid=0xCBCB, motors=1,           sub_type="Right Motor" ),
                                 HexpansionType(0x10CF, "HexDrive2",   vid=0xCBCB, motors=1, servos=1, sub_type="1 Mot 1 Srvo" )]

        # report app starting and which port it is running on
        print(f"HT:HexTest App V{self.VERSION} by RobotMad on port {self.config.port}")

        self._rotation_rate_enable(False)  # start with rotation rate emitter and sensors off until we enter motor test mode

        # I2S sound output
        for vid, pid in _I2S_PIN_MAPPINGS.keys():
            ports = get_slots_by_vid_pid(vid, pid)
            if ports:
                # Only initialise the first HexAudio found, but keep track of all ports with HexAudio for future use
                if self._hexaudio_in_use_port is None:
                    self._initialise_hexaudio(ports[0], _I2S_PIN_MAPPINGS[(vid, pid)])
                self._hexaudio_ports.extend(ports)

        self._pitch = 20 # test tone generation - initial note

        # Event handlers for gaining and losing focus
        eventbus.on_async(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self._lose_focus, self)
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)
        eventbus.on_async(HexpansionMountedEvent, self._handle_mounted, self)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_removal, self)

        # Start with a Message state to show the app name and version
        self.show_message(["HexTest", f"V{self.VERSION}", "By RobotMad"], [(0.2,1,0.2), (1,1,0), (1,1,1)], return_state=STATE_MENU)



    # ------------------------------------------------------------------

    @property
    def logging(self):
        """Convenience property to access logging setting."""
        if 'logging' in self.settings:
            return self.settings['logging'].v
        return True

    # ------------------------------------------------------------------

    @property
    def rotation_rate_rounding(self) -> int:
        return (self._rotation_rate_measurement_period * self.rotation_rate_spokes) // 2

    @property
    def rotation_rate_emitter_duty(self) -> int:
        """Duty cycle (0-255) for the IR emitter when doing rotation rate testing."""
        if 'ir_pwm' in self.settings:
            return self.settings['ir_pwm'].v
        return _DEFAULT_ROTATION_RATE_EMITTER_DUTY

    @rotation_rate_emitter_duty.setter
    def rotation_rate_emitter_duty(self, value: int):
        self.settings['ir_pwm'].v = value
        if self.config is not None:
            for pin_num in _ROTATION_RATE_EMITTER_PINS:
                self.config.ls_pin[pin_num].duty(value)

    @property
    def rotation_rate_spokes(self) -> int:
        """Number of times the photodiode will be triggered per full rotation of the wheel."""
        if 'spokes' in self.settings:
            return self.settings['spokes'].v
        return _DEFAULT_SPOKES_PER_ROTATION

    # ------------------------------------------------------------------

    def update_settings(self):
        """Update settings from EEPROM."""
        if self.logging:
            print("HT:Updating settings from EEPROM")
        for s in self.settings:
            self.settings[s].v = platform_settings.get(f"{_PRE}{s}", self.settings[s].d)
            if self.logging:
                print(f"Setting {s} = {self.settings[s].v}")
        if self._sequencer is not None:
            self._sequencer.volume = self.settings['volume'].v


    def _rotation_rate_enable(self, enable: bool = True) -> bool:
        if self.config is None:
            return False
        try:
            if enable:
                if self.logging:
                    print("HT:Enabling rotation rate emitters and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self.config.ls_pin[pin_num].init(mode=ePin.PWM)  # Set LS pins to output mode to turn on the IR emitters
                    self.config.ls_pin[pin_num].duty(self.rotation_rate_emitter_duty)  # Set LS pins to the current duty cycle to drive the IR emitters)
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self.config.ls_pin[pin_num].init(mode=Pin.OUT)  # Set LS pins to output mode to enable the phototransistors for rotation rate measurement
                    self.config.ls_pin[pin_num].value(1)  # Set LS enable pins high to turn on the phototransistors for rotation rate measurement
            else:
                if self.logging:
                    print("HT:Disabling rotation rate emitters and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self.config.ls_pin[pin_num].init(mode=Pin.IN)  # Set LS pins to input mode to turn off the IR emitters
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self.config.ls_pin[pin_num].init(mode=Pin.IN)  # Set LS pins to input mode to turn off the phototransistors for rotation rate measurement

            for pin_num in _ROTATION_RATE_SENSOR_PINS:
                self.config.pin[pin_num].init(mode=Pin.IN)  # Set HS pins to input mode to read the phototransistors for rotation rate measurement
        except AttributeError:
            pass  # Simulator Pin stubs lack .init()
        return True



    def deinit(self) -> bool:
        """ De-initialise the app - return True if successful, False if failed."""
        eventbus.remove(HexpansionMountedEvent, self._handle_mounted, self)
        eventbus.remove(HexpansionRemovalEvent, self._handle_removal, self)
        eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
        # deinit any allocated Counters
        for counter in self._rotation_rate_counters:
            counter.deinit()
        for hs_pin in self.config.pin:
            hs_pin.init(mode=Pin.IN)
        if self._sequencer is not None:
            self._sequencer.deinit()
            self._sequencer = None

        return True


    def _exit_app(self):
        """ Clean up and exit the app, returning to the main menu."""
        eventbus.emit(RequestStopAppEvent(self))


    def _initialise_hexaudio(self, port: int, pin_mapping: tuple[int, int, int] | None = None):
        """Initialise the HexAudio I2S sequencer if a HexAudio is detected on any port."""
        if pin_mapping is None:
            if hexpansion_app is not None:
                if hasattr(hexpansion_app, "_hexpansion_manager"):
                    manager = hexpansion_app._hexpansion_manager        # pylint: disable=protected-access
                    if manager is not None:
                        header = manager.hexpansion_headers[port]
                        if header is not None:
                            pin_mapping = _I2S_PIN_MAPPINGS.get((header.vid, header.pid), None)
            if pin_mapping is None:
                print(f"HT:{self.config.port}:I2S initialization failed: No pin mapping for HexAudio on port {port}")
                return
        # Re-initialize the I2S sequencer with the (new) port
        if port in _HS_PIN_TO_GPIO:
            sck_pin = _HS_PIN_TO_GPIO[port][pin_mapping[0]]  # I2S clock pin
            ws_pin  = _HS_PIN_TO_GPIO[port][pin_mapping[1]]   # I2S word select pin
            sd_pin  = _HS_PIN_TO_GPIO[port][pin_mapping[2]]   # I2S data pin
            try:
                self._sequencer = AsyncI2SSequencer(i2s_id=0, sck_pin=sck_pin, ws_pin=ws_pin, sd_pin=sd_pin, sample_rate=_I2S_RATE, volume=self.settings['volume'].v)
                self._hexaudio_in_use_port = port
            except Exception as e:      # pylint: disable=broad-except
                print(f"HT:{self.config.port}:I2S initialization failed: {e}")
        else:
            print(f"HT:{self.config.port}:I2S initialization failed: Invalid port {port} for HexAudio")


    # ------------------------------------------------------------------
    # Async event handlers (registered directly on eventbus)
    # ------------------------------------------------------------------

    async def _handle_removal(self, event: HexpansionRemovalEvent):
        self._hut_id_seeded_ports.discard(event.port)
        if event.port == self._hexaudio_in_use_port:
            if self.logging:
                print(f"HT:HexAudio removed from port {event.port}")
            if event.port in self._hexaudio_ports:  # possible that the port was never added to the list if it was default port and never detected as HexAudio
                self._hexaudio_ports.remove(event.port)
            if self._sequencer is not None:
                self._sequencer.deinit()
                self._sequencer = None
            self._hexaudio_in_use_port = None
            if 0 < len(self._hexaudio_ports):
                self._initialise_hexaudio(self._hexaudio_ports[0])
        if event.port == self._hexdrive_in_use_port:
            if self.logging:
                print(f"HT:HexDrive removed from port {event.port}")
            self._hexdrive_app = None
            self._hexdrive_in_use_port = None
            self._hexdrive_ports.remove(event.port)
            self.notification = Notification("HexDrive Removed")
            if self.current_state == STATE_MOTOR_TEST:
                self._stop_motor_test_mode()
            elif self.current_state == STATE_SENSOR:
                self._stop_sensor_test_mode()
            if 0 < len(self._hexdrive_ports):
                self._hexdrive_in_use_port = self._hexdrive_ports[0]
                if self.logging:
                    print(f"HT:HexDrive switched to port {self._hexdrive_in_use_port}")
                # Attempt to find the HexDrive app on the new port
                hexdrive_app = get_app_by_slot(self._hexdrive_in_use_port)
                if hexdrive_app is not None:
                    self._hexdrive_app = _as_hexdrive_app(hexdrive_app)
                    if self.logging:
                        print(f"HT:HexDrive app found on port {self._hexdrive_in_use_port}")
                else:
                    if self.logging:
                        print(f"HT:No HexDrive app found on port {self._hexdrive_in_use_port}")


    async def _handle_mounted(self, event: HexpansionMountedEvent):
        if self.logging:
            print(f"HT:Hexpansion mounted on port {event.port}")
        # Check if it is a HexDrive we can use for testing
        # make a simple list of vid, pid pairs that we can check against efficiently
        vid_pid_pairs = [(type.vid, type.pid) for type in self.HEXDRIVE_TYPES]
        header = getattr(event, "header", None)
        if header is not None and (header.vid, header.pid) in vid_pid_pairs:
            self._hexdrive_ports.append(event.port)
            if self._foreground and self.current_state in [STATE_WAITING_FOR_HEXDRIVE]:
                if self.logging:
                    print(f"HT:Attempting to use newly mounted HexDrive on port {event.port}")
                if self._motor_test_start():
                    if self.logging:
                        print(f"HT:Successfully started motor test with newly mounted HexDrive on port {event.port}")
                    self.current_state = STATE_MOTOR_TEST
                    #self.set_menu(None)
                else:
                    if self.logging:
                        print(f"HT:Failed to start motor test with newly mounted HexDrive on port {event.port}")
        elif header is not None and (header.vid, header.pid) in _I2S_PIN_MAPPINGS.keys():
            # HexAudio mounted
            if self.logging:
                print(f"HT:HexAudio mounted on port {event.port} (VID:PID {header.vid:04X}:{header.pid:04X})")
                if event.port not in self._hexdrive_ports:
                    self._hexaudio_ports.append(event.port)
                if self._hexaudio_in_use_port is None:
                    self._initialise_hexaudio(event.port, _I2S_PIN_MAPPINGS[(header.vid, header.pid)])


    async def _gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            if self.logging:
                print(f"HexTest gained focus in state {self.current_state}")
            self._foreground = True
            if self.current_state == STATE_WAITING_FOR_HEXDRIVE:
                if self._motor_test_start():
                    self.current_state = STATE_MOTOR_TEST


    async def _lose_focus(self, event: RequestForegroundPopEvent):
        if event.app is self:
            if self.logging:
                print(f"HexTest lost focus from state {self.current_state}")
            self._foreground = False


    async def _handle_stop_app(self, event):
        """ Handle the RequestStopAppEvent so that we can release resources """
        try:
            if event.app == self:
                if self._logging:
                    print(f"HT:{self.config.port}:Stop")
                self.deinit()
        except (AttributeError, TypeError):
            pass


    async def background_task(self):
        """Background task loop for handling time-based updates. This runs independently of the main update/draw loop
           and is suitable for tasks that need to run at a consistent interval regardless of the current state or drawing performance."""
        last_time: int = time.ticks_ms()

        while True:
            cur_time: int = time.ticks_ms()
            delta_ticks: int = time.ticks_diff(cur_time, last_time)
            self._background_update(delta_ticks)
            await asyncio.sleep_ms(max (1, self.update_period - (time.ticks_ms() - cur_time)))  # sleep for the remainder of the update period, accounting for time taken by background_update
            last_time = cur_time


    ### NON-ASYNC FUNCTIONS ###

    def _background_update(self, delta: int):
        """Perform background updates based on the current sub-state."""
        self._sample_ina226_in_background(delta)
        # push motor outputs to HexDrive if we are in motor test mode
        if self.current_state == STATE_MOTOR_TEST:
            if self._hexdrive_app is not None:
                try:
                    if not self._hexdrive_app.set_motors((self._rotation_rate_motor_power, self._rotation_rate_motor_power)):
                        if self.logging:
                            print("Failed to set motor outputs to HexDrive app")
                except AttributeError:
                    pass


    def set_logging(self, state: bool):
        """ Set the logging state - True to enable logging, False to disable logging. """
        self._logging = state


    # multi level auto repeat
    def auto_repeat_check(self, delta: int, speed_up: bool = True) -> bool:
        """Check if the auto-repeat threshold has been reached for a button hold, and update the auto-repeat level accordingly.
           If speed_up is True, the auto-repeat interval decreases as the level increases, allowing for faster repeats the
           longer the button is held. If speed_up is False, the interval remains constant, but the level can still increase
           to allow for larger increments/decrements in settings adjustments.
           Returns True if the auto-repeat action should be triggered, False otherwise.
        """
        self._auto_repeat += delta
        # multi stage auto repeat - the repeat gets faster the longer the button is held
        if self._auto_repeat > self._auto_repeat_intervals[self.auto_repeat_level if speed_up else 0]:
            self._auto_repeat = 0
            self._auto_repeat_count += 1
            # variable threshold to count to increase level so that it is not too easy to get to the highest level as the auto repeat period is reduced
            if self._auto_repeat_count > ((_AUTO_REPEAT_COUNT_THRES*_AUTO_REPEAT_MS) // self._auto_repeat_intervals[self.auto_repeat_level if speed_up else 0]):
                self._auto_repeat_count = 0
                if self.auto_repeat_level < (_AUTO_REPEAT_SPEED_LEVEL_MAX if speed_up else _AUTO_REPEAT_LEVEL_MAX):
                    self.auto_repeat_level += 1
                    if self.logging:
                        print(f"Auto Repeat Level: {self.auto_repeat_level}")

            return True
        return False


    def auto_repeat_clear(self):
        """Reset the auto-repeat counters and level. This should be called when a button is released to ensure that the next button press starts with the initial auto-repeat interval and level."""
        self._auto_repeat = 1+ self._auto_repeat_intervals[0] # so that we trigger immediately on next press

        self._auto_repeat_count = 0
        self.auto_repeat_level = 0


    # ------------------------------------------------------------------
    # MOTOR TEST Entry point from menu
    # ------------------------------------------------------------------

    def _motor_test_start(self) -> bool:
        """Enter the Sensor Test flow from the main menu."""
        self._sensor_data = {}
        self._display_data = {}
        self.refresh = True
        hexpansion_type: HexpansionType | None = None
        # Find HexDrive to test
        # look for any type of hexdrive (including HexDrive2 variants) in any port by their VID/PID
        for hexpansion_type in self.HEXDRIVE_TYPES:
            if self.logging:
                print(f"HT:Looking for {hexpansion_type.name} (VID:PID {hexpansion_type.vid:04X}:{hexpansion_type.pid:04X}, Motors: {hexpansion_type.motors}, Servos: {hexpansion_type.servos})")
            ports = get_slots_by_vid_pid(hexpansion_type.vid, hexpansion_type.pid)
            if ports:
                if self.logging:
                    print(f"HT:Found {hexpansion_type.name} on port(s): {ports}")
                self._hexdrive_ports.extend(ports)
                break

        for port in self._hexdrive_ports:
            hexdrive_app = _as_hexdrive_app(get_app_by_slot(port))
            if hexdrive_app is not None:
                self._hexdrive_in_use_port = port
                if self.logging:
                    print(f"HT:Found HexDrive app to test on port {port}")
                self._hexdrive_app = hexdrive_app
                break

        if self._hexdrive_app is not None:
            # First detect for this mounted hexpansion: seed HUT ID from non-zero UID once.
            self._seed_hut_id_from_detected_hexdrive(self._hexdrive_in_use_port)

            #Setup UUT = HexDrive
            try:
                if self._hexdrive_app.initialise() and self._hexdrive_app.set_power(True) and self._hexdrive_app.set_freq(_MOTOR_PWM_FREQUENCY):
                    self._hexdrive_app.set_logging(self.logging)
                    self._hexdrive_app.set_keep_alive(2000)   # Updates can be quite slow as we are using the draw function
            except AttributeError as e:
                print(f"HT:Failed to initialise HexDrive app for motor test mode: {e}")

            # Setup INA226:
            if self._init_ina226_for_motor_test():
                if self._ina226_sensor_mgr is not None:
                    self.update_period = self._ina226_sensor_mgr.read_interval   # update at the sensor read interval

            # Enable the IR emitter for measuring wheel rotation rate
            self._rotation_rate_enable(True)

            # Enable the phototransistor input for measuring wheel rotation rate
            for pin_num in _ROTATION_RATE_SENSOR_PINS:
                # configure the ESP32S3 hardware to count pulses on the HS pin(s)
                # Counter not yet available in this Micropython port so we have created our own...
                gpio_num = _HS_PIN_TO_GPIO[self.config.port][pin_num]
                # Is there a motor to test that we can measure the rotation rate of with the phototransistor sensors?
                # If so, set up a counter to measure the rotation rate based on the phototransistor pulses.
                # Assume first rotation rate sensor pin is for motor 1 and second (if present) is for motor 2.
                # could be extended to cope with HexDrive variants for Left and Right motors...
                if hexpansion_type is not None and hexpansion_type.motors < (pin_num + 1):
                    if self.logging:
                        print(f"HT:Not setting up rotation rate counter on pin {pin_num} (GPIO {gpio_num}) as this HexDrive type only has {hexpansion_type.motors} motors")
                    continue
                counter = Counter(None, gpio_num, filter_ns=1000000, logging=False)  # auto-select PCNT unit
                if counter is not None and counter.unit is not None:
                    self._rotation_rate_counters.append(counter)
                else:
                    if self.logging:
                        print(f"HT:Failed to allocate PCNT counter for pin {pin_num} (GPIO {gpio_num})")
                    self.notification = Notification("PCNT Init     Failed")
                    # deinit any counters we did manage to create before returning
                    for c in self._rotation_rate_counters:
                        if c is not None:
                            c.deinit()
                    self._rotation_rate_counters = []
                    return False
                if self.logging:
                    print(f"HT:Rate counter {counter}")
            self._rotation_rate_measurement_period_elapsed = 0
            self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
            return True
        if self.logging:
            print("HT:Failed to initialise for motor test mode - no hexdrive to test")
        self.notification = Notification("HexDrive   not Found")
        if self._sequencer is not None:
            self._sequencer.play(SEQ_ERROR)
        return False


    def _get_header_for_port(self, port: int) -> HexpansionHeader | None:
        header = None
        if hexpansion_app is not None:
            if hasattr(hexpansion_app, "_hexpansion_manager"):
                manager = hexpansion_app._hexpansion_manager        # pylint: disable=protected-access
                if manager is not None:
                    header = manager.hexpansion_headers[port]
        return header


    def _seed_hut_id_from_detected_hexdrive(self, port: int | None, header: HexpansionHeader | None = None) -> None:
        if port is None or port in self._hut_id_seeded_ports:
            return

        if header is None:
            header = self._get_header_for_port(port)

        if header is not None:
            try:
                unique_id = int(header.unique_id)
            except (TypeError, ValueError):
                unique_id = 0

            if unique_id != 0:
                self._hut_id = unique_id
                if self._logging:
                    print(f"HT:Initialised HUT ID from UID {unique_id} on port {port}")

        # Mark this port as handled for the current mount cycle so user edits are preserved.
        self._hut_id_seeded_ports.add(port)



    def _stop_motor_test_mode(self):
        if self._logging:
            print("HT:Stopping Motor Test mode and cleaning up")
        self._scan_mode = False
        self._scan_done = False
        self._rotation_rate_motor_power = 0
        self._ina226_reading = {}
        self._reset_ina226_accumulators()
        if self._ina226 is not None:
            if self._ina226_sensor_mgr is not None:
                try:
                    self._ina226_sensor_mgr.close()
                except Exception as exc:          # pylint: disable=broad-exception-caught
                    print("HT:INA226 sensor manager close failed:", exc)
                self._ina226_sensor_mgr = None
        self._ina226 = None

        if self._hexdrive_app is not None:
            try:
                self._hexdrive_app.set_motors((0, 0))
                self._hexdrive_app.set_power(False)
            except AttributeError as e:
                print(f"HT:Failed to set motor outputs off {e}")
        self._hexdrive_in_use_port = None

        for c in self._rotation_rate_counters:
            if c is not None:
                c.deinit()
        self._rotation_rate_counters = []
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        self._rotation_rate_enable(False)
        self.return_to_menu()


    def _stop_sensor_test_mode(self):
        if self._logging:
            print("HT:Stopping Sensor Test mode and cleaning up")
        self._sensor_data = {}
        self._display_data = {}
        self._hexdrive_in_use_port = None
        self.return_to_menu()


    def _init_ina226_for_motor_test(self) -> bool:
        self._ina226 = None
        self._ina226_sensor_mgr = None
        self._ina226_reading = {}
        self._reset_ina226_accumulators()
        try:
            mgr = SensorManager(logging=self._logging)
            # The INA226 sensor can't be on a port with an EEPROM because that would clash with the UUT EEPROM.
            for port in range(1, 1+_SLOTS):
                if not mgr.open(port):
                    mgr.close()
                    if self._logging:
                        print(f"HT:INA226 - no sensors found on port {port}")
                    continue
                # Find the first INA226 sensor in the discovered list
                sensor = mgr.get_sensor_by_name("INA226")
                if sensor is not None:
                    self._ina226 = sensor
                    self._ina226_sensor_mgr = mgr
                    if self._logging:
                        print(f"HT:INA226 found @ 0x{sensor.i2c_addr:02X} on port {port}")
                    return True
                # No INA226 found; close the manager
                mgr.close()
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"HT:INA226 init failed: {e}")
        return False


    def _reset_ina226_accumulators(self) -> None:
        self._ina226_sum_current_ma = 0
        self._ina226_sum_bus_mv = 0
        self._ina226_sample_count = -1


    def _sample_ina226_in_background(self, delta: int) -> None:     # pylint: disable=unused-argument
        sensor = self._ina226
        if sensor is None:
            return
        data = sensor.read_sample_if_ready()
        if data is None:
            return
        try:
            if self._ina226_sample_count >= 0:
                # only use samples after the first one, to allow the INA226 to settle after a power change before we start accumulating data for averaging
                self._ina226_sum_current_ma += int(data.get("mA", 0))
                self._ina226_sum_bus_mv += int(data.get("mV", 0))
            self._ina226_sample_count += 1
        except Exception as e:       # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"HT:INA226 sample error: {e}")
            return


    def _consume_ina226_average(self) -> int | None:
        if self._ina226_sample_count <= 0:
            self._ina226_reading = {}
            return None
        count = self._ina226_sample_count
        current_ma = self._ina226_sum_current_ma // count
        voltage_mv = self._ina226_sum_bus_mv // count
        self._ina226_reading = {
            "mA": current_ma,
            "mV": voltage_mv,
        }
        self._reset_ina226_accumulators()
        return current_ma


    def _auto_rotation_rate_step(self):
        # Advance to next power level
        self._scan_step += 1
        self.refresh = True
        if self._scan_step >= _AUTO_SCAN_STEPS:
            if self._scan_decrease_power:
                self._scan_decrease_power = False
                if self._scan_direction == -1:
                    # Scan complete
                    print("HT:Completed scan")
                    self._scan_done = True
                    self._rotation_rate_motor_power = 0
                    #self._auto_fit_calculate()
                    self._save_capture_data_csv()
                    if self._sequencer is not None:
                        self._sequencer.play(SEQ_COMPLETE)
                    return
                else:
                    print("HT:Starting second scan pass in reverse direction")
                    self._scan_direction = -1  # reverse direction for second pass
                self._rotation_detected = False
            else:
                print("HT:Decreasing power")
                self._scan_decrease_power = True
            self._scan_step = 0
        # start measurement at the new power level
        step = self._scan_step if not self._scan_decrease_power else (_AUTO_SCAN_STEPS - 1 - self._scan_step)
        self._rotation_rate_motor_power = self._scan_direction * (_AUTO_SCAN_MIN_POWER + (((_MAX_POWER - _AUTO_SCAN_MIN_POWER) * step) // (_AUTO_SCAN_STEPS - 1)))
        self._rotation_rate_measurement_period_elapsed = 0
        self._capture_settling = True
        if self._sequencer is not None:
            self._sequencer.play(SEQ_KEYPRESS)


    def _data_save_path_option(self):
        try:
            return int(self.settings["path"].v)
        except Exception:      # pylint: disable=broad-exception-caught
            return 0


    def _mount_current_fs(self):
        if self.config is None or getattr(self.config, "port", None) is None:
            print("HT:Hex fs save unavailable when running from badge")
            return None, False
        mountpoint = "/hextest"
        header = self._get_header_for_port(self.config.port)
        if header is None:
            print("HT:Failed to read HexTest EEPROM header")
            return None, False
        try:
            eeprom_addr, addr_len = detect_eeprom_addr(self.config.i2c)
            if eeprom_addr is None or addr_len is None:
                return None, False
            _, partition = get_hexpansion_block_devices(self.config.i2c, header, eeprom_addr, addr_len=addr_len)
        except RuntimeError as exc:
            print(f"HT:Failed to get block device: {exc}")
            return None, False

        mounted_here = True
        try:
            vfs.mount(partition, mountpoint, readonly=False)
        except OSError as exc:
            if exc.args and exc.args[0] == 1:
                mounted_here = False
            else:
                print(f"HT:Failed to mount {mountpoint}: {exc}")
                return None, False
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"HT:Failed to mount {mountpoint}: {exc}")
            return None, False
        return mountpoint, mounted_here


    def _data_save_path(self):
        if self._data_save_path_option() == 1:
            mountpoint, mounted_here = self._mount_current_fs()
            if mountpoint is None:
                return None, None, False
            return f"{mountpoint}/{_FILE}{self._hut_id}.{_EXT}", mountpoint, mounted_here
        return f"/{_FILE}{self._hut_id}.{_EXT}", None, False


    def _save_capture_data_csv(self):
        if not self._capture_data and not self._unsaved_data:
            return False

        output_path, mountpoint, mounted_here = self._data_save_path()
        if output_path is None:
            self.notification = Notification("  Save   Failed")
            return False

        rpm_count = len(self._rotation_rate_rpms)
        header = ["power"] + [f"rpm{index + 1}" for index in range(rpm_count)] + ["ma"] + ["decreasing"]
        try:
            with open(output_path, "wb") as csv_file:
                csv_file.write((",".join(header) + "\n").encode())
                for power, rpms, current_ma, decreasing in self._capture_data:
                    row = [str(power)]
                    row.extend(str(rpm) for rpm in rpms)
                    row.append(str(current_ma))
                    row.append(str(decreasing))
                    csv_file.write((",".join(row) + "\n").encode())
        except Exception as exc:      # pylint: disable=broad-exception-caught
            print(f"HT:Failed to save CSV {output_path}: {exc}")
            self.notification = Notification("  Save   Failed")
            return False
        finally:
            if mounted_here and mountpoint is not None:
                try:
                    vfs.umount(mountpoint)
                except Exception as exc:      # pylint: disable=broad-exception-caught
                    print(f"HT:Failed to unmount {mountpoint}: {exc}")
        self._unsaved_data = False
        print(f"HT:Saved CSV to {output_path}")
        self.notification = Notification("   Data     Saved")
        # auto increment HUT ID for next save
        self._hut_id += 1
        return True


    # ------------------------------------------------------------------
    # HEXPANSION operations
    # ------------------------------------------------------------------


 ### MAIN APP CONTROL FUNCTIONS ###

    def update(self, delta: int):
        """Main update function called from the main loop. Handles state transitions, user input, and delegates to functional area managers."""
        if not self._foreground:
            # This triggers the automatic foreground display
            eventbus.emit(RequestForegroundPushEvent(self))
            #self._foreground = True

        if self.notification:
            self.notification.update(delta)
            try:
                # in case access to protected member _is_closed() is not allowed, we catch the exception and
                # to prevent crashes - this means that in this case we won't be able to automatically clear
                # notifications when they are closed, but at least the app won't crash.
                if self.notification._is_closed():  # pylint: disable=protected-access
                    self.notification = None
            except Exception as e:  # pylint: disable=broad-exception-caught
                if self.logging:
                    print(f"HT:Error: checking notification status: {e}")

        # Unfortunately, even though we can track if there is an active notification that we have triggered,
        # we don't have a way to track if there are any other notifications active that we
        # didn't trigger, so we need to perform extra display refresh cycles in case.
        # As the draw function is VERY slow, and hence it stalls background updates
        # we only do extra refresh cycles if the update period is long.
        if self.update_period >= DEFAULT_BACKGROUND_UPDATE_PERIOD:
            self.refresh = True

        ## DO STUFF BASED ON STATE ##
        if self.current_state == STATE_MENU:
            if self.current_menu is None:
                self.set_menu()
                self.refresh = True
            else:
                menu = self.menu
                if menu is None:
                    self.set_menu()
                    self.refresh = True
                    return
                menu.update(delta)
                if menu.is_animating != "none":
                    if self.logging:
                        print("HT:Menu is animating")
                    self.refresh = True
            # FOR TESTING
            if self._sequencer is not None:
                # Scan is complete - no further updates required - play sound every 30 seconds to indicate that the scan is complete and the user can exit.
                self._rotation_rate_measurement_period_elapsed += delta
                if self._rotation_rate_measurement_period_elapsed >= 30000:
                    self._rotation_rate_measurement_period_elapsed = 0
                    self._sequencer.play(SEQ_COMPLETE)

        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
            self.minimise()
        elif self.current_state == STATE_MESSAGE or self.current_state == STATE_WAITING_FOR_HEXDRIVE:
            self._update_state_message(delta)
        elif self.current_state == STATE_SETTINGS:
            self._settings_mgr_update(delta)
        elif self.current_state == STATE_MOTOR_TEST:
            self._motor_test_update(delta)


        if self.current_state != self.previous_state:
            if self.logging:
                print(f"HT:State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # something has changed - so worth redrawing
            self.refresh = True


    def _update_state_message(self, delta: int):      # pylint: disable=unused-argument
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            if self.logging:
                print("HT:Message acknowledged by user")
            self.button_states.clear()
            if self.message_type == "reboop":
                # Reboot has been acknowledged by the user - unfortunately we can't actually reboot the badge from Python.
                return # leave the message on screen.
            elif self.message_return_state is not None:
                self.current_state = self.message_return_state
            else:
                # Message has been acknowledged by the user - allow access to the menu
                # refresh the menu in case available options have changed
                self.set_menu()
                self.refresh = True
                self.current_state = STATE_MENU
            self.message = []
            self.message_colours = []
            self.message_type = None
            self.message_return_state = None
        # "CANCEL" button is handled in common for all MINIMISE_VALID_STATES so no custom code here



    def _motor_test_update(self, delta: int):  # pylint: disable=unused-argument
        # CANCEL always exits motor test mode
        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            #self._show_auto_results_fit()
            self._stop_motor_test_mode()
            return

        # CONFIRM toggles between manual and auto mode
        elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            self._last_current_ma = 0
            self._rotation_rate_measurement_period_elapsed = 0
            self._reset_ina226_accumulators()
            for counter in self._rotation_rate_counters:
                if counter is not None:
                    counter.value(0)      # reset counter
            if self._scan_mode:
                # Switch back to manual
                #self._show_auto_results_fit()
                self._rotation_rate_motor_power = 0
                self._rotation_rate_measurement_period = _ROTATION_RATE_MEASUREMENT_PERIOD_MS
                self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                self._ina226_reading = {}
                self._scan_mode = False
                self._scan_done = False
            else:
                # Start auto scan
                self._scan_mode = True
                self._scan_done = False
                self._scan_step = 0
                self._scan_decrease_power = False
                self._scan_direction = 1
                self._rotation_rate_motor_power = self._scan_direction * _AUTO_SCAN_MIN_POWER
                self._rotation_rate_measurement_period = _AUTO_SCAN_MEASURE_MS
                self._capture_settling = True
                self._capture_data = []
                self._unsaved_data = False
                self._max_rpm = 0
                self._ina226_reading = {}
                self._max_current_ma = 0
                self._rotation_detected = False
            self.refresh = True
            return

        if self._scan_mode:
            if not self._scan_done:
                self._rotation_rate_measurement_period_elapsed += delta
                if self._capture_settling:
                    if self._rotation_rate_measurement_period_elapsed >= _AUTO_SCAN_SETTLE_MS:
                        # Settle phase done — discard counter and start measuring
                        count = 0
                        for counter in self._rotation_rate_counters:
                            if counter is not None:
                                count += counter.value(0)  # read-and-reset to discard
                        if count == 0 and (not self._rotation_detected or self._scan_decrease_power):
                            # There has been no motion from any motors - so we can skip the measure phase and move straight to the next power level
                            current_ma = self._consume_ina226_average()
                            if current_ma is not None:
                                current_abs = abs(current_ma)
                                self._last_current_ma = current_ma
                                if current_abs > self._max_current_ma:
                                    self._max_current_ma = current_abs
                            power = self._rotation_rate_motor_power
                            self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                            if self._logging:
                                print(f"HT:Auto Scan Step {self._scan_step+1}/{_AUTO_SCAN_STEPS} - Power: {power}, Rate: 0 rpm, Current: {current_ma}mA")
                            self._capture_data.append((power, [0] * len(self._rotation_rate_counters), current_ma, self._scan_decrease_power))
                            if self._scan_decrease_power:
                                self._scan_step = _AUTO_SCAN_STEPS  #Force termination
                            self._auto_rotation_rate_step()
                            if not self._unsaved_data:
                                self._unsaved_data = True
                        else:
                            self._rotation_detected = True
                            # estimate how long we need to measure for based on the count we got during the settle period, to ensure we get a good RPM (2%)
                            # reading even at low speeds, while still keeping the overall scan time reasonable#
                            cpm = (60000 * count) // self._rotation_rate_measurement_period_elapsed # rounded down - never displayed
                            self._rotation_rate_measurement_period = min(_AUTO_SCAN_MEASURE_MS, (60000 * 50) // cpm) if cpm > 0 else _AUTO_SCAN_MEASURE_MS
                            self._rotation_rate_measurement_period_elapsed = 0
                            self._capture_settling = False
                            self._reset_ina226_accumulators()
                else:
                    if self._rotation_rate_measurement_period_elapsed >= self._rotation_rate_measurement_period:
                        # Measure phase done — read counter and record result
                        self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                        for index, counter in enumerate(self._rotation_rate_counters):
                            if counter is not None:
                                count = counter.value(0)
                                rpm = ((60000 * count) + self.rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self.rotation_rate_spokes)
                                if rpm > self._max_rpm:
                                    self._max_rpm = rpm
                                self._rotation_rate_rpms[index] = rpm

                        ### duplicate of block above - could be a method
                        current_ma = self._consume_ina226_average()
                        if current_ma is not None:
                            current_abs = abs(current_ma)
                            self._last_current_ma = current_ma
                            if current_abs > self._max_current_ma:
                                self._max_current_ma = current_abs
                        power = self._rotation_rate_motor_power
                        if self._logging:
                            print(f"HT:Auto Scan Step {self._scan_step+1}/{_AUTO_SCAN_STEPS} - Power: {power}, Rates: {self._rotation_rate_rpms} rpm, Current: {current_ma}mA")
                        self._capture_data.append((power, self._rotation_rate_rpms, current_ma, self._scan_decrease_power))
                        self._auto_rotation_rate_step()
                        if self._unsaved_data:
                            self._unsaved_data = True
            elif self._sequencer is not None:
                # Scan is complete - no further updates required - play sound every 30 seconds to indicate that the scan is complete and the user can exit.
                self._rotation_rate_measurement_period_elapsed += delta
                if self._rotation_rate_measurement_period_elapsed >= 30000:
                    self._rotation_rate_measurement_period_elapsed = 0
                    self._sequencer.play(SEQ_COMPLETE)


            # In auto mode, no manual button control for power/IR
            return
        else:
            # manual measurement mode
            self._rotation_rate_measurement_period_elapsed += delta
            if self._rotation_rate_measurement_period_elapsed >= self._rotation_rate_measurement_period:
                count = 0
                for index, counter in enumerate(self._rotation_rate_counters):
                    if counter is not None:
                        count = counter.value(0)  # read-and-reset to get the count for the elapsed period
                        self._rotation_rate_rpms[index] = ((60000 * count) + self.rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self.rotation_rate_spokes)
                self._rotation_rate_measurement_period_elapsed = 0
                self._consume_ina226_average()
                #if self.logging:
                #    print(f"HT:Rotation Rates: {self._rotation_rate_rpms}")

        # Manual mode button handling
        if self.button_states.get(BUTTON_TYPES["UP"]):
            self.button_states.clear()
            if self.settings['serialise'].v:
                self._hut_id += 1
            else:
                self.rotation_rate_emitter_duty = min(255, self.rotation_rate_emitter_duty + _IR_EMITTER_PWM_STEP_SIZE)
                if self.logging:
                    print(f"HT:IR+Emitter Duty: {self.rotation_rate_emitter_duty}")
            if self._pitch < len(NOTE_LIST) - 1:
                self._pitch += 1
            if self._sequencer is not None:
                self._sequencer.play_note_by_index(self._pitch)
            self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            self.button_states.clear()
            if self.settings['serialise'].v:
                self._hut_id = max(0, self._hut_id - 1)
            else:
                self.rotation_rate_emitter_duty = max(0, self.rotation_rate_emitter_duty - _IR_EMITTER_PWM_STEP_SIZE)
                if self.logging:
                    print(f"HT:IR-Emitter Duty: {self.rotation_rate_emitter_duty}")
            if self._pitch > 0:
                self._pitch -= 1
            if self._sequencer is not None:
                self._sequencer.play_note_by_index(self._pitch)
            self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
            self.button_states.clear()
            self._rotation_rate_motor_power = min(_MAX_POWER, self._rotation_rate_motor_power + 1000)
            if self.logging:
                print(f"HT:Motor+Power: {self._rotation_rate_motor_power}")
            self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["LEFT"]):
            self.button_states.clear()
            self._rotation_rate_motor_power = max(-_MAX_POWER, self._rotation_rate_motor_power - 1000)
            if self.logging:
                print(f"HT:Motor-Power: {self._rotation_rate_motor_power}")
            self.refresh = True




        ### End of Update ###



    def draw(self, ctx):
        """Main draw function called from the main loop. Handles drawing the current state, including any notifications."""
        if self.current_state == STATE_MENU and self.menu is not None:
            # These need to be drawn every frame as they contain animations
            clear_background(ctx)
            self.menu.draw(ctx)
        elif self.refresh or self.notification:
            self.refresh = False
            clear_background(ctx)
            ctx.font_size = label_font_size
            if ctx.text_align != ctx.LEFT:
                # See https://github.com/emfcamp/badge-2024-software/issues/181
                ctx.text_align = ctx.LEFT
            ctx.text_baseline = ctx.BOTTOM

             # Common states for messages and errors, which can be triggered by any functional area manager and are displayed in a consistent way
            if self.current_state == STATE_MESSAGE or self.current_state == STATE_WAITING_FOR_HEXDRIVE:
                if self.message_colours == []:
                    self.message_colours = [(1,0,0)]*len(self.message)
                self.draw_message(ctx, self.message, self.message_colours, label_font_size)
                if self.message_type is None or self.message_type == "warning":
                    button_labels(ctx, confirm_label="OK", cancel_label="Exit")
            elif self.current_state == STATE_SETTINGS:
                self.settings_mgr_draw(ctx)
            elif self.current_state == STATE_SENSOR:
                #self.sensor_test_draw(ctx)
                pass
            elif self.current_state == STATE_MOTOR_TEST:
                self._motor_test_draw(ctx)

        # Notifications are drawn on top of everything else, so that they are visible regardless of the current state.
        # They also contain animations, so need to be drawn every frame when active.
        # As they 'withdraw' they reveal whatever is underneath them so this must be redrawn every frame while they are active to avoid leaving visual glitches on the screen.
        if self.notification:
            self.notification.draw(ctx)



    @staticmethod
    def draw_message(ctx, message, colours, size=label_font_size):
        """Utility function to draw a multi-line message on the screen, with optional colour for each line.
           The message is centred on the screen, and the y-position of each line is adjusted based on the total number of lines to ensure it is visually balanced."""
        ctx.font_size = size
        num_lines = len(message)
        for i_num, instr in enumerate(message):
            text_line = str(instr)
            width = ctx.text_width(text_line)
            try:
                colour = colours[i_num]
            except IndexError:
                colour = None
            if colour is None:
                colour = (1,1,1)
            # Font is not central in the height allocated to it due to space for descenders etc...
            # this is most obvious when there is only one line of text
            # # position fine tuned to fit around button labels when showing 5 lines of text
            y_position = int(0.35 * ctx.font_size) if num_lines == 1 else int((i_num-((num_lines-2)/2)) * ctx.font_size - 2)
            ctx.rgb(*colour).move_to(-width//2, y_position).text(text_line)


    def _motor_test_draw(self, ctx):
        if self.config is None:
            return
        if self._scan_mode:
            self._draw_auto_scan(ctx)
            return
        #print("DRAWING")
        if self.settings['serialise'].v:
            lines = [f"ID:{self._hut_id}"]  # show the current HUT ID for data logging purposes
        else:
            # Manual mode: show the current emitter duty cycle as a percentage in the label, and show the current photodiode reading and rate counter value in the display data
            lines = [f"IR:{int(self.rotation_rate_emitter_duty * 100 // 255)}%"]
        colours = [(1, 1, 0)]
        # Show power
        lines += [f"Pwr:{self._rotation_rate_motor_power}"]
        colours += [(0, 1, 1)]
        for index, rpm in enumerate(self._rotation_rate_rpms):
            if rpm is not None:
                lines += [f"{index}: {rpm}rpm"]
                colours += [(1, 0, 1)]
        if self._ina226_reading:
            lines += [f"I:{self._ina226_reading.get('mA', 0)}mA V:{format_voltage_mv(self._ina226_reading.get('mV', 0))}"]
            colours += [(0.3, 0.8, 1.0)]
        else:
            lines += [""]
            colours += [(0.3, 0.8, 1.0)]
        self.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx,
                      up_label="ID+" if self.settings['serialise'].v else "IR+",
                      down_label="ID-" if self.settings['serialise'].v else "IR-",
                      cancel_label="Back", left_label="Pwr-", right_label="Pwr+", confirm_label="Scan")


    def _draw_auto_scan(self, ctx):
        """Draw a chart of power vs RPM from the auto scan results."""
        # Chart area within the 240x240 circular display (origin at centre)
        chart_left = -90
        chart_right = 90
        chart_top = -65
        chart_bottom = 35
        chart_w = chart_right - chart_left
        chart_h = chart_bottom - chart_top

        # Background
        ctx.rgb(0.05, 0.05, 0.05).rectangle(chart_left - 5, chart_top - 5, chart_w + 10, chart_h + 10).fill()

        # Axes
        ctx.rgb(0.4, 0.4, 0.4)
        ctx.move_to(chart_left, chart_bottom).line_to(chart_right, chart_bottom).stroke()  # X axis
        ctx.move_to(chart_left, chart_bottom).line_to(chart_left, chart_top).stroke()      # Y axis

        n = len(self._capture_data)
        max_rpm = self._max_rpm if self._max_rpm > 0 else 1
        max_current_ma = self._max_current_ma if self._max_current_ma > 0 else 1

        if n > 1:
            # Plot data points as bars. Offset between those from increasign power vs decreasing power to make them distinguishable.
            # Auto-scan results contain a list/tuple of per-counter RPMs.
            bar_w = max(1, chart_w // (2 * _AUTO_SCAN_STEPS))
            for i in range(n):
                power, rpms, current_ma, decreasing = self._capture_data[i]
                # allow for the minimum power level being above zero, and for negative power levels, by using the absolute value of power and offsetting the X position accordingly
                x = chart_left + ((abs(power) - _AUTO_SCAN_MIN_POWER) * chart_w) // (_MAX_POWER - _AUTO_SCAN_MIN_POWER) + (bar_w if decreasing else 0)
                for index, rpm in enumerate(rpms):
                    h = (rpm * chart_h) // max_rpm
                    if h > 0:
                        # colour by index to differentiate multiple counters if present
                        if power < 0:
                            index = index + len(self._rotation_rate_counters)  # offset index for negative power to differentiate on the graph
                        if decreasing:
                            index = index + len(self._rotation_rate_counters) * 2  # offset index for decreasing power to differentiate on the graph
                        ctx.rgb(*self._colour_for_index(index)).rectangle(x, chart_bottom - h - 1, bar_w, 2).fill()
                if current_ma is not None:
                    current_h = (abs(current_ma) * chart_h) // max_current_ma
                    marker_y = chart_bottom - current_h
                    ctx.rgb(1.0,0.2,0.2).rectangle(x, marker_y - 1, bar_w, 2).fill()

        # Title and max RPM label
        ctx.font_size = label_font_size
        if self._scan_done:
            ctx.move_to(-30, chart_top - 25).text("Motors")

            ctx.font_size = label_font_size - 8
            ctx.rgb(0.0, 1.0, 1.0).move_to(chart_left, chart_bottom + 5 + ctx.font_size).text(f"{(100 * (_AUTO_SCAN_MIN_POWER + (_MAX_POWER//200)))//_MAX_POWER}%")
            width = ctx.text_width("Power")
            ctx.move_to(-width//2, chart_bottom + 5 + ctx.font_size).text("Power")
            width = ctx.text_width("100%")
            ctx.move_to(chart_right - width, chart_bottom + 5 + ctx.font_size).text("100%")
            # provide a legend for the colours on the graph for the rpms only
            for index in range(len(self._rotation_rate_counters)):
                ctx.rgb(*self._colour_for_index(index)).move_to(chart_left+20, chart_bottom + 5 + ((index + 2) * (ctx.font_size))).text(f"Motor {index+1} RPM")
                # Plot best fit line
                fit = self._motor_calibration_fit[index] if index < len(self._motor_calibration_fit) else None
                if fit is None:
                    continue
                slope, intercept = fit
                # get min and max power values from the scan range
                left_power = self._capture_data[0][0]
                right_power = self._capture_data[n-1][0]
                # is intercept going to be with X or Y axis as only positive quadrant shown
                if intercept < 0:
                    x1 = chart_left - ((intercept * max_rpm) // slope)
                    y1 = chart_bottom
                else:
                    x1 = chart_left
                    y1 = chart_bottom - ((slope * left_power + intercept) * chart_h) // max_rpm
                # is line going to leave chart along the top or right edge?
                if slope * right_power + intercept > max_rpm:
                    x2 = chart_left + ((max_rpm - intercept) * right_power) // slope
                    y2 = chart_top
                else:
                    x2 = chart_right
                    y2 = chart_bottom - ((slope * right_power + intercept) * chart_h) // max_rpm
                    print(f"ST:Motor {index+1} calibration line: slope={slope}, intercept={intercept}, x1={x1}, y1={y1}, x2={x2}, y2={y2}")
                ctx.rgb(*self._colour_for_index(index)).move_to(x1, y1).line_to(x2, y2).stroke()

        else:
            # Calculate progress taking into account whether we are in the first or second pass of the scan, and whether we are increasing or decreasing power.
            previous_steps = _AUTO_SCAN_STEPS if self._scan_decrease_power else 0
            previous_steps += ((2 * _AUTO_SCAN_STEPS) if self._scan_direction == -1 else 0)
            progress = (previous_steps + self._scan_step * 100) // (_AUTO_SCAN_STEPS * 4)  # 0-25% for first pass, 25-50% for second pass, 50-75% for third pass, 75-100% for fourth pass
            ctx.rgb(1.0,1.0,1.0).move_to(-50, chart_top - 25).text(f"Scan {progress}%")

            # Instantaneous current label (updated live during the scan)
            ctx.font_size = label_font_size - 8
            for index, rpm in enumerate(self._rotation_rate_rpms):
                colour_index = index + len(self._rotation_rate_counters) if self._rotation_rate_motor_power < 0 else index  # offset index for negative power to differentiate on the graph
                if self._scan_decrease_power:
                    colour_index = colour_index + len(self._rotation_rate_counters) * 2  # offset index for decreasing power to differentiate on the graph
                ctx.rgb(*self._colour_for_index(colour_index)).move_to(chart_left+20, chart_bottom + 5 + ((index + 2) * (ctx.font_size))).text(f"Mtr{index+1}: {rpm}rpm")
            ctx.rgb(1.0, 0.0, 1.0).move_to(chart_left+20, chart_bottom + 5 + ctx.font_size).text(f"PWM:{(100*abs(self._rotation_rate_motor_power)+(_MAX_POWER//2))//_MAX_POWER}%")
            ctx.rgb(1.0, 0.2, 0.2).move_to(25, chart_bottom + 5 + ctx.font_size).text(f"{self._last_current_ma}mA")

        # Y axis Maximum RPM and Current labels
        ctx.font_size = label_font_size - 8
        ctx.rgb(1.0, 1.0, 0.2).move_to(-15, chart_top - 5).text("Max")
        ctx.rgb(0.2, 1.0, 1.0).move_to(chart_left+10, chart_top - 5).text(f"rpm:{self._max_rpm}")
        ctx.rgb(1.0, 0.2, 0.2).move_to(25, chart_top - 5).text(f"mA:{self._max_current_ma}")

        button_labels(ctx, confirm_label="OK" if self._scan_done else "Quit")


    def _colour_for_index(self, index: int) -> tuple[float, float, float]:
        # lookup from table of colours, green, orange, blue, yellow
        return {
            0: (0.0, 1.0, 0.5),
            1: (1.0, 0.5, 0.0),
            2: (0.0, 0.5, 1.0),
            3: (0.5, 1.0, 0.0),
            4: (0.0, 0.5, 0.3),
            5: (0.5, 0.3, 0.0),
            6: (0.0, 0.3, 0.5),
            7: (0.3, 0.5, 0.0)
        }.get(index, (1.0, 1.0, 1.0))  # default to white if index out of range






    def return_to_menu(self, menu_name: str | None = None):
        """Utility function to return to the main menu from any state. This is used when the user cancels out of a submenu or after acknowledging a warning message."""
        if self.logging:
            print("HT:Returning to menu")
        if menu_name is not None:
            self.set_menu(menu_name)
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        self.current_state = STATE_MENU
        self.refresh = True


    def show_message(self, msg_content, msg_colours, msg_type = None, return_state: int | None = None):
        """Utility function to set the current state to the message display, and populate the message content and colours.
           The message_type can be used to indicate whether this is an 'error' (red) or 'warning' (green) message, which can
           affect both the display and the behaviour when the user acknowledges the message."""
        if self.logging:
            print(f"HT:Showing message: '{msg_content}' with type {msg_type}")
        self.message = msg_content
        self.message_colours = msg_colours
        self.message_type = msg_type
        self.message_return_state = return_state
        self.current_state = STATE_MESSAGE
        self.refresh = True



### MENU FUNCTIONALITY ###


    def set_menu(self, menu_name: str | None = "main"):  #: Literal["main"]): does it work without the type hint?
        """Set the current menu to the specified menu name, and construct the menu if necessary.
           If menu_name is None, it will clear the current menu and return to the previous state
           (e.g. from a submenu back to the main menu)."""
        if self.logging:
            print(f"HT:Set Menu {menu_name}")
        if self.menu is not None:
            try:
                self.menu._cleanup()        # pylint: disable=protected-access
            except Exception:               # pylint: disable=broad-except
                # See badge-2024-software PR#168
                # in case badge s/w changes and this is done within the menu s/w
                # and then access to this function is removed
                pass
        self.current_menu = menu_name
        if menu_name == "main":
            # construct the main menu based on template
            menu_items = MAIN_MENU_ITEMS.copy()
            self.menu = Menu(
                    self,
                    menu_items,
                    select_handler=self._main_menu_select_handler,
                    back_handler=self._menu_back_handler,
                )
        elif menu_name == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]: # "Settings"
            # construct the settings menu
            _settings_menu_items = ["SAVE ALL", "DEFAULT ALL"]
            for _, setting in enumerate(self.settings):
                _settings_menu_items.append(f"{setting}")
            self.menu = Menu(
                self,
                _settings_menu_items,
                select_handler=self._settings_menu_select_handler,
                back_handler=self._menu_back_handler,
                )


    # this appears to be able to be called at any time
    def _main_menu_select_handler(self, item: str, idx: int):
        if self.logging:
            print(f"HT:Main Menu {item} at index {idx}")
        if item == MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_TEST]:   # Motor Test
            self.button_states.clear()
            if self._motor_test_start():
                if self._sequencer is not None:
                    self._sequencer.play(SEQ_STARTUP)
                self.current_state = STATE_MOTOR_TEST
            else:
                self.show_message(["Please Insert","HexDrive"], [(1,0.5,0)]*2)
                self.current_state = STATE_WAITING_FOR_HEXDRIVE
            self.set_menu(None)
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SENSOR_TEST]: # Sensor Test
            self.button_states.clear()
            self.set_menu(None)
            self.show_message(["Sensor test","not implemented","yet"], [(1,0.5,0)]*3, msg_type="warning")
            #if self._sensor_test_start():
            #    self.current_state = STATE_SENSOR
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]:   # Settings
            self.set_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_ABOUT]:      # About
            self.button_states.clear()
            self.set_menu(None)
            self.show_message(["HexTest", f"V{self.VERSION}", "By RobotMad"], [(0.2,1,0.2), (1,1,0), (1,1,1)])
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_EXIT]:       # Exit
            self._exit_app()


    def _settings_menu_select_handler(self, item: str, idx: int):
        if self.logging:
            print(f"HT:Setting {item} @ {idx}")
        if idx == 0: #Save
            if self.logging:
                print("HT:Settings Save All")
            platform_settings.save()
            self.notification = Notification("  Settings  Saved")
            self.set_menu()
        elif idx == 1: #Default
            if self.logging:
                print("HT:Settings Default All")
            for s in self.settings:
                self.settings[s].v = self.settings[s].d
                self.settings[s].persist()
            self.notification = Notification("  Settings Defaulted")
            self.set_menu()
        elif self.settings_mgr_start(item):
            self.current_state = STATE_SETTINGS


    def _menu_back_handler(self):
        if self.current_menu == "main":
            self.minimise()
        # for submenus, just return to the main menu
        self.set_menu()




# --------------------------------------------------
# Private methods for internal use only.
# --------------------------------------------------

    @staticmethod
    def _parse_version(version):
        """ Parse a version string, e.g. that of BadgeOS, into a list of components for comparison. Handles versions in the format v1.9.0-beta.1+build.123
            The version is split into components based on the delimiters '.' '-' and '+'."""
        #pre_components = ["final"]
        #build_components = ["0", "000000z"]
        #build = ""
        components = []
        if "+" in version:
            version, build = version.split("+", 1)          # pylint: disable=unused-variable
        #    build_components = build.split(".")
        if "-" in version:
            version, pre_release = version.split("-", 1)    # pylint: disable=unused-variable
        #    if pre_release.startswith("rc"):
        #        # Re-write rc as c, to support a1, b1, rc1, final ordering
        #        pre_release = pre_release[1:]
        #    pre_components = pre_release.split(".")
        version = version.strip("v").split(".")
        components = [int(item) if item.isdigit() else item for item in version]
        #components.append([int(item) if item.isdigit() else item for item in pre_components])
        #components.append([int(item) if item.isdigit() else item for item in build_components])
        return components



    # ------------------------------------------------------------------

    def  settings_mgr_start(self, item: str) -> bool:
        """Enter Settings editing mode from the main menu."""
        self.set_menu(None)
        self.button_states.clear()
        self.refresh = True
        self.auto_repeat_clear()
        if self._logging:
            print("HT:Entered Settings editing mode")
        self.edit_setting = item
        self.edit_setting_value = self.settings[item].v
        return True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def _settings_mgr_update(self, delta):
        """Handle Settings editing UI.  Returns True if this module handled the state."""

        if self.button_states.get(BUTTON_TYPES["UP"]):
            if self.auto_repeat_check(delta, False):
                self.edit_setting_value = self.settings[self.edit_setting].inc(self.edit_setting_value, self.auto_repeat_level)
                if self._logging:
                    print(f"HT:Setting: {self.edit_setting} (+) Value: {self.edit_setting_value}")
                self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            if self.auto_repeat_check(delta, False):
                self.edit_setting_value = self.settings[self.edit_setting].dec(self.edit_setting_value, self.auto_repeat_level)
                if self._logging:
                    print(f"HT:Setting: {self.edit_setting} (-) Value: {self.edit_setting_value}")
                self.refresh = True
        else:
            self.auto_repeat_clear()
            if self.button_states.get(BUTTON_TYPES["RIGHT"]) or self.button_states.get(BUTTON_TYPES["LEFT"]):
                self.button_states.clear()
                self.edit_setting_value = self.settings[self.edit_setting].d
                if self._logging:
                    print(f"HT:Setting: {self.edit_setting} Default: {self.edit_setting_value}")
                self.refresh = True
                self.notification = Notification("Default")
            elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                if self._logging:
                    print(f"HT:Setting: {self.edit_setting} Cancelled")
                self.return_to_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                if self._logging:
                    print(f"HT:Setting: {self.edit_setting} = {self.edit_setting_value}")
                self.settings[self.edit_setting].v = self.edit_setting_value
                self.settings[self.edit_setting].persist()
                self.notification = Notification(f"  Setting:   {self.edit_setting}={self.edit_setting_value}")
                self.return_to_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
        return True


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def settings_mgr_draw(self, ctx):
        """Render Settings editing UI.  Returns True if handled."""
        disp_val = self.settings[self.edit_setting].label(self.edit_setting_value)
        self.draw_message(ctx, ["Edit Setting", f"{self.edit_setting}:", f"{disp_val}"], [(1, 1, 0), (0, 0, 1), (0, 1, 0)], label_font_size)
        button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
        return True


    # ------------------------------------------------------------------


    # ------------------------------------------------------------------
    # Audio
    # ------------------------------------------------------------------
    #def play_tone(self, freq: int = 440, amplitude: int = 32000, duration: int = 1000):
    #    # populate a single cycle of a sine wave at the given frequency
    #    # and play it for the given duration in milliseconds
    #    # use I2S to output the audio to the DAC, which is connected to the speaker
    #    # The audio output is mono, so we only need to generate a single channel of audio data.
    #    # I2S has already been configured in the main loop, so we just need to write the audio data to the I2S buffer.
    #    if self._i2s is None:
    #         return
    #     # check if a tone is already playing, and if so, stop it before starting a new one
    #    if self._i2s_cycles > 0:
    #        self._i2s_cycles = 1
    #        print("HT:Stopping previous tone")
    #        while self._i2s_cycles > 0:
    #            time.sleep_ms(10)
    #        self._i2s.irq(None)  # disable callback
    #    try:
    #        from math import sin, pi
    #        _CH = 1                      # mono
    #        n: int = _I2S_RATE // freq   # number of samples per cycle
    #        c: int = 0                   # number of cycles per buffer
    #        buffer_len: int = 0

            # Ensure that n is large enough to avoid excessive interrupt load. If n is too small, the I2S buffer will be emptied too quickly and the callback will be called too frequently, which can cause audio glitches.
    #        while buffer_len < _MIN_SAMPLES_PER_INTERRUPT:
    #            buffer_len += n
    #            c += 1

    #        self._i2s_buffer = bytearray(n * 2 * _CH)  # 16-bit sample

    #        for i in range(buffer_len):
    #            l = int(sin(2 * pi * i / n) * amplitude)
    #            self._i2s_buffer[i*2*_CH:i*2*_CH+2] = l.to_bytes(2, 'little', True)
    #            if _CH == 2:  # Stereo
    #                r = int(sin(2 * pi * i / n) * amplitude)
    #                self._i2s_buffer[i*4+2:i*4+4] = r.to_bytes(2, 'little', True)
   #         print(f"HT: Playing tone @ {freq}Hz for {duration}ms")
   #         # How many buffers to play for the given duration
   #         self._i2s_cycles = (duration * freq) // (c * 1000)
   #         if self._i2s_cycles == 0:
   #             self._i2s_cycles = 1                  # ensure at least one buffer worth is played
   #         self._i2s.irq(self._i2s_callback)         # i2s_callback is called when buf is emptied

    #    except Exception as e:
    #        print(f"HT: Error preparing tone: {e}")
    #        return

        #try:
        #    _ = self._i2s.write(bytes(self._i2s_buffer))  # returns immediately
        #except Exception as e:
        #    print(f"HT: Error Writing I2S: {e}")



    #def _i2s_callback(self, event):
    #    """Callback function for I2S events. This is called when the I2S buffer is emptied and needs more data."""
    #    try:
    #        if self._i2s_cycles > 0:
    #            self._i2s_cycles -= 1
    #            if self._i2s_cycles == 0:
    #                print("HT: I2S Tone playback complete")
    #                self._i2s.irq(None)  # disable callback
   #                 return
   #         self._i2s.write(bytes(self._i2s_buffer))
   #     except Exception as e:
   #         print(f"HT: Error in I2S callback: {e}")
   #         self._i2s.irq(None)  # disable callback


class HexpansionType:
    """Descriptor for known hexpansion types, used for detection and EEPROM programming.

    Parameters
    ----------
        pid: the PID value to identify the hexpansion type from its EEPROM header
        name: human-friendly name of the hexpansion type (e.g. "HexDrive")
        vid: the VID value to identify the hexpansion type from its EEPROM header (default 0xCAFE)
        motors, servos, sensors: the capabilities of this hexpansion type, used to configure the app when detected
        sub_type: a human-friendly string describing the specific variant of this hexpansion type
        app_mpy_name: the filename of the .mpy to copy to the hexpansion EEPROM for this type (if any)
        app_mpy_version: the version string to report for the .mpy copied to the hexpansion EEPROM for this type (if any)
        app_name: the name of the App class to look for when checking if a detected hexpansion's app is running (if any)
    """
    def __init__(self, pid: int, name: str, vid: int =0xCAFE, motors: int =0, servos: int =0, sensors: int =0, sub_type: str | None =None):
        self.vid: int = vid
        self.pid: int = pid
        self._name: str = name
        self._sub_type: str | None = sub_type
        self._motors: int = motors
        self._servos: int = servos
        self._sensors: int = sensors

    @property
    def name(self):
        return self._name

    @property
    def sub_type(self):
        return self._sub_type

    @property
    def motors(self):
        return self._motors

    @property
    def servos(self):
        return self._servos

    @property
    def sensors(self):
        return self._sensors




class MySetting:
    """Class to represent a single setting, including its current value, default value, min/max values, and optional labels for display. """
    def __init__(self, container, default, minimum, maximum, labels=None):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum
        self._labels = labels

    def __str__(self):
        return str(self.v)

    def _index(self):
        for k, v in self._container.items():
            if v == self:
                return k
        return None

    def label(self, index: int | None = None):
        """ Return the label for the given index, or the current value if no index is provided. """
        if index is not None:
            if self._labels is not None and index < len(self._labels):
                return self._labels[int(index)]
            return str(index)
        if self._labels is not None and self.v is not None and self.v < len(self._labels):
            return self._labels[int(self.v)]
        return str(self.v)

    @staticmethod
    def _quantize_tenths(value: float) -> float:
        """Round to 0.1 steps deterministically to avoid float drift artifacts."""
        scaled = int((value * 10) + (0.5 if value >= 0 else -0.5))
        return scaled / 10.0

    def inc(self, v, l=0):
        """ Increment the setting value.  If l > 0, increment by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.)"""
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l == 0:
                v += 1
            else:
                d = 10 ** l
                v = ((v // d) + 1) * d
            if v > self._max:
                if self._labels is not None:
                    # settings that are purely label-based wrap around
                    v = 0
                else:
                    v = self._max
        elif isinstance(self.v, float):
            v = self._quantize_tenths(v) + 0.1
            if v > self._max:
                v = self._max
            v = self._quantize_tenths(v)
        elif self._container['logging'].v:
            print(f"H:inc type: {type(self.v)}")
        return v

    def dec(self, v, l=0):
        """Decrement the setting value.  If l > 0, decrement by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.)"""
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l == 0:
                v -= 1
            else:
                d = 10 ** l
                v = (((v + (9 * (10 ** (l - 1)))) // d) - 1) * d
            if v < self._min:
                if self._labels is not None:
                    # settings that are purely label-based wrap around
                    v = len(self._labels) - 1
                else:
                    v = self._min
        elif isinstance(self.v, float):
            v = self._quantize_tenths(v) - 0.1
            if v < self._min:
                v = self._min
            v = self._quantize_tenths(v)
        elif self._container['logging'].v:
            print(f"H: dec type: {type(self.v)}")
        return v

    def persist(self):
        """Persist the setting value to platform storage.  If the value is equal to the default, the setting will be removed from storage to save space."""
        index = self._index()
        if index is None:
            return
        key = f"{_PRE}.{index}"
        try:
            platform_settings.set(key, self.v if self.v != self.d else None)
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Failed to persist setting {key}: {e}")

    # ------------------------------------------------------------------

#------------------------------------------------------------------
# ESP32S3 PCNT (Pulse Counter) hardware register definitions and bit masks
# Supports all 4 PCNT units (0-3) on the ESP32-S3.
#-------------------------------------------------------------------

_SYSTEM_BASE      = const(0x600C0000)
_GPIO_BASE        = const(0x60004000)
_PCNT_BASE        = const(0x60017000)

_PCNT_NUM_UNITS   = const(4)   # ESP32-S3 has 4 PCNT units

_PCNT_CLK_BIT     = const(1 << 10)  # SYSTEM_PCNT_CLK_EN / SYSTEM_PCNT_RST (bit 10)

# System/Clock registers
_CLK_EN0_REG      = const(_SYSTEM_BASE + 0x0018)
_RST_EN0_REG      = const(_SYSTEM_BASE + 0x0020)

# GPIO Matrix Base
_GPIO_FUNC_IN_SEL_CFG_BASE = const(_GPIO_BASE + 0x0154)
_SIG_IN_SEL_BIT   = const(1 << 6) # Enable routing via GPIO Matrix

# PCNT register offsets (per-unit, relative to _PCNT_BASE)
#   CONF0: _PCNT_BASE + unit * 0x0C
#   CONF1: _PCNT_BASE + unit * 0x0C + 0x04
#   CONF2: _PCNT_BASE + unit * 0x0C + 0x08
#   CNT:   _PCNT_BASE + 0x30 + unit * 4
#   STATUS:_PCNT_BASE + 0x50 + unit * 4
_PCNT_CTRL_REG    = const(_PCNT_BASE + 0x0060)

# _PCNT_CTRL_REG bits — per-unit reset and pause at (unit * 2) and (unit * 2 + 1)
_PCNT_CTRL_CLK_EN = const(1 << 16)  # Register clock gate — must be 1 for register access

# CONF0 bit layout (same layout for all units)
_CONF0_FILTER_THRES_M  = const(0x3FF)   # bits [9:0]
_CONF0_FILTER_EN       = const(1 << 10)
_CONF0_CH0_POS_MODE_S  = const(18)      # bits [19:18]

# GPIO signal index base for PCNT: Unit N, CH0 pulse = 33 + N*4, CH0 ctrl = 35 + N*4
_PCNT_SIG_BASE    = const(33)

# APB clock frequency for filter calculation (Hz)
_APB_CLK_HZ       = const(80_000_000)


# Reverse lookup: GPIO number -> (port, pin_index) for diagnostics
_GPIO_TO_HS = {}
for _port, _gpios in _HS_PIN_TO_GPIO.items():
    for _idx, _gpio in enumerate(_gpios):
        _GPIO_TO_HS[_gpio] = (_port, _idx)


# ------------------------------------------------------------------

class Counter:
    """Wrapper around ESP32-S3 PCNT hardware for counting rising edges.

    Parameters
    ----------
    src : int
        The ESP32-S3 GPIO number to count pulses on.
        Use ``_HS_PIN_TO_GPIO[port][index]`` to convert from a badge HS pin.
    id : int | None
        PCNT unit to use (0-3).  If ``None``, the first available (unused) unit
        is auto-selected.  If the requested unit is already in use, ``__init__``
        sets ``self.unit = None`` to signal failure.
    filter_ns : int
        Minimum pulse width in nanoseconds.  Pulses shorter than this are
        rejected by the hardware glitch filter.  Set to 0 to disable filtering.
    logging : bool
        Print diagnostic messages to the console.

    CURRENTLY ONLY COUNTS UP ON RISING EDGES
    """

    def __init__(self, unit: int | None, src: int, filter_ns: int = 0, logging: bool = False):
        self.logging = logging
        self._configured = False

        if unit is not None:
            if unit < 0 or unit >= _PCNT_NUM_UNITS:
                if self.logging:
                    print(f"PCNT: unit {unit} out of range (0-{_PCNT_NUM_UNITS - 1})")
                self.unit = None
                return
            if self._unit_in_use(unit):
                self.unit = None
                return
            self.unit = unit
        else:
            # Auto-select first available unit
            self.unit = None
            for u in range(_PCNT_NUM_UNITS):
                if not self._unit_in_use(u):
                    self.unit = u
                    break
            if self.unit is None:
                if self.logging:
                    print("PCNT: all units in use, no free unit available")
                return

        if not self.init(src, filter_ns):
            if self.logging:
                print(f"PCNT: failed to configure unit {self.unit}")
            self.unit = None


    def _unit_in_use(self, unit: int) -> bool:
        """Check whether a PCNT unit appears to already be in use.

        A unit is considered in use if:
        - The peripheral clock is enabled AND
        - The register clock gate is enabled AND
        - The unit is NOT held in reset AND
        - CONF0 is non-zero (has been configured)
        """
        # Check peripheral clock
        clk_on = (mem32[_CLK_EN0_REG] & _PCNT_CLK_BIT) != 0
        if not clk_on:
            if self.logging:
                print(f"PCNT: unit {unit} - peripheral clock off, unit free")
            return False

        ctrl = mem32[_PCNT_CTRL_REG]

        # Check register clock gate
        if not ctrl & _PCNT_CTRL_CLK_EN:
            if self.logging:
                print(f"PCNT: unit {unit} - register clock gate off, unit free")
            return False

        # Check if held in reset (reset bit = unit * 2)
        rst_bit = 1 << (unit * 2)
        if ctrl & rst_bit:
            if self.logging:
                print(f"PCNT: unit {unit} - held in reset, unit free")
            return False

        # Check CONF0 register
        conf0_addr = _PCNT_BASE + unit * 0x0C
        conf0 = mem32[conf0_addr]
        if conf0 == 0x3C10: # a slightly odd reset state
            if self.logging:
                print(f"PCNT: unit {unit} - CONF0=0x3C10 (unconfigured), unit free")
            return False

        # Unit appears to be actively configured and running
        if self.logging:
            cnt_addr = _PCNT_BASE + 0x30 + unit * 4
            cnt = mem32[cnt_addr] & 0xFFFF
            pulse_sig = _PCNT_SIG_BASE + unit * 4
            gpio_route = mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + pulse_sig * 4]
            routed_gpio = gpio_route & 0x3F
            print(f"PCNT: unit {unit} - IN USE: CONF0=0x{conf0:08X}, count={cnt}, routed to GPIO {routed_gpio}")
        return True


    def __str__(self):
        if self.unit is None:
            return "Counter(not configured)"
        count = self.value()
        return f"Counter(unit={self.unit}, GPIO={self.pin}, count={count})"


    def init(self, src: int, filter_ns: int | None = None) -> bool:
        """Configure a PCNT unit to count rising edges on the GPIO pin specified by src."""
        self.pin = src

        unit = self.unit
        if unit is None:
            return False
        conf0_addr = _PCNT_BASE + unit * 0x0C
        cnt_addr = _PCNT_BASE + 0x30 + unit * 4
        rst_bit = 1 << (unit * 2)
        pulse_sig = _PCNT_SIG_BASE + unit * 4       # PCNT_SIG_CH0_INn
        ctrl_sig = _PCNT_SIG_BASE + unit * 4 + 2    # PCNT_CTRL_CH0_INn

        if self.logging:
            hs = _GPIO_TO_HS.get(self.pin)
            hs_str = f" port {hs[0]} HS pin {hs[1]})" if hs else ""
            print(f"PCNT U{unit}: on GPIO {self.pin}{hs_str}, filter_ns={filter_ns}ns")
            print(f"  CONF0 addr=0x{conf0_addr:08X}, CNT addr=0x{cnt_addr:08X}")
            print(f"  pulse_sig={pulse_sig}, ctrl_sig={ctrl_sig}")

        try:
            # --- 1. ENABLE PERIPHERAL CLOCK ---
            mem32[_CLK_EN0_REG] |= _PCNT_CLK_BIT
            mem32[_RST_EN0_REG] &= ~_PCNT_CLK_BIT

            # --- 2. ENABLE REGISTER CLOCK GATE, HOLD THIS UNIT IN RESET ---
            # Read-modify-write to preserve other units' state
            ctrl = mem32[_PCNT_CTRL_REG]
            ctrl |= _PCNT_CTRL_CLK_EN | rst_bit
            mem32[_PCNT_CTRL_REG] = ctrl

            # --- 3. ROUTE GPIO VIA MATRIX ---
            mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (pulse_sig * 4)] = _SIG_IN_SEL_BIT | self.pin
            # Route constant high (0x38) to control signal
            mem32[_GPIO_FUNC_IN_SEL_CFG_BASE + (ctrl_sig * 4)] = _SIG_IN_SEL_BIT | 0x38

            # --- 4. CONFIGURE COUNTING ---
            # Calculate filter threshold from min pulse width
            if filter_ns is not None and filter_ns > 0:
                filter_val = (_APB_CLK_HZ * filter_ns) // 1_000_000_000
                if filter_val > 1023:
                    filter_val = 1023
                config = (filter_val & _CONF0_FILTER_THRES_M) | _CONF0_FILTER_EN
            else:
                config = 0
            config |= (1 << _CONF0_CH0_POS_MODE_S)  # Inc on rising edge
            mem32[conf0_addr] = config

            # --- 5. RELEASE FROM RESET ---
            ctrl = mem32[_PCNT_CTRL_REG]
            ctrl &= ~rst_bit
            mem32[_PCNT_CTRL_REG] = ctrl

            self._configured = True

        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"PCNT U{unit}: error configuring: {e}")
            return False

        if self.logging:
            print(f"PCNT U{unit}: configured - CONF0=0x{mem32[conf0_addr]:08X}, CTRL=0x{mem32[_PCNT_CTRL_REG]:08X}, CNT={mem32[cnt_addr] & 0xFFFF}")
        return True


    def value(self, value: int | None = None) -> int:
        """Read the current count and optionally reset the counter to zero.
          DOES NOT SUPPORT SETTING THE COUNTER TO AN ARBITRARY VALUE, ONLY RESETTING TO ZERO."""
        if not self._configured:
            return 0

        unit = self.unit
        if unit is None:
            return 0

        rst_bit = 1 << (unit * 2)
        cnt_addr = _PCNT_BASE + 0x30 + unit * 4
        if value is not None and value == 0:
            irq_state = disable_irq()
            count = mem32[cnt_addr] & 0xFFFF
            mem32[_PCNT_CTRL_REG] |= rst_bit
            mem32[_PCNT_CTRL_REG] &= ~rst_bit
            enable_irq(irq_state)
        else:
            count = mem32[cnt_addr] & 0xFFFF
        return count


    def deinit(self):
        """Release the PCNT unit: hold it in reset and clear its CONF0."""
        if not self._configured or self.unit is None:
            return
        unit = self.unit
        conf0_addr = _PCNT_BASE + unit * 0x0C
        rst_bit = 1 << (unit * 2)
        mem32[_PCNT_CTRL_REG] |= rst_bit   # hold in reset
        mem32[conf0_addr] = 0               # clear config so unit appears free
        self._configured = False

        if self.logging:
            print(f"PCNT U{unit}: released")

        # disable the peripheral clock if no units are in use to save power
        if not any(self._unit_in_use(u) for u in range(_PCNT_NUM_UNITS)):
            mem32[_CLK_EN0_REG] &= ~_PCNT_CLK_BIT
            mem32[_RST_EN0_REG] |= _PCNT_CLK_BIT
            if self.logging:
                print("PCNT: all units released, peripheral clock disabled")











class SensorBase:
    """Abstract base class for BadgeBot I2C sensor drivers."""
    # Sub-classes must override these
    I2C_ADDR = 0x00
    NAME = "Unknown"
    READ_INTERVAL_MS = 250
    TYPE = "Generic"

    def __init__(self, i2c_addr: int | None = None, logging: bool = False):
        self._i2c = None
        self._ready = False
        self._i2c_addr = self.I2C_ADDR if i2c_addr is None else i2c_addr
        self._logging = logging

    # ------------------------------------------------------------------
    # Public API (called by SensorManager / app.py)
    # ------------------------------------------------------------------

    def begin(self, i2c) -> bool:
        """Initialise the sensor on the given I2C bus.

        Returns True if the sensor is found and configured successfully.
        Store the i2c object for later use in read().
        """
        self._i2c = i2c
        self._ready = False
        try:
            self._ready = self._init()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} begin error: {e}")
            self._ready = False
        return self._ready

    def read(self, timeout: int | None = None) -> dict:
        """Return the latest measurement as {label: value_string}.

        Returns an empty dict or {'Error': 'msg'} on failure.
        """
        if not self._ready:
            return {"Error": "not ready"}
        try:
            return self._measure(timeout=timeout) if timeout is not None else self._measure()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} read error: {e}")
            return {"Error": str(e)}

    def read_sample_if_ready(self) -> dict | None:
        """Optional non-blocking sample hook for sensors that support it."""
        return None

    def reset(self):
        """Put the sensor into a low-power / safe state."""
        try:
            self._shutdown()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} reset error: {e}")
        self._ready = False

    def shutdown(self):
        """Put the sensor into a low-power state without changing ready state."""
        if self._i2c is None:
            return
        try:
            self._shutdown()
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"S:{self.NAME} shutdown error: {e}")

    @property
    def is_ready(self) -> bool:
        """True if the sensor is initialised and ready for measurements."""
        return self._ready

    @property
    def i2c_addr(self) -> int:
        """Return the I2C address of the sensor."""
        return self._i2c_addr

    # ------------------------------------------------------------------
    # Internal helpers - override in sub-classes
    # ------------------------------------------------------------------

    def _init(self) -> bool:
        """Hardware initialisation. Return True on success."""
        raise NotImplementedError

    def _measure(self, timeout: int = 0) -> dict:
        """Perform measurement. Return dict of {label: value_str}."""
        raise NotImplementedError

    def _shutdown(self):
        """Optional power-down hook.

        Subclasses can implement register writes here when hardware supports
        an explicit shutdown mode. Drivers without dedicated power management
        can leave this as a no-op.
        """
        return

    # ------------------------------------------------------------------
    # Utility helpers available to all drivers
    # ------------------------------------------------------------------

    def _write_reg(self, reg: int, data: bytes):
        if self._i2c is None:
            raise RuntimeError("I2C not initialized")
        self._i2c.writeto_mem(self._i2c_addr, reg, data)

    def _read_reg(self, reg: int, n: int = 1) -> bytes:
        if self._i2c is None:
            raise RuntimeError("I2C not initialized")
        return self._i2c.readfrom_mem(self._i2c_addr, reg, n)

    def _read_u8(self, reg: int) -> int:
        return self._read_reg(reg, 1)[0]

    def _read_u16_le(self, reg: int) -> int:
        d = self._read_reg(reg, 2)
        return d[0] | (d[1] << 8)

    def _read_u16_be(self, reg: int) -> int:
        d = self._read_reg(reg, 2)
        return (d[0] << 8) | d[1]

    def _read_s16_be(self, reg: int) -> int:
        value = self._read_u16_be(reg)
        if value & 0x8000:
            value -= 0x10000
        return value

    def _write_u8(self, reg: int, value: int):
        self._write_reg(reg, bytes([value & 0xFF]))

    def _write_u16_be(self, reg: int, value: int):
        self._write_reg(reg, bytes([(value >> 8) & 0xFF, value & 0xFF]))





# Register map
_REG_CONFIGURATION = const(0x00)      # Configuration register
_REG_SHUNT_VOLTAGE = const(0x01)      # Shunt voltage result (signed)
_REG_BUS_VOLTAGE = const(0x02)        # Bus voltage result (unsigned)
_REG_POWER = const(0x03)              # Power result (unsigned)
_REG_CURRENT = const(0x04)            # Current result (signed)
_REG_CALIBRATION = const(0x05)        # Calibration register
_REG_MASK_ENABLE = const(0x06)        # Alert mask/enable register
_REG_ALERT_LIMIT = const(0x07)        # Alert threshold register
_REG_MANUFACTURER_ID = const(0xFE)    # Manufacturer ID register
_REG_DIE_ID = const(0xFF)             # Die ID register


# Configuration register bits (0x00)
_CFG_RESET_BIT = const(0x8000)        # Software reset bit
_CFG_AVG_SHIFT = const(9)             # Averaging field shift (bits 11:9)
_CFG_VBUSCT_SHIFT = const(6)          # Bus voltage conversion time field shift (bits 8:6)
_CFG_VSHCT_SHIFT = const(3)           # Shunt voltage conversion time field shift (bits 5:3)
_CFG_MODE_SHIFT = const(0)            # Operating mode field shift (bits 2:0)

# AVG field values (bits 11:9)
_CFG_AVG_1 = const(0b000)             # 1 sample average
_CFG_AVG_4 = const(0b001)             # 4 sample average
_CFG_AVG_16 = const(0b010)            # 16 sample average
_CFG_AVG_64 = const(0b011)            # 64 sample average
_CFG_AVG_128 = const(0b100)           # 128 sample average
_CFG_AVG_256 = const(0b101)           # 256 sample average
_CFG_AVG_512 = const(0b110)           # 512 sample average
_CFG_AVG_1024 = const(0b111)          # 1024 sample average

# Conversion time field values for VBUSCT/VSHCT (bits 8:6 and 5:3)
_CFG_CT_140US = const(0b000)          # 140 us conversion time
_CFG_CT_204US = const(0b001)          # 204 us conversion time
_CFG_CT_332US = const(0b010)          # 332 us conversion time
_CFG_CT_588US = const(0b011)          # 588 us conversion time
_CFG_CT_1100US = const(0b100)         # 1.1 ms conversion time
_CFG_CT_2116US = const(0b101)         # 2.116 ms conversion time
_CFG_CT_4156US = const(0b110)         # 4.156 ms conversion time
_CFG_CT_8244US = const(0b111)         # 8.244 ms conversion time

# Operating mode field values (bits 2:0)
_CFG_MODE_POWER_DOWN = const(0b000)   # Power-down mode
_CFG_MODE_SHUNT_TRIG = const(0b001)   # Shunt voltage, triggered
_CFG_MODE_BUS_TRIG = const(0b010)     # Bus voltage, triggered
_CFG_MODE_SHUNT_BUS_TRIG = const(0b011)   # Shunt and bus, triggered
_CFG_MODE_ADC_OFF = const(0b100)      # ADC off (disabled)
_CFG_MODE_SHUNT_CONT = const(0b101)   # Shunt voltage, continuous
_CFG_MODE_BUS_CONT = const(0b110)     # Bus voltage, continuous
_CFG_MODE_SHUNT_BUS_CONT = const(0b111)   # Shunt and bus, continuous


# Mask/Enable register bits (0x06)
_MASK_SOL = const(0x8000)             # Shunt over-voltage alert flag
_MASK_SUL = const(0x4000)             # Shunt under-voltage alert flag
_MASK_BOL = const(0x2000)             # Bus over-voltage alert flag
_MASK_BUL = const(0x1000)             # Bus under-voltage alert flag
_MASK_POL = const(0x0800)             # Power over-limit alert flag
_MASK_CNVR = const(0x0400)            # Conversion ready alert flag
_MASK_AFF = const(0x0010)             # Alert function flag
_MASK_CVRF = const(0x0008)            # Conversion ready flag
_MASK_OVF = const(0x0004)             # Math overflow flag
_MASK_APOL = const(0x0002)            # Alert pin polarity select
_MASK_LEN = const(0x0001)             # Alert latch enable


# Device identification
_MANUFACTURER_ID_TI = const(0x5449)   # Texas Instruments manufacturer ID


# Driver configuration constants (100 mΩ shunt)
_SHUNT_RESISTOR_MILLIOHM = const(100)
_CALIBRATION_VALUE = const(0x0200)    # 512 => 0.1 mA current register LSB with 100 mΩ shunt
_CURRENT_LSB_UA = const(100)          # 0.1 mA current LSB in microamps
_POWER_LSB_UW = const(2500)           # 2.5 mW power LSB in microwatts
_READ_TIMEOUT_MS = const(50)

# Default operating configuration:
#  - shunt conversion: 8.244 ms
#  - bus conversion:   1.1 ms
#  - averaging:        16 sample
_DEFAULT_CONFIGURATION = (
      (_CFG_AVG_16 << _CFG_AVG_SHIFT)
    | (_CFG_CT_1100US << _CFG_VBUSCT_SHIFT)
    | (_CFG_CT_8244US << _CFG_VSHCT_SHIFT)
    | (_CFG_MODE_SHUNT_BUS_CONT << _CFG_MODE_SHIFT)
)


class INA226(SensorBase):
    """INA226 sensor driver with integer fixed-point outputs."""

    I2C_ADDR = 0x40
    I2C_ADDRS = tuple(range(0x40, 0x43))    # only allow the addresses we actually expect (full range is to 0x50)
    NAME = "INA226"
    READ_INTERVAL_MS = 150
    TYPE = "Power"

    def _measure_from_registers(self) -> dict[str, int]:
        bus_raw = self._read_u16_be(_REG_BUS_VOLTAGE)
        current_raw = self._read_s16_be(_REG_CURRENT)

        # Bus LSB = 1.25 mV
        bus_mv = (bus_raw * 125) // 100
        # Current LSB from calibration = 100 uA (0.1 mA)
        current_ma = (current_raw * _CURRENT_LSB_UA) // 1000

        #print(f"S:{self.NAME} {bus_mv}mV, {current_ma}mA")
        return {
            "mV": bus_mv,
            "mA": current_ma,
        }

    def read_sample_if_ready(self) -> dict[str, int] | None:
        """Return one sample in integer units when a new conversion is ready.

        This helper is intended for high-rate internal consumers (for example
        background averaging in motor test mode). The public SensorBase `read()`
        API still returns string values for UI rendering consistency.
        """
        if not self._ready:
            return None
        status = self._read_u16_be(_REG_MASK_ENABLE)
        if (status & _MASK_CVRF) == 0:
            #print(f"S:{self.NAME} sample not ready (status=0x{status:04X})")
            return None
        if (status & _MASK_OVF) != 0:
            print(f"S:{self.NAME} math overflow (status=0x{status:04X})")
            return None
        return self._measure_from_registers()


    def _init(self) -> bool:
        manufacturer = self._read_u16_be(_REG_MANUFACTURER_ID)
        if manufacturer != _MANUFACTURER_ID_TI:
            return False

        self._write_u16_be(_REG_CONFIGURATION, _DEFAULT_CONFIGURATION)
        self._write_u16_be(_REG_CALIBRATION, _CALIBRATION_VALUE)
        self._write_u16_be(_REG_MASK_ENABLE, _MASK_CNVR | _MASK_LEN)  # Enable conversion ready alert with latching
        return True


    def _measure(self, timeout: int=_READ_TIMEOUT_MS) -> dict:
        deadline = time.ticks_add(time.ticks_ms(), timeout)
        while True:
            sample = self.read_sample_if_ready()
            if sample is not None:
                return {
                    "mV": str(sample["mV"]),
                    "mA": str(sample["mA"]),
                }
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return {"Error": "timeout"}
            time.sleep_ms(1)

    def _shutdown(self) -> None:
        self._write_u16_be(_REG_CONFIGURATION, _CFG_MODE_POWER_DOWN)






_LED_PIN = const(2)        # LED to illumiinate area under colour sensor to measure reflected light from surface below.
_COLOUR_INT_PIN = const(1)  # Not currently used, but we can set it up as an input for future interrupt-based drivers
_DIST_INT_PIN = const(3)  # Not currently used, but we can set it up as an input for future interrupt-based drivers

ALL_SENSOR_CLASSES = [INA226]

class SensorManager:
    """Manages detection, initialisation, and reading of sensors on a hexpansion I2C port."""
    def __init__(self, logging: bool = False):
        self._logging: bool = logging
        self._i2c = None
        self._port: int | None = None
        self._sensors: list[SensorBase] = []      # list of initialised SensorBase instances
        self._index: int = 0         # currently selected sensor
        self._last_data = {}
        self._read_interval_ms = 10
        self._type = "Generic"
        if self._logging:
            print("HT:SensorManager initialised")


    # ------------------------------------------------------------------

    @property
    def read_interval(self) -> int:
        """Return the recommended read interval in milliseconds for the currently selected sensor."""
        return self._read_interval_ms

    @property
    def type(self) -> str:
        """Return the type of the currently selected sensor, or "Generic" if no sensors or only unknown sensors are found."""
        return self._type


    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self, port: int) -> bool:
        """Open hexpansion I2C port (1–6), scan, and initialise any found sensors.
        Returns True if at least one sensor was found."""
        self.close()
        self._port = port

        try:
            self._i2c = I2C(port)
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"HT:Cannot open I2C port {port}: {e}")
            return False

        try:
            found_addrs = set(self._i2c.scan())
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"HT:I2C scan failed on port {port}: {e}")
            return False

        if self._logging:
            print(f"HT:Port {port} scan: {[hex(a) for a in found_addrs]}")

        used_addrs = set()
        for cls in ALL_SENSOR_CLASSES:
            addresses = getattr(cls, "I2C_ADDRS", (getattr(cls, "I2C_ADDR", 0),))
            for address in addresses:
                if address not in found_addrs:
                    continue
                if address in used_addrs:
                    continue
                try:
                    sensor = cls(i2c_addr=address, logging=self._logging)
                except TypeError:
                    sensor = cls()
                if sensor.begin(self._i2c):
                    self._sensors.append(sensor)
                    used_addrs.add(address)
                    if self._logging:
                        print(f"HT:  + {cls.NAME} @ 0x{sensor.i2c_addr:02X} {cls.TYPE}")
                elif self._logging:
                    print(f"HT:  - {cls.NAME} @ 0x{address:02X} begin() failed")

        self._index = 0
        self._last_data = {}

        # Set read interval from the first found sensor, or default to 250ms  TODO: support multiple sensors with different intervals?
        if self._sensors:
            self._read_interval_ms = getattr(self._sensors[0], 'READ_INTERVAL_MS', 250)
            self._type = getattr(self._sensors[0], 'TYPE', 'Generic')
        else:
            self._read_interval_ms = 250
            self._type = "Generic"

        # Enable LED only when at least one Colour sensor is present
        # (avoids pin conflicts with non-colour hexpansions such as the motor-test board)
        if len(self._sensors) > 0 and any(getattr(s, 'TYPE', '') == 'Colour' for s in self._sensors):
            config = HexpansionConfig(port)
            if self._logging:
                print(f"HT:LED On port {port} pin {config.ls_pin[_LED_PIN]} for colour sensor")
            config.ls_pin[_LED_PIN].init(mode=Pin.OUT)
            config.ls_pin[_LED_PIN].value(1)
            config.ls_pin[_COLOUR_INT_PIN].init(mode=Pin.IN)
            config.ls_pin[_DIST_INT_PIN].init(mode=Pin.IN)

        if len(self._sensors) > 0 and any(getattr(s, 'TYPE', '') == 'Colour' for s in self._sensors):
            config = HexpansionConfig(port)
            config.ls_pin[_DIST_INT_PIN].init(mode=Pin.IN)
        return len(self._sensors) > 0


    def close(self):
        """Shutdown all sensors and release the I2C bus."""
        for s in self._sensors:
            try:
                s.reset()
            except Exception:       # pylint: disable=broad-exception-caught
                pass
        if self._port is not None:
            if len(self._sensors) > 0 and any(getattr(s, 'TYPE', '') == 'Colour' for s in self._sensors):
                if self._logging:
                    print(f"HT:LED Off port {self._port}")
                config = HexpansionConfig(self._port)
                if config is not None:
                    config.ls_pin[_LED_PIN].value(0)
                    config.ls_pin[_LED_PIN].init(mode=Pin.IN)
        self._sensors = []
        self._index = 0
        self._last_data = {}
        self._i2c = None
        self._port = None


    # ------------------------------------------------------------------
    # Sensor selection
    # ------------------------------------------------------------------

    def next_sensor(self):
        """Select the next sensor in the list."""
        if self._sensors:
            self._index = (self._index + 1) % len(self._sensors)
            self._last_data = {}
            self._read_interval_ms = getattr(self._sensors[self._index], 'READ_INTERVAL_MS', 250)
            self._type = getattr(self._sensors[self._index], 'TYPE', 'Generic')


    def prev_sensor(self):
        """Select the previous sensor in the list."""
        if self._sensors:
            self._index = (self._index - 1) % len(self._sensors)
            self._last_data = {}
            self._read_interval_ms = getattr(self._sensors[self._index], 'READ_INTERVAL_MS', 250)
            self._type = getattr(self._sensors[self._index], 'TYPE', 'Generic')


    # ------------------------------------------------------------------
    # Reading
    # ------------------------------------------------------------------

    def read_current(self) -> dict:
        """Read the currently selected sensor; cache result in last_data."""
        if not self._sensors:
            return {"Error": "no sensors"}
        self._last_data = self._sensors[self._index].read()
        #self.report_interrupt()
        return self._last_data


    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def num_sensors(self) -> int:
        """Return the number of initialised sensors."""
        return len(self._sensors)

    @property
    def current_sensor_name(self) -> str:
        """Return the name of the currently selected sensor, or 'none' if no sensors."""
        if not self._sensors:
            return "none"
        sensor = self._sensors[self._index]
        return f"{sensor.NAME}"
        #return f"{sensor.NAME}@0x{sensor.i2c_addr:02X}"

    @property
    def current_sensor_index(self) -> int:
        """Return the index of the currently selected sensor."""
        return self._index

    @property
    def last_data(self) -> dict:
        """Return the last data read from the currently selected sensor."""
        return self._last_data

    @property
    def port(self) -> int | None:
        """Return the currently open port number, or None if no port is open."""
        return self._port

    @property
    def is_open(self) -> bool:
        """True if the I2C bus is open and at least one sensor is initialised."""
        return self._i2c is not None and len(self._sensors) > 0

    def sensor_list(self) -> list[tuple[int, str]]:
        """Return [(index, name), ...] for all found sensors."""
        return [(i, s.NAME) for i, s in enumerate(self._sensors)]

    def get_sensor_by_name(self, name: str) -> SensorBase | None:
        """Return the first sensor instance whose NAME matches, or None."""
        for s in self._sensors:
            if s.NAME == name:
                return s
        return None


def format_voltage_mv(voltage_mv: int | None) -> str:
    if voltage_mv is None:
        return "--"
    sign = "-" if voltage_mv < 0 else ""
    absolute = abs(int(voltage_mv))
    whole = absolute // 1000
    fraction = (absolute % 1000) // 10
    return f"{sign}{whole}.{fraction:02d}V"


# ------------------------------------------------------------------
# Audio Engine
# ------------------------------------------------------------------


# Frequency Map Definition (A3 to A7)
# Create an indexed index list for rapid up/down navigation lookup
# 1. GUARANTEED LINEAR ORDER TUPLE (For Up/Down Buttons)
NOTE_LIST = (
    "REST",
    # Octave 2
    "C2", "C#2", "D2", "D#2", "E2", "F2", "F#2", "G2", "G#2", "A2", "A#2", "B2",
    # Octave 3
    "C3", "C#3", "D3", "D#3", "E3", "F3", "F#3", "G3", "G#3", "A3", "A#3", "B3",
    # Octave 4
    "C4", "C#4", "D4", "D#4", "E4", "F4", "F#4", "G4", "G#4", "A4", "A#4", "B4",
    # Octave 5
    "C5", "C#5", "D5", "D#5", "E5", "F5", "F#5", "G5", "G#5", "A5", "A#5", "B5",
    # Octave 6
    "C6", "C#6", "D6", "D#6", "E6", "F6", "F#6", "G6", "G#6", "A6", "A#6", "B6",
    # Octave 7
    "C7", "C#7", "D7", "D#7", "E7", "F7", "F#7", "G7", "G#7", "A7", "A#7", "B7",
    # Octave 8
    "C8", "C#8", "D8", "D#8", "E8", "F8", "F#8", "G8", "G#8", "A8", "A#8", "B8",
    # Octave 9
    "C9", "C#9", "D9", "D#9", "E9", "F9", "F#9", "G9", "G#9", "A9"
)

# 2. RUNTIME LOOKUP DATA MATRIX (Order inside here does not matter anymore)
# Structure: "NOTE_NAME": (Frequency, Base_Cycles, Total_Samples_In_Block)
NOTE_DATA = {
    "REST":  (0.0, 1, 100),
    # Octave 2
    "C2":    ( 65.41, 1, 337),   "C#2":   (69.30, 1, 318),   "D2":    (73.42, 1, 300),   "D#2":   (77.78, 1, 283),
    "E2":    ( 82.41, 1, 268),   "F2":    (87.31, 1, 253),   "F#2":   (92.50, 1, 238),   "G2":    (98.00, 1, 225),
    "G#2":   (103.83, 1, 212),  "A2":    (110.00, 1, 200),  "A#2":   (116.54, 1, 189),  "B2":    (123.47, 1, 179),
    # Octave 3
    "C3":    (130.81, 1, 169),  "C#3":   (138.59, 1, 159),  "D3":    (146.83, 1, 150),  "D#3":   (154.60, 1, 143),
    "E3":    (164.81, 1, 134),  "F3":    (174.61, 1, 126),  "F#3":   (185.00, 1, 119),  "G3":    (196.00, 1, 113),
    "G#3":   (207.65, 1, 106),  "A3":    (220.00, 1, 100),  "A#3":   (233.08, 1, 95),   "B3":    (246.94, 1, 89),
    # Octave 4
    "C4":    (261.63, 7, 590),  "C#4":   (277.18, 5, 398),  "D4":    (293.66, 3, 225),  "D#4":   (311.13, 8, 567),
    "E4":    (329.63, 2, 134),  "F4":    (349.23, 5, 316),  "F#4":   (369.99, 3, 179),  "G4":    (392.00, 1, 56),
    "G#4":   (415.30, 4, 212),  "A4":    (440.00, 1, 50),   "A#4":   (466.16, 5, 236),  "B4":    (493.88, 2, 89),
    # Octave 5
    "C5":    (523.25, 7, 295),  "C#5":   (554.37, 5, 199),  "D5":    (587.33, 6, 225),  "D#5":   (622.25, 16, 567),
    "E5":    (659.25, 2, 67),   "F5":    (698.46, 5, 158),  "F#5":   (739.99, 6, 179),  "G5":    (783.99, 2, 56),
    "G#5":   (830.61, 4, 106),  "A5":    (880.00, 1, 25),   "A#5":   (932.33, 5, 118),  "B5":    (987.77, 4, 89),
    # Octave 6
    "C6":    (1046.50, 14, 295),"C#6":   (1108.73, 10, 199),"D6":    (1174.66, 12, 225),"D#6":   (1244.51, 4, 71),
    "E6":    (1318.51, 3, 50),  "F6":    (1396.91, 5, 79),  "F#6":   (1479.98, 1, 15),  "G6":    (1567.98, 5, 70),
    "G#6":   (1661.22, 2, 27),  "A6":    (1760.00, 2, 25),  "A#6":   (1864.66, 5, 59),  "B6":    (1975.53, 5, 56),
    # Octave 7
    "C7":    (2093.00, 2, 21),  "C#7":   (2217.46, 1, 10),  "D7":    (2349.32, 4, 38),  "D#7":   (2489.02, 8, 71),
    "E7":    (2637.02, 6, 50),  "F7":    (2793.83, 10, 79), "F#7":   (2959.96, 4, 30),  "G7":    (3135.96, 5, 35),
    "G#7":   (3322.44, 4, 27),  "A7":    (3520.00, 4, 25),  "A#7":   (3729.31, 10, 59), "B7":    (3951.07, 10, 56),
    # Octave 8
    "C8":    (4186.01, 4, 21),  "C#8":   (4434.92, 2, 10),  "D8":    (4698.63, 8, 38),  "D#8":   (4978.03, 16, 71),
    "E8":    (5274.04, 12, 50), "F8":    (5587.65, 20, 79), "F#8":   (5919.91, 8, 30),  "G8":    (6271.93, 10, 35),
    "G#8":   (6644.88, 8, 27),  "A8":    (7040.00, 8, 25),  "A#8":   (7458.62, 20, 59), "B8":    (7902.13, 20, 56),
    # Octave 9
    "C9":    (8372.01, 8, 21),  "C#9":   (8869.84, 4, 10),  "D9":    (9397.27, 16, 38), "D#9":   (9956.06, 32, 71),
    "E9":    (10548.08, 24, 50), "F9":   (11175.30, 40, 79), "F#9":  (11839.82, 16, 30), "G9":   (12543.85, 20, 35),
    "G#9":   (13289.75, 16, 27), "A9":   (14080.00, 16, 25)
}

_WAVEFORM_SINE = 0
_WAVEFORM_SQUARE = 1
_WAVEFORM_TRIANGLE = 2

# 1. Windows-Style Startup Tune (Nostalgic rising progression)
SEQ_STARTUP = [
    ("D#4", 150, 4, _WAVEFORM_SINE), ("A#4", 150, 4, _WAVEFORM_SINE), ("F5", 150, 5, _WAVEFORM_SINE), ("A#5", 150, 5, _WAVEFORM_SINE),
    ("D#6", 200, 6, _WAVEFORM_SINE), ("A#5", 200, 5, _WAVEFORM_SINE), ("F6", 600, 7, _WAVEFORM_SINE)
]

# 2. Keypress UI Tick (Tiny, fast, high-pitched chirp)
SEQ_KEYPRESS = [
    ("A6", 50, 3, _WAVEFORM_SQUARE)
]

# 3. Warning Alert (Alternating high-low tones with a brief gap)
SEQ_WARNING = [
    ("A5", 150, 6, _WAVEFORM_SINE), ("REST", 50, 0, _WAVEFORM_SINE), ("E5", 150, 6, _WAVEFORM_SINE),
    ("REST", 50, 0, _WAVEFORM_SINE), ("A5", 150, 6, _WAVEFORM_SINE), ("REST", 50, 0, _WAVEFORM_SINE), ("E5", 150, 6, _WAVEFORM_SINE)
]

# 4. Error Alert (Harsh low-pitch buzz)
SEQ_ERROR = [
    ("A3", 120, 7, _WAVEFORM_SINE), ("REST", 40, 0, _WAVEFORM_SINE), ("A3", 300, 8, _WAVEFORM_SINE)
]

# 5. Task Complete Fanfare (Bright upbeat major arpeggio)
SEQ_COMPLETE = [
    ("C5", 100, 4, _WAVEFORM_SQUARE), ("E5", 100, 4, _WAVEFORM_TRIANGLE), ("G5", 100, 4, _WAVEFORM_TRIANGLE), ("C6", 400, 7, _WAVEFORM_SINE)
]

# 6. Waiting Mode / Heartbeat (Gentle rhythmic background pulses)
SEQ_WAITING = [
    ("E4", 80, 3, _WAVEFORM_SINE), ("REST", 100, 0, _WAVEFORM_SINE), ("E4", 80, 3, _WAVEFORM_SINE), ("REST", 100, 0, _WAVEFORM_SINE)
]

class AsyncI2SSequencer:
    def __init__(self, i2s_id=0, sck_pin=16, ws_pin=17, sd_pin=18, sample_rate: int=22050, max_duration_ms: int=1000, volume: int=5, channels: int=1):
        from machine import I2S

        self.sample_rate: int = sample_rate
        self._channels: int = channels  # Mono output channel count
        self._volume: int = volume
        # Determine absolute worst-case maximum buffer limits
        # 16-bit mono = 2 bytes per sample
        bytes_per_sample: int  = 2 * self._channels
        self.max_samples: int  = int((sample_rate * max_duration_ms) / 1000)
        max_buffer_bytes: int = self.max_samples * bytes_per_sample

        # 2. Permanent static heap allocations (Created EXACTLY once)
        # We pre-allocate the maximum sizes needed so the GC never touches this at runtime
        self._static_output_buffer: bytearray = bytearray(max_buffer_bytes)
        self._static_base_block: bytearray = bytearray(2000 * bytes_per_sample) # Caps base block at ~2000 samples

        # Create persistent memoryviews over our static memory blocks
        self.out_view: memoryview = memoryview(self._static_output_buffer)
        self.base_view: memoryview = memoryview(self._static_base_block)

        # Initialize I2S peripheral in standard blocking mode (managed asynchronously)
        self.audio_out = I2S(
            i2s_id,
            sck=Pin(sck_pin),
            ws=Pin(ws_pin),
            sd=Pin(sd_pin),
            mode=I2S.TX,
            bits=16,
            format=I2S.MONO,
            rate=self.sample_rate,
            ibuf=16384
        )

        self.current_task = None
        self.interrupted = asyncio.Event()


    @property
    def volume(self) -> int:
        """Return the current volume level (0 to 10)."""
        return self._volume


    @volume.setter
    def volume(self, value: int):
        """Set the volume level (0 to 10)."""
        if not (0 <= value <= 10):
            raise ValueError("Volume must be between 0 and 10")
        print(f"[Audio] Volume set to {value}/10")
        self._volume = value


    def _generate_wave(self, note: int, duration_ms: int, volume: int, waveform: int):
        """Calculates high-precision multi-cycle blocks for perfect high frequencies.
        Fills the pre-allocated static buffers in-place.
        Returns a sliced memoryview pointing to valid data limits.
        """

        # 1. Instant O(1) Data Retrieval - No math loops!
        note_name = NOTE_LIST[note] if 0 <= note < len(NOTE_LIST) else "REST"
        frequency, num_base_cycles, samples_in_base_block = NOTE_DATA.get(note_name, (0, 0, 0))

        if 1 > samples_in_base_block:
            return None, 0

        from math import pi, sin
        start_time = time.ticks_ms()

        bytes_per_sample = 2 * self._channels


        # 2. Hard Safety Check: Protect against base block buffer overflows
        if samples_in_base_block * bytes_per_sample > len(self._static_base_block):
            print(f"HT:[Error] Base block size {samples_in_base_block} exceeds static buffer limit {len(self._static_base_block) // bytes_per_sample} samples")
            samples_in_base_block = len(self._static_base_block) // bytes_per_sample


        # 3. Calculate target duration blocks
        block_duration_ms = (samples_in_base_block / self.sample_rate) * 1000
        #num_blocks_needed = max(1, int(round(duration_ms / block_duration_ms)))
        num_blocks_needed = max(1, int(duration_ms / block_duration_ms))
        total_samples = num_blocks_needed * samples_in_base_block

        # Static buffer safety check
        if total_samples > self.max_samples:
            print(f"HT:[Warning] Note truncated from {total_samples}={num_blocks_needed}*{samples_in_base_block} based on block_duration_ms of {block_duration_ms} to fit within static buffer limits ({self.max_samples} samples)")
            num_blocks_needed = self.max_samples // samples_in_base_block
            total_samples = num_blocks_needed * samples_in_base_block

        # 4. Render the waveform directly into our permanent static base block slice

        # Human Auditory Curve Mapping (0-100 UI to 0.0-1.0 Exponential Scalar)
        # Using a square curve (k=2) for natural log feel on small speakers
        amplitude = int(32767 * ((self._volume * volume) / 100.0) ** 2)

        def _store_sample(n, sample):
            """Helper to store a sample in the static base block for mono/stereo."""
            self._static_base_block[n*2*self._channels:n*2*self._channels+2] = sample.to_bytes(2, 'little', True)
            if self._channels == 2:  # Stereo
                self._static_base_block[n*4+2:n*4+4] = sample.to_bytes(2, 'little', True)

        if amplitude == 0:
            # Fill with silence if amplitude is zero
            for n in range(samples_in_base_block):
                _store_sample(n, 0)
        elif waveform == _WAVEFORM_SINE:
            # sine wave: standard sin() function
            omega = (2 * pi * num_base_cycles) / samples_in_base_block
            for n in range(samples_in_base_block):
                sample = int(amplitude * sin(n * omega))#
                _store_sample(n, sample)
        elif waveform == _WAVEFORM_TRIANGLE:
            # triangle wave: linear ramp up and down
            half_cycle = samples_in_base_block // (2 * num_base_cycles)
            slope = (2 * amplitude) / half_cycle
            for n in range(samples_in_base_block):
                if n % (2 * half_cycle) < half_cycle:
                    sample = int(-amplitude + slope * (n % (2 * half_cycle)))
                else:
                    sample = int(amplitude - slope * (n % (2 * half_cycle) - half_cycle))
                _store_sample(n, sample)
        elif waveform == _WAVEFORM_SQUARE:
            # square wave: simple high/low toggle
            half_cycle = samples_in_base_block // (2 * num_base_cycles)
            for n in range(samples_in_base_block):
                if n % (2 * half_cycle) < half_cycle:
                    sample = amplitude
                else:
                    sample = -amplitude
                _store_sample(n, sample)
        else:
            raise ValueError("Unsupported waveform type")


        # 5. Extract strict target window slices over the pre-allocated memory blocks
        block_len = samples_in_base_block * bytes_per_sample
        active_base_slice = self.base_view[0 : block_len]
        total_len = total_samples * bytes_per_sample
        active_output_slice = self.out_view[0 : total_len]

        # Seed the first block using fast native in-place copy
        active_output_slice[0:block_len] = active_base_slice

        # Fast, zero-allocation exponential doubling duplication loop
        current_filled = block_len

        while current_filled < total_len:
            next_chunk_size = min(current_filled, total_len - current_filled)
            active_output_slice[current_filled : current_filled + next_chunk_size] = active_output_slice[0 : next_chunk_size]
            current_filled += next_chunk_size

        print(f"[Audio] Generated {total_samples} samples for {frequency}Hz with base_block {samples_in_base_block} in {time.ticks_diff(time.ticks_ms(), start_time)}ms")

        # Return the exact slice containing our generated data
        return active_output_slice, block_len


    async def _play_sequence_task(self, sequence):
        """Asynchronous worker that iterates through the sequence definitions."""
        try:
            for note, duration_ms, amplitude, waveform in sequence[:128]: # Strict 128 tone limit enforced
                # Instant termination check before executing the next tone segment
                if self.interrupted.is_set():
                    print("[Audio] Sequence interrupted, halting playback.")
                    break

                # Generate the sound chunk dynamically to conserve runtime RAM
                wave_data, block_len = self._generate_wave(note, duration_ms, amplitude, waveform)

                if wave_data is None:
                    continue

                # write the generated wave data in multiples of the base block size to the I2S output
                # so that if interrutped we end on a zero crossing.
                replay_len = block_len
                while replay_len < 4096 and replay_len < len(wave_data):
                    replay_len += block_len
                replay_len = min(replay_len, len(wave_data))
                # Write to I2S loop chunks. Co-loading sleep allows asyncio yielding.
                bytes_written = 0
                while bytes_written < len(wave_data):
                    if self.interrupted.is_set():
                        print("[Audio] Sequence interrupted during write, halting playback.")
                        return
                    if 0 < bytes_written:
                        # no waiting for the first write, subsequent writes yield control to avoid blocking
                        await asyncio.sleep_ms(10)

                    # Write block yields control via minimal delay to prevent blocking core execution
                    #irq_state = disable_irq()
                    written = self.audio_out.write(wave_data[bytes_written:bytes_written + replay_len])
                    #enable_irq(irq_state)

                    bytes_written += written
                    #print(f"[Audio] Wrote {written} bytes to I2S, total {bytes_written}/{len(wave_data)}")


        except asyncio.CancelledError:
            print("[Audio] Sequence task cancelled.")
            #pass # Gracefully handle task termination on quick-switch interruptions


    def play(self, sequence):
        """Triggers a sequence immediately, cutting off any active sounds."""
        self.interrupted.set() # Flag current running task loop to halt execution

        if self.current_task and not self.current_task.done():
            print("[Audio] Interrupting current sequence...")
            self.current_task.cancel()

        self.interrupted.clear()
        # Schedule the new task sequence immediately onto the MicroPython asyncio loop
        self.current_task = asyncio.create_task(self._play_sequence_task(sequence))


    def play_note_by_index(self, index: int, duration_ms: int=1000, amplitude: int=10):
        """
        Accepts an integer index, looks up the note name,
        and plays it immediately while interrupting any prior notes.
        """
        # Constrain index safely within bounds of our available notes
        safe_index = max(0, min(index, len(NOTE_LIST) - 1))
        note_name = NOTE_LIST[safe_index]

        # Format the single note request as a single-element sequence loop array
        single_note_sequence = [(note_name, duration_ms, amplitude, _WAVEFORM_TRIANGLE)]

        # Pull frequency metrics out of dictionary purely for tracking print logs
        freq, _, _ = NOTE_DATA.get(note_name, (0, 0, 0))
        print(f"HT:Pitch Test Index: {safe_index} -> Note: {note_name} ({freq} Hz)")
        self.play(single_note_sequence)


    def deinit(self):
        """Safely release the hardware peripheral."""
        if self.current_task:
            self.current_task.cancel()
        self.audio_out.deinit()

__app_export__ = HexTestApp
