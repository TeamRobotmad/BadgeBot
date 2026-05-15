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
from machine import I2C, Pin, mem32, disable_irq, enable_irq

import settings as platform_settings
from app_components import Menu, button_labels, clear_background, label_font_size
from app_components.notification import Notification
from events.input import BUTTON_TYPES, Buttons

from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.hexpansion.events import HexpansionMountedEvent, HexpansionRemovalEvent
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent,
                                     RequestStopAppEvent)
from system.hexpansion.header import HexpansionHeader
from system.scheduler import scheduler
try:
    from system.hexpansion.util import get_app_by_slot, get_slots_by_vid_pid
except ImportError:
    # In case we are running on old version of BadgeOS, where these functions are not available, define stubs that return None or empty lists.
    from system.hexpansion.util import detect_eeprom_addr

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
        for port in range(1, _NUM_HEXPANSION_SLOTS + 1):
            try:
                i2c = I2C(port)
                # Autodetect eeprom addr
                eeprom_addr, addr_len = detect_eeprom_addr(i2c)
                if eeprom_addr is None:
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
from micropython import const

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
_MIN_BADGEOS_VERSION = [1, 9, 0]     # v1.9.0 is required to be able to read the EEPROM with 16-bit addressing

SETTINGS_NAME_PREFIX = "hextest."  # Prefix for settings keys in EEPROM

# HexTest Hexpansion constants
# Hardware defintions:
_NUM_HEXPANSION_SLOTS = 6

# Constants for rotation rate measurement and motor test mode.
_ROTATION_RATE_MEASUREMENT_PERIOD_MS = 2500     # how often to update the displayed rotation rate measurement in ms (tradeoff between display responsiveness and stability of the reading)
_DEFAULT_ROTATION_RATE_EMITTER_DUTY = 20        # default duty cycle for the IR emitter when doing rate testing, 0-255 (0=off, 255=full on)
_DEFAULT_SPOKES_PER_ROTATION = 3                # number of times the photodiode will be triggered per full rotation of the wheel
_MOTOR_TEST_BACKGROUND_UPDATE_PERIOD = 1000     # background update period in ms to use during motor test mode (tradeoff between display responsiveness and CPU load)
_ROTATION_RATE_EMITTER_PINS = [1, 2]            # LS_B & LS_C pins used to drive the IR emitter for rotation rate testing
_ROTATION_RATE_SENSOR_PINS = [0, 1]             # HS_F & HS_G pins used to read the phottransistors for rotation rate testing
_ROTATION_RATE_SENSOR_ENABLE_PINS = [3, 4]      # LS_D & LS_E pins used to enable the phototransistors for rotation rate testing (set to output and high to enable, input to disable)
_IR_EMITTER_PWM_STEP_SIZE = 2                   # Step size for adjusting IR emitter brightness in manual mode, 0-255 (0=off, 255=full on)
_POWER_SCALE_FACTOR = 66
_MOTOR_PWM_FREQUENCY = 20000                    # Default PWM frequency to set on the HexDrive for testing, in Hz.

# Rotation Rate Auto scan configuration
_AUTO_SCAN_STEPS       = 60     # Number of power levels to test during auto scan
_AUTO_SCAN_SETTLE_MS   = 500    # ms to wait after setting power before starting actual measurement period
_AUTO_SCAN_MEASURE_MS  = 5000   # ms measurement window per step (maximum)
_AUTO_RESULTS_FILENAME = "mtrtst.csv"
_AUTO_RESULTS_DEST_LABELS = ("badge fs", "hex fs")

# App states
STATE_MENU = 0
STATE_MESSAGE = 1         # Message display
STATE_SETTINGS = 2        # Edit Settings
STATE_SENSOR = 3          # Sensor Test
STATE_MOTOR_TEST = 4      # Motor Test

# App states where user can minimise app (Menu, Message, Logo)
MINIMISE_VALID_STATES = [STATE_MENU, STATE_MESSAGE]

# Main Menu Items
MAIN_MENU_ITEMS = ["Sensor Test", "Motor Test", "Settings", "About","Exit"]
MENU_ITEM_SENSOR_TEST = 0
MENU_ITEM_MOTOR_TEST = 1
MENU_ITEM_SETTINGS = 2
MENU_ITEM_ABOUT = 3
MENU_ITEM_EXIT = 4

DEFAULT_BACKGROUND_UPDATE_PERIOD = 100    # mS when not moving
_LOGGING = True
_AUTO_REPEAT_MS = 200       # Time between auto-repeats, in ms
_AUTO_REPEAT_COUNT_THRES = 10 # Number of auto-repeats before increasing level
_AUTO_REPEAT_SPEED_LEVEL_MAX = 4  # Maximum level of auto-repeat speed increases
_AUTO_REPEAT_LEVEL_MAX = 3  # Maximum level of auto-repeat digit increases


# Pages of information to show for each sensor (can be switched with up/down buttons)
_PAGE_RAW = 0
_PAGE_STATS = 1
_PAGE_DATA = 2
_PAGE_CAL = 3
_PAGE_NAMES = {
    0: "Raw",
    1: "Stats",
    2: "Data",
    3: "Cal",
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
    VERSION = 1         # Increment this when making changes to the app that require the hexpansion app to be re-flashed with the new code.

    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()
        if config is None:
            raise ValueError("HexTestApp requires a HexpansionConfig on initialisation")

        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            #print(f"D:S/W {ver}")
            # e.g. v1.9.0-beta.1
            if ver >= _MIN_BADGEOS_VERSION:
                pass
            else:
                raise RuntimeError("HexTestApp requires BadgeOS Upgrade")
        except Exception as e:      # pylint: disable=broad-except
            print(f"T:Ver check failed {e}!")

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

        # Settings - common settings first, then each module registers its own later
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
        self._rotation_rate_spokes: int = _DEFAULT_SPOKES_PER_ROTATION

        # Auto scan state
        self._auto_mode: bool = False             # True = auto scanning, False = manual
        self._auto_direction: int = 1             # 1 = forwards, -1 = reverse
        self._auto_step: int = 0                  # current step index (0.._AUTO_SCAN_STEPS-1)
        self._auto_settling: bool = True          # True = in settle phase, False = in measure phase
        self._rotation_detected: bool = False     # True once motion has been observed during auto scan
        self._auto_results: list[tuple[int, list[int], int | None]] = []   # list of (power, rpm list, current mA)
        self._auto_max_rpm: int = 0               # max rpm seen during scan
        self._auto_max_current_ma: int = 0        # max current seen during scan
        self._auto_last_current_ma: int = 0       # latest current sampled in auto mode
        self._auto_done: bool = False             # True = scan complete
        self._motor_calibration_fit: list[tuple[float, float] | None] = []  # list of (slope, intercept) fits, indexed by motor number

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

        if MySetting is not None:
            # General settings
            self.settings['logging']       = MySetting(self.settings, _LOGGING, False, True)
            self.settings["path"]          = MySetting(self.settings, 0, 0, len(_AUTO_RESULTS_DEST_LABELS) - 1, labels=_AUTO_RESULTS_DEST_LABELS)

            self.update_settings()

        self.HEXPANSION_TYPES = [HexpansionType(0xCBCB, "HexDrive",                motors=2, servos=4, sub_type="Uncommitted" ),
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
        print(f"T:HexTest App V{self.VERSION} by RobotMad on port {self.config.port}")

        self._rotation_rate_enable(False)  # start with rotation rate emitter and sensors off until we enter motor test mode

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
    def _rotation_rate_rounding(self) -> int:
        return (self._rotation_rate_measurement_period * self._rotation_rate_spokes) // 2

    @property
    def rotation_rate_emitter_duty(self) -> int:
        """Duty cycle (0-255) for the IR emitter when doing rotation rate testing."""
        return self._rotation_rate_emitter_duty

    @rotation_rate_emitter_duty.setter
    def rotation_rate_emitter_duty(self, value: int):
        self._rotation_rate_emitter_duty = value
        if self.config is not None:
            for pin_num in _ROTATION_RATE_EMITTER_PINS:
                self.config.ls_pin[pin_num].duty(self._rotation_rate_emitter_duty)

    # ------------------------------------------------------------------

    def update_settings(self):
        """Update settings from EEPROM."""
        if self.logging:
            print("T:Updating settings from EEPROM")
        for s in self.settings:
            self.settings[s].v = platform_settings.get(f"{SETTINGS_NAME_PREFIX}{s}", self.settings[s].d)
            if self.logging:
                print(f"Setting {s} = {self.settings[s].v}")


    def _rotation_rate_enable(self, enable: bool = True) -> bool:
        if self.config is None:
            return False
        try:
            if enable:
                if self._logging:
                    print("T:Enabling rotation rate emitters and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self.config.ls_pin[pin_num].init(mode=ePin.PWM)  # Set LS pins to output mode to turn on the IR emitters
                    self.config.ls_pin[pin_num].duty(self.rotation_rate_emitter_duty)  # Set LS pins to the current duty cycle to drive the IR emitters)
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self.config.ls_pin[pin_num].init(mode=Pin.OUT)  # Set LS pins to output mode to enable the phototransistors for rotation rate measurement
                    self.config.ls_pin[pin_num].value(1)  # Set LS enable pins high to turn on the phototransistors for rotation rate measurement
            else:
                if self._logging:
                    print("T:Disabling rotation rate emitters and sensors")
                for pin_num in _ROTATION_RATE_EMITTER_PINS:
                    self.config.ls_pin[pin_num].init(mode=Pin.IN)  # Set LS pins to input mode to turn off the IR emitters
                for pin_num in _ROTATION_RATE_SENSOR_ENABLE_PINS:
                    self.config.ls_pin[pin_num].init(mode=Pin.IN)  # Set LS pins to input mode to turn off the phototransistors for rotation rate measurement

            for pin_num in _ROTATION_RATE_SENSOR_PINS:
                self.config.pin[pin_num].init(mode=Pin.IN)  # Set HS pins to input mode to read the phototransistors for rotation rate measurement
        except AttributeError:
            pass  # Simulator Pin stubs lack .init()
        return True



    def deinitialise(self) -> bool:
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
        return True


    def _exit_app(self):
        """ Clean up and exit the app, returning to the main menu."""

        eventbus.emit(RequestStopAppEvent(self))


    # ------------------------------------------------------------------
    # Async event handlers (registered directly on eventbus)
    # ------------------------------------------------------------------

    async def _handle_removal(self, event: HexpansionRemovalEvent):
        if event.port == self._hexdrive_in_use_port:
            if self._logging:
                print(f"H:Hexpansion removed from port {event.port}")
            self._hexdrive_app = None
            self._hexdrive_in_use_port = None
            self._hexdrive_ports.remove(event.port)
            self.notification = Notification("HexDrive Removed")
            if self.current_state == STATE_MOTOR_TEST:
                self._stop_motor_test_mode()
            elif self.current_state == STATE_SENSOR:
                self._stop_sensor_test_mode()


    async def _handle_mounted(self, event: HexpansionMountedEvent):
        if self._foreground and self.current_state in [STATE_MESSAGE, STATE_MENU]:
            if self._logging:
                print(f"H:Hexpansion mounted on port {event.port}")
            # Check if it is a HexDrive we can use for testing
            # make a simple list of vid, pid pairs that we can check against efficiently
            vid_pid_pairs = [(type.vid, type.pid) for type in self.HEXPANSION_TYPES]
            if hasattr(event, "header") and (event.header.vid, event.header.pid) in vid_pid_pairs:
                if self.logging:
                    print(f"H:Attempting to use newly mounted HexDrive on port {event.port}")

                if self._motor_test_start():
                    if self.logging:
                        print(f"H:Successfully started motor test with newly mounted HexDrive on port {event.port}")
                    self.current_state = STATE_MOTOR_TEST
                    self.set_menu(None)
                else:
                    if self.logging:
                        print(f"H:Failed to start motor test with newly mounted HexDrive on port {event.port}")
            else:
                # Old BadgeOS didn't include the header in the event - so assume it is a HexDrive
                if self._motor_test_start():
                    if self.logging:
                        print(f"H:Successfully started motor test with newly mounted HexDrive on port {event.port} (no header in event, assumed HexDrive)")
                    self.current_state = STATE_MOTOR_TEST
                    self.set_menu(None)


    async def _gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            if self.logging:
                print(f"HexTest gained focus in state {self.current_state}")
            self._foreground = True


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
                    print(f"D:{self.config.port}:Stop")
                self.deinitialise()
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
        for hexpansion_type in self.HEXPANSION_TYPES:
            if self.logging:
                print(f"T:Looking for {hexpansion_type.name} (VID:PID {hexpansion_type.vid:04X}:{hexpansion_type.pid:04X}, Motors: {hexpansion_type.motors}, Servos: {hexpansion_type.servos})")
            ports = get_slots_by_vid_pid(hexpansion_type.vid, hexpansion_type.pid)
            if ports:
                if self.logging:
                    print(f"T:Found {hexpansion_type.name} on port(s): {ports}")
                self._hexdrive_ports.extend(ports)
                break

        for port in self._hexdrive_ports:
            hexpansion_app = _as_hexdrive_app(get_app_by_slot(port))
            if hexpansion_app is not None:
                self._hexdrive_in_use_port = port
                if self.logging:
                    print(f"T:Found HexDrive app to test on port {port}")
                self._hexdrive_app = hexpansion_app
                break

        if self._hexdrive_app is not None:
            #Setup UUT = HexDrive
            try:
                if self._hexdrive_app.initialise() and self._hexdrive_app.set_power(True) and self._hexdrive_app.set_freq(_MOTOR_PWM_FREQUENCY):
                    self._hexdrive_app.set_keep_alive(2000)   # Updates can be quite slow as we are using the draw function
            except AttributeError:
                pass

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
                        print(f"T:Not setting up rotation rate counter on pin {pin_num} (GPIO {gpio_num}) as this HexDrive type only has {hexpansion_type.motors} motors")
                    continue
                counter = Counter(None, gpio_num, filter_ns=1000000, logging=False)  # auto-select PCNT unit
                if counter is not None and counter.unit is not None:
                    self._rotation_rate_counters.append(counter)
                else:
                    if self.logging:
                        print(f"T:Failed to allocate PCNT counter for pin {pin_num} (GPIO {gpio_num})")
                    self.notification = Notification("PCNT Init     Failed")
                    # deinit any counters we did manage to create before returning
                    for c in self._rotation_rate_counters:
                        if c is not None:
                            c.deinit()
                    self._rotation_rate_counters = []
                    return False
                if self.logging:
                    print(f"T:Rate counter {counter}")
            self._rotation_rate_measurement_period_elapsed = 0
            self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
            return True
        if self.logging:
            print("T:Failed to initialise for motor test mode - no hexdrive to test")
        self.notification = Notification("HexDrive   not Found")
        return False



    def _stop_motor_test_mode(self):
        if self._logging:
            print("T:Stopping Motor Test mode and cleaning up")
        self._auto_mode = False
        self._auto_done = False
        self._rotation_rate_motor_power = 0
        self._ina226_reading = {}
        self._reset_ina226_accumulators()
        if self._ina226 is not None:
            if self._ina226_sensor_mgr is not None:
                try:
                    self._ina226_sensor_mgr.close()
                except Exception as exc:          # pylint: disable=broad-exception-caught
                    print("T:INA226 sensor manager close failed:", exc)
                self._ina226_sensor_mgr = None
        self._ina226 = None

        if self._hexdrive_app is not None:
            try:
                self._hexdrive_app.set_motors((0, 0))
                self._hexdrive_app.set_power(False)
            except AttributeError as e:
                print(f"T:Failed to set motor outputs off {e}")
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
            print("T:Stopping Sensor Test mode and cleaning up")
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
            for port in range(1, 7):
                if not mgr.open(port):
                    mgr.close()
                    if self._logging:
                        print(f"T:INA226 - no sensors found on port {port}")
                    continue
                # Find the first INA226 sensor in the discovered list
                sensor = mgr.get_sensor_by_name("INA226")
                if sensor is not None:
                    self._ina226 = sensor
                    self._ina226_sensor_mgr = mgr
                    if self._logging:
                        print(f"T:INA226 found @ 0x{sensor.i2c_addr:02X} on port {port}")
                    return True
                # No INA226 found; close the manager
                mgr.close()
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"T:INA226 init failed: {e}")
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
                print(f"T:INA226 sample error: {e}")
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
        self._auto_step += 1
        self.refresh = True
        if self._auto_step >= _AUTO_SCAN_STEPS:
            # Scan complete — stop motors
            self._auto_done = True
            self._rotation_detected = False
            self._rotation_rate_motor_power = 0
            self._auto_direction *= -1  # reverse direction for next scan
            #self._auto_fit_calculate()
            #self._save_auto_results_csv()
        else:
            # Advance to next power level
            self._rotation_rate_motor_power = self._auto_direction * (65535 * self._auto_step) // (_AUTO_SCAN_STEPS - 1)
        self._rotation_rate_measurement_period_elapsed = 0
        self._auto_settling = True

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
                    print(f"T:Error: checking notification status: {e}")

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
                        print("T:Menu is animating")
                    self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
            self.minimise()
        elif self.current_state == STATE_MESSAGE:
            self._update_state_message(delta)
        elif self.current_state == STATE_SETTINGS:
            self._settings_mgr_update(delta)
        elif self.current_state == STATE_MOTOR_TEST:
            self._motor_test_update(delta)


        if self.current_state != self.previous_state:
            if self.logging:
                print(f"T:State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # something has changed - so worth redrawing
            self.refresh = True


    def _update_state_message(self, delta: int):      # pylint: disable=unused-argument
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            if self.logging:
                print("T:Message acknowledged by user")
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
            self._rotation_rate_motor_power = 0
            self._auto_last_current_ma = 0
            self._rotation_rate_measurement_period_elapsed = 0
            self._reset_ina226_accumulators()
            for counter in self._rotation_rate_counters:
                if counter is not None:
                    counter.value(0)      # reset counter
            if self._auto_mode:
                # Switch back to manual
                #self._show_auto_results_fit()
                self._rotation_rate_measurement_period = _ROTATION_RATE_MEASUREMENT_PERIOD_MS
                self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                self._auto_mode = False
                self._auto_done = False
            else:
                # Start auto scan
                self._auto_mode = True
                self._auto_done = False
                self._auto_step = 0
                self._rotation_rate_measurement_period = _AUTO_SCAN_MEASURE_MS
                self._auto_settling = True
                self._auto_results = []
                self._auto_max_rpm = 0
                self._auto_max_current_ma = 0
                self._rotation_detected = False
            self.refresh = True
            return

        if self._auto_mode:
            if not self._auto_done:
                self._rotation_rate_measurement_period_elapsed += delta
                if self._auto_settling:
                    if self._rotation_rate_measurement_period_elapsed >= _AUTO_SCAN_SETTLE_MS:
                        # Settle phase done — discard counter and start measuring
                        count = 0
                        for counter in self._rotation_rate_counters:
                            if counter is not None:
                                count += counter.value(0)  # read-and-reset to discard
                        if count == 0 and not self._rotation_detected:

                            # There has been no motion from any motors - so we can skip the measure phase and move straight to the next power level
                            current_ma = self._consume_ina226_average()
                            if current_ma is not None:
                                current_abs = abs(current_ma)
                                self._auto_last_current_ma = current_ma
                                if current_abs > self._auto_max_current_ma:
                                    self._auto_max_current_ma = current_abs
                            power = self._rotation_rate_motor_power
                            self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                            if self._logging:
                                print(f"T:Auto Scan Step {self._auto_step}/{_AUTO_SCAN_STEPS} - Power: {power}, Rate: 0 rpm, Current: {current_ma}mA")
                            self._auto_results.append((power//_POWER_SCALE_FACTOR, [0] * len(self._rotation_rate_counters), current_ma))
                            self._auto_rotation_rate_step()

                        else:
                            self._rotation_detected = True
                            # estimate how long we need to measure for based on the count we got during the settle period, to ensure we get a good RPM (2%)
                            # reading even at low speeds, while still keeping the overall scan time reasonable#
                            cpm = (60000 * count) // self._rotation_rate_measurement_period_elapsed # rounded down - never displayed
                            self._rotation_rate_measurement_period = min(_AUTO_SCAN_MEASURE_MS, (60000 * 50) // cpm) if cpm > 0 else _AUTO_SCAN_MEASURE_MS
                            self._rotation_rate_measurement_period_elapsed = 0
                            self._auto_settling = False
                            self._reset_ina226_accumulators()
                else:
                    if self._rotation_rate_measurement_period_elapsed >= self._rotation_rate_measurement_period:
                        # Measure phase done — read counter and record result
                        self._rotation_rate_rpms = [0] * len(self._rotation_rate_counters)
                        for index, counter in enumerate(self._rotation_rate_counters):
                            if counter is not None:
                                count = counter.value(0)
                                rpm = ((60000 * count) + self._rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self._rotation_rate_spokes)
                                if rpm > self._auto_max_rpm:
                                    self._auto_max_rpm = rpm
                                self._rotation_rate_rpms[index] = rpm

                        ### duplicate of block above - could be a method
                        current_ma = self._consume_ina226_average()
                        if current_ma is not None:
                            current_abs = abs(current_ma)
                            self._auto_last_current_ma = current_ma
                            if current_abs > self._auto_max_current_ma:
                                self._auto_max_current_ma = current_abs
                        power = self._rotation_rate_motor_power
                        if self._logging:
                            print(f"T:Auto Scan Step {self._auto_step}/{_AUTO_SCAN_STEPS} - Power: {power}, Rates: {self._rotation_rate_rpms} rpm, Current: {current_ma}mA")
                        self._auto_results.append((power//_POWER_SCALE_FACTOR, self._rotation_rate_rpms, current_ma))
                        self._auto_rotation_rate_step()

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
                        self._rotation_rate_rpms[index] = ((60000 * count) + self._rotation_rate_rounding) // (self._rotation_rate_measurement_period_elapsed * self._rotation_rate_spokes)
                self._rotation_rate_measurement_period_elapsed = 0
                self._consume_ina226_average()
                #if self.logging:
                #    print(f"T:Rotation Rates: {self._rotation_rate_rpms}")

        # Manual mode button handling
        if self.button_states.get(BUTTON_TYPES["UP"]):
            self.button_states.clear()
            self.rotation_rate_emitter_duty = min(255, self.rotation_rate_emitter_duty + _IR_EMITTER_PWM_STEP_SIZE)
            if self.logging:
                print(f"T:IR+Emitter Duty: {self.rotation_rate_emitter_duty}")
            self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            self.button_states.clear()
            self.rotation_rate_emitter_duty = max(0, self.rotation_rate_emitter_duty - _IR_EMITTER_PWM_STEP_SIZE)
            if self.logging:
                print(f"T:IR-Emitter Duty: {self.rotation_rate_emitter_duty}")
            self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
            self.button_states.clear()
            self._rotation_rate_motor_power = min(65535, self._rotation_rate_motor_power + 1000)
            if self.logging:
                print(f"T:Motor+Power: {self._rotation_rate_motor_power}")
            self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["LEFT"]):
            self.button_states.clear()
            self._rotation_rate_motor_power = max(-65535, self._rotation_rate_motor_power - 1000)
            if self.logging:
                print(f"T:Motor-Power: {self._rotation_rate_motor_power}")
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
            if self.current_state == STATE_MESSAGE:
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
        if self._auto_mode:
            self._draw_auto_scan(ctx)
            return
        #print("DRAWING")
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
            lines += [f"I:{self._ina226_reading.get('mA', 0)}mA"]
            colours += [(0.3, 0.8, 1.0)]
            #lines += [f"V:{self._ina226_reading.get('mV', 0)}mV"]
            #colours += [(0.3, 0.8, 1.0)]
        else:
            lines += [""]
            colours += [(0.3, 0.8, 1.0)]
        self.draw_message(ctx, lines, colours, label_font_size)
        button_labels(ctx, up_label="IR+", down_label="IR-", cancel_label="Back",
                      left_label="Pwr-", right_label="Pwr+", confirm_label="Auto")


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

        n = len(self._auto_results)
        max_rpm = self._auto_max_rpm if self._auto_max_rpm > 0 else 1
        max_current_ma = self._auto_max_current_ma if self._auto_max_current_ma > 0 else 1

        if n > 1:
            # Plot data points as small bars.
            # Auto-scan results may contain either a scalar RPM or a list/tuple
            # of per-counter RPMs. Reduce multi-counter readings to a single
            # scalar for this chart by using the maximum measured RPM.
            bar_w = max(1, chart_w // _AUTO_SCAN_STEPS)
            for i in range(n):
                power, rpms, current_ma = self._auto_results[i]
                x = chart_left + (abs(power) * chart_w) // (65536//_POWER_SCALE_FACTOR)
                for index, rpm in enumerate(rpms):
                    h = (rpm * chart_h) // max_rpm
                    if h > 0:
                        # colour by index to differentiate multiple counters if present
                        ctx.rgb(*self._colour_for_index(index)).rectangle(x, chart_bottom - h - 1, bar_w, 2).fill()
                if current_ma is not None:
                    current_h = (abs(current_ma) * chart_h) // max_current_ma
                    marker_y = chart_bottom - current_h
                    ctx.rgb(1.0, 0.2, 0.2)
                    ctx.rectangle(x, marker_y - 1, bar_w, 2).fill()

        # Title and max RPM label
        ctx.font_size = label_font_size
        if self._auto_done:
            ctx.move_to(-50, chart_top - 25).text("Complete")

            ctx.font_size = label_font_size - 8
            ctx.rgb(0.0, 1.0, 1.0).move_to(chart_left, chart_bottom + 5 + ctx.font_size).text("0%")
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
                left_power = self._auto_results[0][0]
                right_power = self._auto_results[n-1][0]
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
            progress = (self._auto_step * 100) // _AUTO_SCAN_STEPS
            ctx.rgb(1.0,1.0,1.0).move_to(-50, chart_top - 25).text(f"Scan {progress}%")

            # Instantaneous current label (updated live during the scan)
            ctx.font_size = label_font_size - 8
            for index, rpm in enumerate(self._rotation_rate_rpms):
                ctx.rgb(*self._colour_for_index(index)).move_to(chart_left+20, chart_bottom + 5 + ((index + 2) * (ctx.font_size))).text(f"Mtr{index+1}: {rpm}rpm")
            ctx.rgb(1.0, 0.2, 0.2).move_to(15, chart_bottom + 5 + ctx.font_size).text(f"{self._auto_last_current_ma}mA")

        # Y axis Maximum RPM and Current labels
        ctx.font_size = label_font_size - 8
        ctx.rgb(1.0, 1.0, 0.0).move_to(-15, chart_top - 5).text("Max")
        ctx.rgb(0.0, 1.0, 0.5).move_to(chart_left+10, chart_top - 5).text(f"rpm:{self._auto_max_rpm}")
        ctx.rgb(1.0, 0.2, 0.2).move_to(20, chart_top - 5).text(f"mA:{self._auto_max_current_ma}")

        #button_labels(ctx, cancel_label="Back", confirm_label="Manual")

    def _colour_for_index(self, index: int) -> tuple[float, float, float]:
        if index == 0:
            return (0.0, 1.0, 0.5)
        elif index == 1:
            return (1.0, 0.5, 0.0)
        else:
            return (1.0, 1.0, 1.0)





    def return_to_menu(self, menu_name: str | None = None):
        """Utility function to return to the main menu from any state. This is used when the user cancels out of a submenu or after acknowledging a warning message."""
        if self.logging:
            print("T:Returning to menu")
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
            print(f"T:Showing message: '{msg_content}' with type {msg_type}")
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
            print(f"T:Set Menu {menu_name}")
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
            print(f"T:Main Menu {item} at index {idx}")
        if item == MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_TEST]:   # Motor Test
            self.button_states.clear()
            if self._motor_test_start():
                self.set_menu(None)
                self.current_state = STATE_MOTOR_TEST
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
            print(f"T:Setting {item} @ {idx}")
        if idx == 0: #Save
            if self.logging:
                print("T:Settings Save All")
            platform_settings.save()
            self.notification = Notification("  Settings  Saved")
            self.set_menu()
        elif idx == 1: #Default
            if self.logging:
                print("T:Settings Default All")
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

    def _parse_version(self, version):
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
            print("T:Entered Settings editing mode")
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
                    print(f"T:Setting: {self.edit_setting} (+) Value: {self.edit_setting_value}")
                self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            if self.auto_repeat_check(delta, False):
                self.edit_setting_value = self.settings[self.edit_setting].dec(self.edit_setting_value, self.auto_repeat_level)
                if self._logging:
                    print(f"T:Setting: {self.edit_setting} (-) Value: {self.edit_setting_value}")
                self.refresh = True
        else:
            self.auto_repeat_clear()
            if self.button_states.get(BUTTON_TYPES["RIGHT"]) or self.button_states.get(BUTTON_TYPES["LEFT"]):
                self.button_states.clear()
                self.edit_setting_value = self.settings[self.edit_setting].d
                if self._logging:
                    print(f"T:Setting: {self.edit_setting} Default: {self.edit_setting_value}")
                self.refresh = True
                self.notification = Notification("Default")
            elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                if self._logging:
                    print(f"T:Setting: {self.edit_setting} Cancelled")
                self.return_to_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                if self._logging:
                    print(f"T:Setting: {self.edit_setting} = {self.edit_setting_value}")
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
        key = f"{SETTINGS_NAME_PREFIX}.{index}"
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

_PCNT_NUM_UNITS   = 4   # ESP32-S3 has 4 PCNT units

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
_PCNT_SIG_BASE    = 33

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
_REG_CONFIGURATION = 0x00      # Configuration register
_REG_SHUNT_VOLTAGE = 0x01      # Shunt voltage result (signed)
_REG_BUS_VOLTAGE = 0x02        # Bus voltage result (unsigned)
_REG_POWER = 0x03              # Power result (unsigned)
_REG_CURRENT = 0x04            # Current result (signed)
_REG_CALIBRATION = 0x05        # Calibration register
_REG_MASK_ENABLE = 0x06        # Alert mask/enable register
_REG_ALERT_LIMIT = 0x07        # Alert threshold register
_REG_MANUFACTURER_ID = 0xFE    # Manufacturer ID register
_REG_DIE_ID = 0xFF             # Die ID register


# Configuration register bits (0x00)
_CFG_RESET_BIT = 0x8000        # Software reset bit
_CFG_AVG_SHIFT = 9             # Averaging field shift (bits 11:9)
_CFG_VBUSCT_SHIFT = 6          # Bus voltage conversion time field shift (bits 8:6)
_CFG_VSHCT_SHIFT = 3           # Shunt voltage conversion time field shift (bits 5:3)
_CFG_MODE_SHIFT = 0            # Operating mode field shift (bits 2:0)

# AVG field values (bits 11:9)
_CFG_AVG_1 = 0b000             # 1 sample average
_CFG_AVG_4 = 0b001             # 4 sample average
_CFG_AVG_16 = 0b010            # 16 sample average
_CFG_AVG_64 = 0b011            # 64 sample average
_CFG_AVG_128 = 0b100           # 128 sample average
_CFG_AVG_256 = 0b101           # 256 sample average
_CFG_AVG_512 = 0b110           # 512 sample average
_CFG_AVG_1024 = 0b111          # 1024 sample average

# Conversion time field values for VBUSCT/VSHCT (bits 8:6 and 5:3)
_CFG_CT_140US = 0b000          # 140 us conversion time
_CFG_CT_204US = 0b001          # 204 us conversion time
_CFG_CT_332US = 0b010          # 332 us conversion time
_CFG_CT_588US = 0b011          # 588 us conversion time
_CFG_CT_1100US = 0b100         # 1.1 ms conversion time
_CFG_CT_2116US = 0b101         # 2.116 ms conversion time
_CFG_CT_4156US = 0b110         # 4.156 ms conversion time
_CFG_CT_8244US = 0b111         # 8.244 ms conversion time

# Operating mode field values (bits 2:0)
_CFG_MODE_POWER_DOWN = 0b000   # Power-down mode
_CFG_MODE_SHUNT_TRIG = 0b001   # Shunt voltage, triggered
_CFG_MODE_BUS_TRIG = 0b010     # Bus voltage, triggered
_CFG_MODE_SHUNT_BUS_TRIG = 0b011   # Shunt and bus, triggered
_CFG_MODE_ADC_OFF = 0b100      # ADC off (disabled)
_CFG_MODE_SHUNT_CONT = 0b101   # Shunt voltage, continuous
_CFG_MODE_BUS_CONT = 0b110     # Bus voltage, continuous
_CFG_MODE_SHUNT_BUS_CONT = 0b111   # Shunt and bus, continuous


# Mask/Enable register bits (0x06)
_MASK_SOL = 0x8000             # Shunt over-voltage alert flag
_MASK_SUL = 0x4000             # Shunt under-voltage alert flag
_MASK_BOL = 0x2000             # Bus over-voltage alert flag
_MASK_BUL = 0x1000             # Bus under-voltage alert flag
_MASK_POL = 0x0800             # Power over-limit alert flag
_MASK_CNVR = 0x0400            # Conversion ready alert flag
_MASK_AFF = 0x0010             # Alert function flag
_MASK_CVRF = 0x0008            # Conversion ready flag
_MASK_OVF = 0x0004             # Math overflow flag
_MASK_APOL = 0x0002            # Alert pin polarity select
_MASK_LEN = 0x0001             # Alert latch enable


# Device identification
_MANUFACTURER_ID_TI = 0x5449   # Texas Instruments manufacturer ID


# Driver configuration constants (100 mΩ shunt)
_SHUNT_RESISTOR_MILLIOHM = 100
_CALIBRATION_VALUE = 0x0200    # 512 => 0.1 mA current register LSB with 100 mΩ shunt
_CURRENT_LSB_UA = 100          # 0.1 mA current LSB in microamps
_POWER_LSB_UW = 2500           # 2.5 mW power LSB in microwatts
_READ_TIMEOUT_MS = 50

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






_LED_PIN = 2        # LED to illumiinate area under colour sensor to measure reflected light from surface below.
_COLOUR_INT_PIN = 1  # Not currently used, but we can set it up as an input for future interrupt-based drivers
_DIST_INT_PIN = 3  # Not currently used, but we can set it up as an input for future interrupt-based drivers

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
            print("T:SensorManager initialised")


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
                print(f"T:Cannot open I2C port {port}: {e}")
            return False

        try:
            found_addrs = set(self._i2c.scan())
        except Exception as e:      # pylint: disable=broad-exception-caught
            if self._logging:
                print(f"T:I2C scan failed on port {port}: {e}")
            return False

        if self._logging:
            print(f"T:Port {port} scan: {[hex(a) for a in found_addrs]}")

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
                        print(f"T:  + {cls.NAME} @ 0x{sensor.i2c_addr:02X} {cls.TYPE}")
                elif self._logging:
                    print(f"T:  - {cls.NAME} @ 0x{address:02X} begin() failed")

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
                print(f"T:LED On port {port} pin {config.ls_pin[_LED_PIN]} for colour sensor")
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
                    print(f"T:LED Off port {self._port}")
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




__app_export__ = HexTestApp
