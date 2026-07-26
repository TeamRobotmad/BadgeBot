""" Main Application File for BadgeBot."""
import asyncio
import sys
import time
from math import cos, pi

import ota
import settings as platform_settings
from app_components.notification import Notification
from app_components.tokens import button_labels, small_font_size, label_font_size, twentyfour_pt, clear_background
from app_components import Menu
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.hexpansion.util import get_slots_by_vid_pid, get_app_by_slot
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent,
                                     RequestStopAppEvent)
from system.notification.events import ShowNotificationEvent

from tildagonos import tildagonos
from machine import Pin
import app
import micropython

try:
    from micropython import const
except ImportError:
    # CPython / simulator fallback – const() is just an identity function
    # on MicroPython; replicate that so module-level const() calls work.
    const = lambda x: x         #pylint: disable=unnecessary-lambda-assignment

# If you could use hard=True in setting up a Pin IRQ hander, which you can't as of BadgeOS V1.10, then it is recommended to
# allocate the emergency exception buffer to prevent crashes due to OSError: Out of memory when an interrupt occurs and
# there is no memory available to handle the exception.
#import micropython
#micropython.alloc_emergency_exception_buf(100)

from .utils import draw_logo_animated, parse_version

HEXDRIVE_APP_VERSION = 6
HEXDRIVE2_APP_VERSION = 3

SETTINGS_NAME_PREFIX = "badgebot"  # Prefix for settings keys in EEPROM
APP_VERSION = "2.7" # BadgeBot App Version Number

# If you change the URL then you will need to regenerate the QR code
# using the generate_qr_code.py script, and update the _QR_CODE constant below with the new code generated for your URL
_QR_CODE = [
            0x1fcf67f,
            0x104cc41,
            0x174975d,
            0x1744e5d,
            0x175d45d,
            0x104ea41,
            0x1fd557f,
            0x001af00,
            0x04735f7,
            0x1070c97,
            0x1c23ae9,
            0x08ce9bd,
            0x1af3160,
            0x1270a80,
            0x1cc3549,
            0x097ef36,
            0x03ff5e9,
            0x1b18300,
            0x1b5a37f,
            0x0313b41,
            0x03f3d5d,
            0x078b65d,
            0x111e35d,
            0x0b57141,
            0x18bbd7f,
]

_BRIGHTNESS = const(1.0)

# Screen positioning constant for scroll mode display
H_START = const(-63)

# Timings/Settings
MOTOR_PWM_FREQ = const(20000)      # 20kHz is a good default for motors as it is above the audible range for most people and works with most motors and ESC

ACCELERATION_SCALE_FACTOR = const(512)  # Settings store motor power / acceleration divided by this; multiply back to get 0-65535 PWM values
MOTOR_POWER_SCALE_FACTOR = const(512)  # Settings store motor power / acceleration divided by this; multiply back to get 0-65535 PWM values

DEFAULT_ACCELERATION   = const(20000) // ACCELERATION_SCALE_FACTOR  # user-friendly acceleration value per 10ms Tick
DEFAULT_MAX_POWER      = const(55000) // MOTOR_POWER_SCALE_FACTOR   # exposed for use in other modules

_MIN_ACCELERATION      =  const(1024) // ACCELERATION_SCALE_FACTOR
_MIN_MAX_POWER         = const(10240) // MOTOR_POWER_SCALE_FACTOR

_MAX_ACCELERATION      = const(65535) // ACCELERATION_SCALE_FACTOR
_MAX_MAX_POWER         = const(65535) // MOTOR_POWER_SCALE_FACTOR

_LONG_PRESS_MS = const(750)        # Time for long button press to register, in ms
_RUN_COUNTDOWN_MS = const(5000)    # Time after running program until drive starts, in ms
_AUTO_REPEAT_MS = const(200)       # Time between auto-repeats, in ms
_AUTO_REPEAT_COUNT_THRES = const(10) # Number of auto-repeats before increasing level
_AUTO_REPEAT_SPEED_LEVEL_MAX = const(4)  # Maximum level of auto-repeat speed increases
_AUTO_REPEAT_LEVEL_MAX = const(3)  # Maximum level of auto-repeat digit increases
DEFAULT_BACKGROUND_UPDATE_PERIOD = const(50)       # mS when not moving
DEFAULT_ACTIVE_UPDATE_PERIOD     = const(10)       # mS when moving
_NOTIFICATION_DISPLAY_DURATION   = const(1000 * 3) # 3 seconds (hard coded in BadgeOS)

# App states
STATE_MENU = const(0)
STATE_MESSAGE = const(1)         # Message display
STATE_LOGO =  const(2)           # Logo display
STATE_COUNTDOWN = const(3)       # Shared countdown (Motor Moves & PID AutoTune)
STATE_SETTINGS = const(4)        # Edit Settings
STATE_MOTOR_MOVES = const(5)     # Motor Moves (sub-states managed by MotorMovesMgr)
STATE_SERVO = const(6)           # Servo test
STATE_FOLLOWER = const(7)        # Line Follower
STATE_SENSOR = const(8)          # Sensor Test
STATE_AUTODRIVE = const(9)       # Autonomous Drive
STATE_HEXPANSION = const(10)     # Hexpansion Management (sub-states managed by HexpansionMgr)
STATE_BLUETOOTH = const(11)      # Bluetooth Control (sub-states managed by BluetoothMgr)

# App states where user can minimise app (Menu, Message, Logo)
MINIMISE_VALID_STATES = [STATE_MENU, STATE_MESSAGE, STATE_LOGO]

# App states where BadgeBot directly controls the badge LEDs (Motor Moves, Countdown, Message, Logo, Line Follower, AutoTune, AutoDrive, Sensor Test)
_LED_CONTROL_STATES    = [STATE_MOTOR_MOVES, STATE_COUNTDOWN, STATE_MESSAGE, STATE_LOGO, STATE_FOLLOWER, STATE_AUTODRIVE, STATE_SENSOR]

#Misceallaneous Settings
_DEFAULT_LOGGING = False
_IS_SIMULATOR = sys.platform != "esp32"  # True when running in the simulator, not on real badge hardware
_DEFAULT_FWD_DIR = const(0)
_DEFAULT_FRONT_FACE = const(5)        # Front Face is Slot 3 on a standard build BadgeBot
_DEFAULT_MOTOR_DEADBAND = const(1)    # Minimum motor demand output value below which we don't try to move the motor. i.e. only if demand is above this do we apply the compensation below...
_DEFAULT_MOTOR_MIN = const((64 * 65536) // (512 * 100)) # Minimum motor PWM value (0-65535) for each motor, below which the motor will not move.  This is used to compensate for differences in motors and gearboxes, so that both motors start moving at the same time when given the same power level.
                                                # The figure of 64% of full power comes from measurement and analysis of 200 motors used for BadgeBot, which showed that 64% of full power was the minimum required to get 80% ofmotors moving reliably.
                                                # this can be customised through settings for each motor.

# Main Menu Items
MAIN_MENU_ITEMS = ["Bluetooth", "Line Follower", "Motor Moves", "Auto Drive", "Sensor Test", "Servo Test", "Hexpansions", "Settings", "About", "Exit"]
MENU_ITEM_BLUETOOTH = const(0)
MENU_ITEM_LINE_FOLLOWER = const(1)
MENU_ITEM_MOTOR_MOVES = const(2)
MENU_ITEM_AUTO_DRIVE = const(3)
MENU_ITEM_SENSOR_TEST = const(4)
MENU_ITEM_SERVO_TEST = const(5)
MENU_ITEM_HEXPANSION = const(6)
MENU_ITEM_SETTINGS = const(7)
MENU_ITEM_ABOUT = const(8)
MENU_ITEM_EXIT = const(9)

# Front face direction labels (0=BtnA corner between slots 6 & 1, each step = 30° CW)
_FRONT_FACE_LABELS = (
    "BtnA", "Slot 1", "BtnB", "Slot 2", "BtnC", "Slot 3",
    "BtnD", "Slot 4", "BtnE", "Slot 5", "BtnF", "Slot 6",
)
_MOTOR_DIRECTION_LABELS = ("Normal", "Reverse")

_FILE_DEST_LABELS = ("Badge FS", "Hex FS")

_MIN_BADGEOS_VERSION = (2, 2, 0)     # v2.2.0 is required to be able to use the new hexpansion utilite


# Import sub-modules after constants are defined so they can safely
# `from .app import STATE_*` without circular-import timing issues.
# Each module registers its own settings via init_settings()
# This is just a very robust way of doing from .module import XYZ which does
# not crash if anything in the module fails to import, and allows us to import
# individual classes from the modules without importing the whole module.
def _try_import(module_name, *attr_names):
    """Try importing named attributes from a sibling submodule.
    Returns a tuple of the requested attributes (or None for each on failure)."""
    nones = (None,) * len(attr_names)
    pkg_name = __name__.rsplit('.', 1)[0]
    full_name = pkg_name + '.' + module_name
    try:
        __import__(full_name)
        mod = sys.modules[full_name]
        return tuple(getattr(mod, n) for n in attr_names)
    except ImportError as e:
        print(f"Warning: {module_name} module not found ({e})")
    except Exception as e:                          # pylint: disable=broad-except
        print(f"Error importing {module_name} module ({e})")
    return nones

BluetoothMgr, _bluetooth_init_settings                    = _try_import('bluetooth_mgr', 'BluetoothMgr',  'init_settings')
HexpansionMgr, HexpansionType, _hexpansion_init_settings  = _try_import('hexpansion_mgr','HexpansionMgr', 'HexpansionType', 'init_settings')
SettingsMgr, MySetting                                    = _try_import('settings_mgr',  'SettingsMgr',   'MySetting')
MotorMovesMgr, _motor_moves_init_settings                 = _try_import('motor_moves',   'MotorMovesMgr', 'init_settings')
ServoTestMgr, _servo_test_init_settings                   = _try_import('servo_test',    'ServoTestMgr',  'init_settings')
LineFollowMgr, _line_follow_init_settings                 = _try_import('line_follow',   'LineFollowMgr', 'init_settings')
SensorTestMgr, _sensor_test_init_settings                 = _try_import('sensor_test',   'SensorTestMgr', 'init_settings')
AutoDriveMgr, _autodrive_init_settings                    = _try_import('autodrive',     'AutoDriveMgr',  'init_settings')
emit_diagnostics_output, set_diagnostics_output           = _try_import('diagnostics',   'output',        'set_output')


@micropython.viper
def _clamp(value: int, lo: int, hi: int) -> int:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value
class BadgeBotApp(app.App):         # pylint: disable=no-member
    """Main application class for BadgeBot.  Manages overall state, user input, and delegates to functional area managers for specific features."""
    __slots__ = (
        "_logging", "_ble_override_active", "button_states", "last_press", "_auto_repeat_intervals",
        "_auto_repeat", "_auto_repeat_count", "auto_repeat_level", "refresh", "_ring_refresh", "_ring_colour", "rpm",
        "animation_counter", "pattern_status", "qr_code", "app_version", "b_msg", "t_msg", "notification",
        "message",
        "message_colours",
        "message_type",
        "message_return_state",
        "current_menu",
        "menu",
        "_main_menu_position",
        "settings_menu_position",
        "_last_scroll",
        "scroll_mode_enabled",
        "scroll_ignore_next_c_button",
        "is_scroll",
        "scroll_offset",
        "run_countdown_elapsed_ms",
        "countdown_next_state",
        "_motor_deadband",
        "_motor1_reversed",
        "_motor2_reversed",
        "_motor1_min",
        "_motor2_min",
        "max_power",
        "acceleration",
        "_output1",
        "_output2",
        "_front_face",
        "current_state",
        "previous_state",
        "_update_period",
        "settings",
        "HEXPANSION_TYPES",
        "HEXDRIVE_HEXPANSION_INDEX",
        "HEXDRIVE_V2_HEXPANSION_INDEX",
        "HEXSENSE_HEXPANSION_INDEX",
        "HEXDIAG_HEXPANSION_INDEX",
        "HEXAUDIO_HEXPANSION_INDEX",
        "hexpansion_update_required",
        "hexdrive_hexpansion_types",
        "hexdrive_ports",
        "hexdrive_apps",
        "num_motors",
        "num_sensors",
        "num_line_sensors",
        "num_servos",
        "hexaudio_port",
        "hexsense_port",
        "hexdiag_port",
        "_diag_config",
        "motor_controller",
        "_hexpansion_mgr",
        "_motor_moves_mgr",
        "_servo_test_mgr",
        "_settings_mgr",
        "_line_follow_mgr",
        "_autotune_mgr",
        "_sensor_test_mgr",
        "_autodrive_mgr",
        "_state_update_dispatch",
        "_state_draw_dispatch",
        "_state_background_dispatch",
        "countdown_value",
        "_performance_mode",
        "_ble",
        "_notification_end_time",
    )

    DEFAULT_MAX_POWER = DEFAULT_MAX_POWER
    DEFAULT_ACCELERATION = DEFAULT_ACCELERATION

    def __init__(self):
        super().__init__()

        print("B:BadgeBotApp: Initialising...")
        self._logging = True

        # UI Button Controls
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self._auto_repeat_intervals = [ _AUTO_REPEAT_MS, _AUTO_REPEAT_MS//2, _AUTO_REPEAT_MS//4, _AUTO_REPEAT_MS//8, _AUTO_REPEAT_MS//16] # at the top end the loop is unlikley to cycle this fast
        self._auto_repeat: int = 0
        self._auto_repeat_count: int = 0
        self.auto_repeat_level: int = 0

        # UI Feature Controls
        self.refresh: bool = True            # True so that we draw initial screen on first loop, then set to True whenever we want to trigger a screen update
        self._ring_refresh: bool = False      # True when we want to force a refresh on the next loop, even if nothing has changed
        self._ring_colour: tuple[float, float, float] | None = None  # (r, g, b) each 0.0-1.0 while a ring is shown, or None for no ring
        self.rpm: int = 5                    # logo rotation speed in RPM
        self.animation_counter: int = 0
        self.pattern_status: bool = True     # Badge Controlled LED pattern: True = Pattern Enabled, False = Pattern Disabled
        self.qr_code = _QR_CODE
        self.app_version: str = APP_VERSION
        # strings shown on the Logo screen
        self.b_msg: str = f"BadgeBot V{self.app_version}"
        self.t_msg: str = "RobotMad"
        self.notification: Notification | None = None
        self.message: list = []
        self.message_colours: list = []
        self.message_type: str | None = None
        self.message_return_state: int | None = None
        self.current_menu: str | None = None
        self.menu: Menu | None = None
        self._main_menu_position: int = 0
        self.settings_menu_position: int = 0
        self._notification_end_time: int | None = None  # Time when a notification is scheduled to end, in milliseconds since boot. Used to determine if we need to redraw the screen frequently to show a notification.

        # Member data related to scrolling
        self._last_scroll : int = 0 # The last scroll posoition during non-scroll mode
        self.scroll_mode_enabled: bool = False  # Whether pressing the "C" button can toggle scroll mode on/off, which allows the user to scroll through lines on the display.
        self.scroll_ignore_next_c_button: bool = False # Used to ignore the "C" button event that triggers scroll mode on, otherwise it would immediately toggle scroll mode off again
        self.is_scroll: bool = False        # Whether we are in scroll mode - this is displayed by a green border around the screen
        self.scroll_offset: int = 0

        # UI countdown
        self.run_countdown_elapsed_ms: int = 0
        self.countdown_next_state: int | None = None  # which state to go to after countdown

        self._motor_deadband: int = _DEFAULT_MOTOR_DEADBAND  # Minimum motor PWM value below which we don't try to move the motor.
        self._motor1_reversed: bool = False         # 0 or 1 to control direction of motor 1, set based on settings
        self._motor2_reversed: bool = False         # 0 or 1 to control direction of motor 2, set based on settings
        self._motor1_min:  int = _DEFAULT_MOTOR_MIN * MOTOR_POWER_SCALE_FACTOR     # Minimum motor PWM value (0-65535) for motor 1, below which the motor will not move.  This is used to compensate for differences in motors and gearboxes, so that both motors start moving at the same time when given the same power level.
        self._motor2_min:  int = _DEFAULT_MOTOR_MIN * MOTOR_POWER_SCALE_FACTOR     # Minimum motor PWM value (0-65535) for motor 2, below which the motor will not move.  This is used to compensate for differences in motors and gearboxes, so that both motors start moving at the same time when given the same power level.
        self.max_power:    int = DEFAULT_MAX_POWER * MOTOR_POWER_SCALE_FACTOR      # Maximum motor PWM value (0-65535)
        self.acceleration: int = DEFAULT_ACCELERATION * ACCELERATION_SCALE_FACTOR  # Maximum change in motor output per update, used to limit acceleration and deceleration of the motors to prevent wheel slip and loss of control
        self._front_face:  int = _DEFAULT_FRONT_FACE  # Front Face is Slot 3 on a standard build BadgeBot, but can be changed in settings to any of the 12 possible directions (0-11) representing the forward direction for movement.
        self._output1:     int = 0                      # Current motor output for motor 1, after applying acceleration limits
        self._output2:     int = 0                      # Current motor output for motor 2, after applying acceleration limits

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_HEXPANSION
        self.previous_state = self.current_state
        self._update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD   # mS

        # Settings - common settings first, then each module registers its own later
        self.settings: dict = {}
        if MySetting is not None:
            # General settings
            self.settings['brightness']    = MySetting(self.settings, _BRIGHTNESS, 0.1, 1.0)
            self.settings['logging']       = MySetting(self.settings, _DEFAULT_LOGGING, False, True)
            #self.settings['path']         = MySetting(self.settings, 0, 0, len(_FILE_DEST_LABELS) - 1, labels=_FILE_DEST_LABELS)
            # Motor/Drive Direction settings
            self.settings['acceleration']  = MySetting(self.settings, DEFAULT_ACCELERATION,  _MIN_ACCELERATION,  _MAX_ACCELERATION)
            self.settings['max_power']     = MySetting(self.settings, DEFAULT_MAX_POWER, _MIN_MAX_POWER, _MAX_MAX_POWER)
            self.settings['mtr_deadband'] = MySetting(self.settings, _DEFAULT_MOTOR_DEADBAND, 0, 127)
            self.settings['mtr1_dir']    = MySetting(self.settings, _DEFAULT_FWD_DIR, 0, 1, labels=_MOTOR_DIRECTION_LABELS)
            self.settings['mtr2_dir']    = MySetting(self.settings, _DEFAULT_FWD_DIR, 0, 1, labels=_MOTOR_DIRECTION_LABELS)
            self.settings['mtr1_min']    = MySetting(self.settings, _DEFAULT_MOTOR_MIN, 0, 127)
            self.settings['mtr2_min']    = MySetting(self.settings, _DEFAULT_MOTOR_MIN, 0, 127)
            self.settings['front_face']    = MySetting(self.settings, _DEFAULT_FRONT_FACE, 0, 11, labels=_FRONT_FACE_LABELS)

            # Module-specific settings - only initialise modules which are NOT dependent on specific Hexpansion hardware here, as we want to be able to access settings in the HexpansionMgr before we have detected what hardware is present.  For Hexpansion-dependent modules, we will initialise their settings after we have scanned for hardware and know which modules we will be using.
            if _hexpansion_init_settings is not None:
                _hexpansion_init_settings(self.settings, MySetting)

            self.update_settings()
            self.fast_settings_update()

        if set_diagnostics_output is not None:
            set_diagnostics_output(self.diagnostics_output)

        # Hexpansion related - SEE ALSO hexpansion_mgr to update _SINGLE_PORT_HEXPANSION_REFS
        #                                       pid      name         vid          eeprom total size        eeprom page size      app mpy name                 app mpy version                       app name                motors    servos    sensors    sub_type
        assert HexpansionType is not None
        self.HEXPANSION_TYPES = [HexpansionType(0xCBCB, "HexDrive",                                                               app_mpy_name="hexdrive", app_mpy_version=HEXDRIVE_APP_VERSION, app_name="HexDriveApp", motors=2, servos=4, sub_type="Uncommitted" ),
                                 HexpansionType(0xCBCA, "HexDrive",                                                               app_mpy_name="hexdrive", app_mpy_version=HEXDRIVE_APP_VERSION, app_name="HexDriveApp", motors=2,           sub_type="2 Motor" ),
                                 HexpansionType(0xCBCC, "HexDrive",                                                               app_mpy_name="hexdrive", app_mpy_version=HEXDRIVE_APP_VERSION, app_name="HexDriveApp",           servos=4, sub_type="4 Servo" ),
                                 HexpansionType(0xCBCD, "HexDrive",                                                               app_mpy_name="hexdrive", app_mpy_version=HEXDRIVE_APP_VERSION, app_name="HexDriveApp", motors=1, servos=2, sub_type="1 Mot 2 Srvo" ),

                                 HexpansionType(0x10C8, "HexDrive2",   vid=0xCBCB, eeprom_total_size=32768, eeprom_page_size= 64, app_mpy_name="hexdrive2", app_mpy_version=HEXDRIVE2_APP_VERSION, app_name="HexDriveApp", motors=2, servos=2, sensors=2, sub_type="Uncommitted" ),
                                 HexpansionType(0x10C9, "HexDrive2",   vid=0xCBCB, eeprom_total_size=32768, eeprom_page_size= 64, app_mpy_name="hexdrive2", app_mpy_version=HEXDRIVE2_APP_VERSION, app_name="HexDriveApp",           servos=2, sensors=2, sub_type="2 Servo" ),
                                 HexpansionType(0x10CA, "HexDrive2",   vid=0xCBCB, eeprom_total_size=32768, eeprom_page_size= 64, app_mpy_name="hexdrive2", app_mpy_version=HEXDRIVE2_APP_VERSION, app_name="HexDriveApp", motors=2,           sensors=2, sub_type="2 Motor" ),
                                 HexpansionType(0x11CE, "HexDrive2",   vid=0xCBCB, eeprom_total_size=32768, eeprom_page_size= 64, app_mpy_name="hexdrive2", app_mpy_version=HEXDRIVE2_APP_VERSION, app_name="HexDriveApp", motors=1,           sensors=2, sub_type="Left Motor" ),
                                 HexpansionType(0x12CE, "HexDrive2",   vid=0xCBCB, eeprom_total_size=32768, eeprom_page_size= 64, app_mpy_name="hexdrive2", app_mpy_version=HEXDRIVE2_APP_VERSION, app_name="HexDriveApp", motors=1,           sensors=2, sub_type="Right Motor" ),
                                 HexpansionType(0x10CF, "HexDrive2",   vid=0xCBCB, eeprom_total_size=32768, eeprom_page_size= 64, app_mpy_name="hexdrive2", app_mpy_version=HEXDRIVE2_APP_VERSION, app_name="HexDriveApp", motors=1, servos=1, sensors=2, sub_type="1 Mot 1 Srvo" ),

                                 HexpansionType(0x2000, "HexSense",    vid=0xCBCB, eeprom_total_size=65536, eeprom_page_size=128,                                                                                                              sub_type="Line Follow" ),
                                 HexpansionType(0x4000, "HexDiag",     vid=0xCBCB, eeprom_total_size=65536, eeprom_page_size=128,                                                                                                              sub_type="Scope Pins" ),
                                 HexpansionType(0x5000, "HexAudio",    vid=0xCBCB, eeprom_total_size=8192,  eeprom_page_size= 32,                                                                                                              sub_type="Output Only" )]

        self.HEXDRIVE_HEXPANSION_INDEX = 0      # Index in the HEXPANSION_TYPES list which corresponds to the basic HexDrive type
        self.HEXDRIVE_V2_HEXPANSION_INDEX = 4   # Index in the HEXPANSION_TYPES list which corresponds to the basic HexDrive2 type
        self.HEXSENSE_HEXPANSION_INDEX = 10     # Index in the HEXPANSION_TYPES list which corresponds to the HexSense type
        self.HEXDIAG_HEXPANSION_INDEX = 11      # Index in the HEXPANSION_TYPES list which corresponds to the HexDiag type
        self.HEXAUDIO_HEXPANSION_INDEX = 12     # Index in the HEXPANSION_TYPES list which corresponds to the HexAudio type

        self.hexpansion_update_required: bool = False # flag from async to main loop

        self.hexdrive_hexpansion_types = [0,1,2,3,4,5,6,7,8,9] # indices in the HEXPANSION_TYPES list which correspond to HexDrive variants - used to check if a detected hexpansion is a HexDrive and to set up the motor and servo counts accordingly

        # HexDrive hexpansion - has an app which we use to control the motors and servos
        self.hexdrive_ports = []
        self.hexdrive_apps = []

        # Motor Driver Hardware
        self.num_motors: int = 0        # initialised to 0 until we detect a HexDrive Hexpansion and can set this based on the actual number of motors it has

        # Sensor Hardware
        self.num_sensors: int = 0       # initialised to 0 until we detect some Sensors

        # Line Sensors Hardware
        self.num_line_sensors: int = 0  # initialised to 0 until we detect a HexSense Hexpansion and can set this based on the actual number of sensors it has

        # Servo Hardware
        self.num_servos: int = 0        # initialised to 0 until we detect a HexDrive Hexpansion and can set this based on the actual number of servos it has


        slots = get_slots_by_vid_pid(0xCBCB, 0x10C8)    # shortcut to initialise HexDrive2 as provided at EMF Camp 2026 BadgeBot Workshop
        if len(slots) > 0:
            self.hexdrive_ports = slots
            _app = get_app_by_slot(slots[0])
            if _app is not None:
                print(f"B:HexDrive2 (with App) found in slot {slots[0]}")
                self.hexdrive_apps.append(_app)
            self._calc_num_motors_servos_sensors()
            if self.logging:
                print(f"B:Num motors={self.num_motors}, servos={self.num_servos}, sensors={self.num_sensors}")

        # HexAudio hexpansion
        self.hexaudio_port  = None            # Store the HexpansionConfig of the HexAudio that is providing the audio output

        # HexSense hexpansion - prototype line sensor expansion
        self.hexsense_port = None

        # Diagnostics hexpansion
        self.hexdiag_port = None
        self._diag_config = None
        self.hexdiag_setup()

        # High-level motor controller (created when HexDrive is found)
        self.motor_controller = None

        # Functional area managers
        self._bluetooth_mgr    = BluetoothMgr(self, logging=self.logging)   if BluetoothMgr is not None else None
        self._hexpansion_mgr   = HexpansionMgr(self, logging=self.logging)  if HexpansionMgr is not None else None
        self._motor_moves_mgr  = MotorMovesMgr(self, logging=self.logging)  if MotorMovesMgr is not None else None
        self._servo_test_mgr   = ServoTestMgr(self, logging=self.logging)   if ServoTestMgr is not None else None
        self._settings_mgr     = SettingsMgr(self, logging=self.logging)    if SettingsMgr is not None else None
        self._line_follow_mgr  = LineFollowMgr(self, logging=self.logging)  if LineFollowMgr is not None else None
        self._sensor_test_mgr  = SensorTestMgr(self, logging=self.logging)  if SensorTestMgr is not None else None
        # Auto Tune and Auto Drive are not available.
        self._autodrive_mgr    = None # AutoDriveMgr(self, logging=self.logging)   if AutoDriveMgr is not None else None
        self._autotune_mgr     = None # AutotuneMgr(self, self._line_follow_mgr, logging=self.logging) if AutotuneMgr is not None else None

        # State -> manager dispatch tables (only include managers that exist)
        self._state_update_dispatch = {}
        self._state_draw_dispatch = {}
        self._state_background_dispatch = {}

        self._register_state_functions(STATE_BLUETOOTH, self._bluetooth_mgr)
        self._register_state_functions(STATE_HEXPANSION, self._hexpansion_mgr)
        self._register_state_functions(STATE_MOTOR_MOVES, self._motor_moves_mgr)
        self._register_state_functions(STATE_FOLLOWER, self._line_follow_mgr)
        self._register_state_functions(STATE_SERVO, self._servo_test_mgr)
        self._register_state_functions(STATE_SETTINGS, self._settings_mgr)
        self._register_state_functions(STATE_SENSOR, self._sensor_test_mgr)
        self._register_state_functions(STATE_AUTODRIVE, self._autodrive_mgr)

        # Countdown timer value
        self.countdown_value: int = 0

        # Performance mode flag - when True we will try to run the app as fast as possible
        self._performance_mode: int = 0     # 0 = normal, 1= performance mode requested, 2 = performance mode active

        # Bluetooth LE
        self._ble_override_active: bool = False

        # Hexpansion event handlers registered directly by hexpansion_mgr
        if self._hexpansion_mgr is not None:
            self._hexpansion_mgr.register_events()

        # Event handlers for gaining and losing focus and being aware of 3rd party Notifications
        eventbus.on_async(RequestForegroundPushEvent, self._gain_focus,      self)
        eventbus.on_async(RequestForegroundPopEvent,  self._lose_focus,      self)
        eventbus.on_async(RequestStopAppEvent,        self._handle_stop_app, self)

        # We start with focus on launch, without an event emmited
        # This version is compatible with the simulator
        asyncio.get_event_loop().create_task(self._gain_focus(RequestForegroundPushEvent(self)))



        # Check what version of the Badge s/w we are running on
        ver: list[int | str] | None = None
        try:
            ver = parse_version(ota.get_version())
            if ver is not None:
                if self.logging:
                    print(f"B:BadgeSW V{ver}")
                version_triplet = tuple(part if isinstance(part, int) else 0 for part in (ver[:3] if ver is not None else []))
                if len(version_triplet) == 3 and version_triplet >= _MIN_BADGEOS_VERSION:
                    # Potential to do things differently based on badge s/w version
                    pass
                else:
                    print(f"B:BadgeSW V{ver} is too old (requires V{_MIN_BADGEOS_VERSION})")
                    self.show_message(["BadgeBot:", "Please", "Upgrade", "BadgeOS"], [(0.5,1.0,0.5),(1,1,1),(1,1,1),(1,1,1)], "reboop")
        except Exception as e: # pylint: disable=broad-exception-caught
            print(f"B:Ver check failed {e}!")

        if self.logging:
            print(f"B:BadgeBot App V{self.app_version} Initialised")


    def _calc_num_motors_servos_sensors(self):
        """Calculate the total number of motors, servos, and sensors based on the detected HexDrive hexpansion types."""
        self.num_motors = 0
        self.num_servos = 0
        self.num_sensors = 0
        for _ in self.hexdrive_ports:
            hexdrive_type_idx = self.HEXDRIVE_V2_HEXPANSION_INDEX # don't force this type
            # when BLE is made a sub-app we won't need to pre-empt hexpansion_mgr and can wait for it to detect the hexpansion types...
            if hexdrive_type_idx is not None and 0 <= hexdrive_type_idx < len(self.HEXPANSION_TYPES):
                self.num_motors   += self.HEXPANSION_TYPES[hexdrive_type_idx].motors
                self.num_servos   += self.HEXPANSION_TYPES[hexdrive_type_idx].servos
                self.num_sensors  += self.HEXPANSION_TYPES[hexdrive_type_idx].sensors


    def _register_state_functions(self, state: int, manager: object | None):
        """Register the update, draw, and background update functions for each state in the dispatch tables."""
        if manager is None:
            return
        update_fn = getattr(manager, "update", None)
        draw_fn = getattr(manager, "draw", None)
        background_fn = getattr(manager, "background_update", None)
        if callable(update_fn):
            self._state_update_dispatch[state] = update_fn
        if callable(draw_fn):
            self._state_draw_dispatch[state] = draw_fn
        if callable(background_fn):
            self._state_background_dispatch[state] = background_fn


    @property
    def logging(self):
        """Convenience property to access logging setting."""
        return self._logging


    @property
    def performance_mode(self) -> bool:
        """Convenience property to access performance_mode setting."""
        return self._performance_mode != 0

    @performance_mode.setter
    def performance_mode(self, value: bool):
        """Convenience property to set performance_mode setting."""
        self._performance_mode = 1 if value else 0


    @property
    def sensor_test_mgr(self):
        """Public access to the SensorTestMgr, used by LineFollowMgr & AutoDriveMgr to share the sensor manager."""
        return self._sensor_test_mgr


    @property
    def update_period(self):
        """Convenience property to access update_period setting."""
        return self._update_period

    @update_period.setter
    def update_period(self, value: int):
        """Convenience property to set update_period setting."""
        # if we have an active Bluetooth Connection then we need to maintain a high update rate so that the motor acceleration is correct
        if self._bluetooth_mgr is not None and self._bluetooth_mgr.is_connected:
            value = min(value, DEFAULT_ACTIVE_UPDATE_PERIOD)  # ensure we don't go below the minimum update period when Bluetooth is active
        if self._logging:
            print(f"B:Setting update_period to {value} ms")
        self._update_period = value


    ### ASYNC EVENT HANDLERS ###

    async def _handle_stop_app(self, event: RequestStopAppEvent):
        """ Handle the RequestStopAppEvent so that we can release resources """
        if event.app == self:
            if self.logging:
                print("B:BadgeBot received RequestStopAppEvent, save settings & releasing resources")
                # Save settings before we exit, so that any changes made during this session are preserved
                platform_settings.save()
            if self.pattern_status:
                eventbus.emit(PatternEnable())
                self.pattern_status = True
            if self._hexpansion_mgr is not None:
                self._hexpansion_mgr.unregister_events()
            if self.scroll_mode_enabled:
                eventbus.remove(ButtonUpEvent, self._handle_button_up, self)
            eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
            eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
            eventbus.remove(RequestStopAppEvent, self._handle_stop_app, self)


    async def _gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            if self.logging:
                print(f"B:BadgeBot gained focus in state {self.current_state}")
            if self.current_state in _LED_CONTROL_STATES:
                eventbus.emit(PatternDisable())
                self.pattern_status = False
            if self.scroll_mode_enabled:
                eventbus.on_async(ButtonUpEvent, self._handle_button_up, self)
            eventbus.on_async(ShowNotificationEvent, self._handle_notification, self)


    async def _lose_focus(self, event: RequestForegroundPopEvent):
        if event.app is self:
            if self.logging:
                print(f"B:BadgeBot lost focus from state {self.current_state}")
            if not self.pattern_status:
                eventbus.emit(PatternEnable())
                self.pattern_status = True
            if self.scroll_mode_enabled:
                eventbus.remove(ButtonUpEvent, self._handle_button_up, self)
            eventbus.remove(ShowNotificationEvent, self._handle_notification, self)


    async def _handle_button_up(self, event: ButtonUpEvent):
        if self.scroll_mode_enabled and event.button == BUTTONS["C"]:
            if self.scroll_ignore_next_c_button:
                self.scroll_ignore_next_c_button = False
                return
            # Toggle scroll mode on/off when "C" button is released
            self.scroll(not self.is_scroll)


    async def _handle_notification(self, event: ShowNotificationEvent):
        """Handle a ShowNotificationEvent by setting the end time for displaying it in ticks from now."""
        self._notification_end_time = _NOTIFICATION_DISPLAY_DURATION
        if self._logging:
            print(f"B:Received ShowNotificationEvent: '{event.message}' (port={event.port})")


    async def background_task(self):
        """Background task loop for handling time-based updates. This runs independently of the main update/draw loop
           and is suitable for tasks that need to run at a consistent interval regardless of the current state or drawing performance."""
        last_time = time.ticks_ms()

        while True:
            cur_time = time.ticks_ms()
            delta_ticks = time.ticks_diff(cur_time, last_time)
            diagnostics_output(0, 1)
            self.background_update(delta_ticks)
            diagnostics_output(0, 0)
            await asyncio.sleep_ms(max (1, self._update_period - (time.ticks_ms() - cur_time)))  # sleep for the remainder of the update period, accounting for time taken by background_update
            last_time = cur_time


    ### NON-ASYNC FUNCTIONS ###

    def background_update(self, delta: int):
        """Background update function that is called at a regular interval from the background task loop.
           It dispatches to the appropriate manager based on the current state, and if motor outputs are returned, it sends them to the HexDrive app."""
        bg_fn = self._state_background_dispatch.get(self.current_state)
        output = bg_fn(delta) if bg_fn is not None else None

        if len(self.hexdrive_apps) > 0 and self._bluetooth_mgr:
            # BLE direction buttons override the state's motor output while held,
            # regardless of whether the current state produced any output.
            ble_override = self._bluetooth_mgr.motor_override(self.max_power)
            if ble_override is not None:
                self._ble_override_active = True
                output = ble_override
            else:
                if self._ble_override_active and output is None:
                    # ensure we stop the motors if we were previously overriding them with BLE and now there is no output from the current state
                    output = (0, 0)
                self._ble_override_active = False

            if output is None and (self._output1 != 0 or self._output2 != 0):
                # ensure we stop the motors if the current state has no output and the previous output was non-zero
                output = (0, 0)

            if output is not None:
                # we have to continue to run the motors until the current state returns None and the outputs are both 0, otherwise the motors will keep running at the last output value
                if not self.hexdrive_apps[0].set_motors(self.apply_motor_calibration(output)):
                    if self.logging:
                        print("Failed to set motor outputs to HexDrive app")


    # Helper properties to determine whether specific features are enabled based on detected hardware and available managers.

    @property
    def enable_motor_moves(self):
        """Whether the Motor Moves feature is enabled, based on whether we have detected motor hardware and have the manager available."""
        if self.num_motors > 1 and self._motor_moves_mgr is not None:
            return True
        else:
            if self.logging:
                print(f"Motor Moves not enabled: num_motors={self.num_motors}")


    @property
    def enable_servo_test(self):
        """Whether the Servo Test feature is enabled, based on whether we have detected servo hardware and have the manager available."""
        return self.num_servos > 0 and self._servo_test_mgr is not None


    @property
    def enable_bluetooth(self):
        """Whether Bluetooth is enabled, based on whether we have BLE controller available and a hexpansion from which to get a unique id."""
        return self._bluetooth_mgr is not None and self._hexpansion_mgr is not None


    @property
    def enable_line_follow(self):
        """Whether the Line Follow feature is enabled.  Requires two motors, the line-follow
        manager, and a colour sensor (detected and driven via the sensor test manager)."""
        return (self.num_motors > 1
                and self._line_follow_mgr is not None
                and self._sensor_test_mgr is not None
                and self._sensor_test_mgr.colour_sensor_present())


    @property
    def enable_autotune(self):
        """Whether the Autotune feature is enabled.  Requires two motors, the line-follow manager, and a colour sensor (detected and driven via the sensor test manager)."""
        return (self.num_motors > 1
                and self._line_follow_mgr is not None
                and self._sensor_test_mgr is not None
                and self._sensor_test_mgr.colour_sensor_present()
                and self._autotune_mgr is not None)


    @property
    def enable_sensor_test(self):
        """Whether the Sensor Test feature is enabled, based on whether we have detected sensor hardware and have the manager available."""
        #print(f"Checking if Sensor Test is enabled: sensor_test_mgr={'present' if self._sensor_test_mgr is not None else 'absent'}")
        return self.num_sensors > 0 and self._sensor_test_mgr is not None


    @property
    def enable_autodrive(self):
        """Whether the Autodrive feature is enabled, based on whether we have detected motor hardware and have the manager available."""
        return self.num_motors > 1 and self._autodrive_mgr is not None


    @property
    def enable_hexpansion_mgr(self):
        """Whether the Hexpansion Manager is enabled, based on whether the manager is available.  Note that this does not necessarily mean that you have hexpansion hardware, as the manager can be enabled and used for managing settings related to hexpansions even if no hexpansion hardware is detected."""
        return False


    def initialise_settings(self):
        """Initialise settings with default values and register them in the app's settings dictionary."""
        if MySetting is None:
            return  # Settings system not available, skip initialisation
        # Module-specific settings
        if self.enable_bluetooth and _bluetooth_init_settings is not None:
            _bluetooth_init_settings(self.settings, MySetting)
        if self.enable_motor_moves and _motor_moves_init_settings is not None:
            _motor_moves_init_settings(self.settings, MySetting)
        if self.enable_servo_test and _servo_test_init_settings is not None:
            _servo_test_init_settings(self.settings, MySetting)
        if self.enable_line_follow and _line_follow_init_settings is not None:
            _line_follow_init_settings(self.settings, MySetting)
        if self.enable_sensor_test and _sensor_test_init_settings is not None:
            _sensor_test_init_settings(self.settings, MySetting)
        if self.enable_autodrive and _autodrive_init_settings is not None:
            _autodrive_init_settings(self.settings, MySetting)
        # The below functions get called during return_to_menu anyway
        #self.update_settings()  # Load settings from EEPROM after initialisation
        #self.fast_settings_update()  # Update fast access settings


    def update_settings(self):
        """Update settings from EEPROM."""
        if self.logging:
            print("B:Updating settings from EEPROM")
        for s, setting in self.settings.items():
            setting.v = platform_settings.get(f"{SETTINGS_NAME_PREFIX}.{s}", setting.d)
            # check settings against min/max values and adjust if necessary - in case min/max values have changed since the setting was last saved
            setting.clamp()
            if self.logging:
                print(f"B:Setting {s} = {setting.v}")


    def fast_settings_update(self):
        """Update fast access settings from the main settings dictionary."""
        if self._logging:
            print("B:Updating fast access settings")
        self._logging: bool = self.settings['logging'].v
        self._front_face: int = self.settings['front_face'].v
        self._motor_deadband: int = self.settings['mtr_deadband'].v * MOTOR_POWER_SCALE_FACTOR
        self._motor1_reversed: bool = self.settings['mtr1_dir'].v != 0
        self._motor2_reversed: bool = self.settings['mtr2_dir'].v != 0
        self._motor1_min: int = self.settings['mtr1_min'].v * MOTOR_POWER_SCALE_FACTOR
        self._motor2_min: int = self.settings['mtr2_min'].v * MOTOR_POWER_SCALE_FACTOR
        self.max_power: int = self.settings['max_power'].v * MOTOR_POWER_SCALE_FACTOR
        self.acceleration: int = self.settings['acceleration'].v * ACCELERATION_SCALE_FACTOR


    def hexdiag_setup(self):
        """ Use HS pins on a spare Hexpansion to make diagnostic timing measurements"""
        if self._diag_config is not None and self.hexdiag_port != self._diag_config.port:
            for i in range(4):
                self._diag_config.pin[i].init(mode=Pin.IN)
            self._diag_config = None
        if self.hexdiag_port is not None and self._diag_config is None:
            print(f"B:HexDiag on port {self.hexdiag_port}")
            self._diag_config = HexpansionConfig(self.hexdiag_port)
            for i in range(4):
                self._diag_config.pin[i].init(mode=Pin.OUT)


    def diagnostics_output(self, index: int, value: int):
        """Output diagnostic values to the HS pins on the diagnostics hexpansion, for measurement with an oscilloscope"""
        if self._diag_config is not None and 0 <= index < 4:
            self._diag_config.pin[index].value(value)


    def _pattern_management(self):
        if self.current_state in _LED_CONTROL_STATES:
            if self.pattern_status:
                eventbus.emit(PatternDisable())
                self.pattern_status = False
                # delay enough to allow the pattern to stop
                time.sleep_ms(500)
        elif self.current_state not in _LED_CONTROL_STATES and not self.pattern_status:
            eventbus.emit(PatternEnable())
            self.pattern_status = True


    ### MAIN APP CONTROL FUNCTIONS ###

    def update(self, delta: int):
        """Main update function called from the main loop. Handles state transitions, user input, and delegates to functional area managers."""
        diagnostics_output(1, 1)

        if self.notification:
            self.notification.update(delta)
            try:
                # in case access to protected member _open() (or _is_closed()) is not allowed, we catch the exception and
                # to prevent crashes - this means that in this case we won't be able to automatically clear
                # notifications when they are closed, but at least the app won't crash.
                if not self.notification._open:  # pylint: disable=protected-access
                    if self._logging:
                        print("B:Notification closed, clearing notification reference")
                    self.notification = None
            except Exception as e:  # pylint: disable=broad-exception-caught
                print(f"B:Error: checking notification status: {e}")
            self.refresh = True  # Ensure we refresh the display while a notification is active

        # if a 3rd party notification is active, we need to refresh the display more frequently to ensure that the notification is visible and updated.
        if self._notification_end_time is not None:
            self._notification_end_time -= delta
            if self._notification_end_time <= 0:
                if self._logging:
                    print("B:Notification expired")
                self._notification_end_time = None
            else:
                if self._logging:
                    print(f"B:Notification active (remaining {self._notification_end_time} ms)")
                self.refresh = True

        # manage LED PatternEnable/Disable for all states
        #self._pattern_management()

        # Update Hexpansion management if something 'hexpansion' related has changed
        if self.hexpansion_update_required:
            if self.current_state != STATE_HEXPANSION and self._hexpansion_mgr is not None:
                # Trigger an update cycle for hexpansion_mgr even though it is not currently active
                self._hexpansion_mgr.update(delta)

        # Update the main application state (menus, countdowns, and delegating to functional area managers)
        self._update_main_application(delta)

        if self.current_state != self.previous_state:
            if self.logging:
                print(f"B:State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # manage LED PatternEnable/Disable for all states
            self._pattern_management()
            # something has changed - so worth redrawing
            self.refresh = True

        if self.current_state in _LED_CONTROL_STATES:
            if self.current_state in [STATE_FOLLOWER]:
                # For Line Follower, set LEDs based on the line sensor readings
                # could be optimised to only update LEDs when sensor readings change, rather than every update cycle
                # nothing while we try to optimise the sensor reading rate
                pass
            else:
                if self.settings['brightness'].v < 1.0:
                    # Scale brightness
                    for i in range(1,13):
                        colour = tildagonos.leds[i]
                        tildagonos.leds[i] = (
                            int(colour[0] * self.settings['brightness'].v),
                            int(colour[1] * self.settings['brightness'].v),
                            int(colour[2] * self.settings['brightness'].v),
                        )
                try:
                    # saw this crash randomly - hence protected by try/except to prevent whole app crashing, and added logging to investigate further
                    tildagonos.leds.write()
                except OSError as e:
                    print(f"Error writing to LEDs: {e}")
        diagnostics_output(1, 0)



    def _update_main_application(self, delta: int) -> None:
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
                    if self._logging:
                        print("Menu is animating")
                    self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            if self.current_state == STATE_MESSAGE and self.message_type == None:
                # If we are in the menu, we want to return to the previous state, not minimise the app
                self.return_to_menu()
                return
            else:
                self.button_states.clear()
                platform_settings.save()  # Save settings before minimising
                self.minimise()

        ### Shared Countdown Display ###
        elif self.current_state == STATE_COUNTDOWN:
            self._update_state_countdown(delta)

        ## Shared Warning and Message Display (for Hexpansion issues and general messages) ###
        elif self.current_state in [STATE_MESSAGE, STATE_LOGO]:
            self._update_state_message(delta)

        ### Delegate to functional area managers via dispatch table ###
        else:
            # Handle scroll mode input for any state where it is enabled, before delegating to the state-specific update function
            if self.scroll_mode_enabled and self.is_scroll:
                if self.button_states.get(BUTTON_TYPES["DOWN"]):
                    self.button_states.clear()
                    self.scroll_offset -= 1
                    self.refresh = True
                elif self.button_states.get(BUTTON_TYPES["UP"]):
                    self.button_states.clear()
                    self.scroll_offset += 1
                    self.refresh = True
            if self.current_state in self._state_update_dispatch:
                update_fn = self._state_update_dispatch.get(self.current_state)
                if update_fn is not None:
                    update_fn(delta)
        ### End of Update ###


    def _update_state_message(self, delta: int):      # pylint: disable=unused-argument
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            if self.message_type == "reboop":
                self.button_states.clear()
                # Reboot has been acknowledged by the user - unfortunately we can't actually reboot the badge from Python.
                return # leave the message on screen.
            elif self.message_return_state is not None:
                self.button_states.clear()
                self.current_state = self.message_return_state
            else:
                # Message has been acknowledged by the user - allow access to the menu
                self.button_states.clear()
                # refresh the menu in case available options have changed
                self.set_menu()
                self.refresh = True
                self.current_state = STATE_MENU
            self.message = []
            self.message_colours = []
            self.message_type = None
            self.message_return_state = None
        else:
            # "CANCEL" button is handled in common for all MINIMISE_VALID_STATES so no custom code here
            # Show the warning screen for 10 seconds
            self.animation_counter += delta
            if self.message_type == "warning" and self.animation_counter > 10000:
                # For Warnings, after 10 seconds show the logo
                self.animation_counter = 0
                self.current_state = STATE_LOGO
                self.message = []
                self.message_colours = []
                self.message_type = None
                self.message_return_state = None
                self.refresh = True
            elif self.current_state == STATE_LOGO:
                # LED management - to match rotating logo:
                for i in range(1,13):
                    colour = (255, 241, 0)      # custom Robotmad shade of yellow
                    # raised cosine cubed wave
                    wave = self.settings['brightness'].v * pow((1.0 + cos(((i) *  pi / 1.5) - (self.rpm * self.animation_counter * pi / 7500)))/2.0, 3)
                    # 4 sides each projecting a pattern of 3 LEDs (12 LEDs in total)
                    tildagonos.leds[i] = (
                        int(wave * colour[0]),
                        int(wave * colour[1]),
                        int(wave * colour[2]),
                    )
                self.refresh = True
            else:
                for i in range(1,13):
                    tildagonos.leds[i] = (255,0,0) if self.message_type == "error" else (0,255,0)


    def _update_state_countdown(self, delta: int):
        self.clear_leds()
        self.run_countdown_elapsed_ms += delta
        if self.run_countdown_elapsed_ms >= _RUN_COUNTDOWN_MS:
            if self.countdown_next_state == STATE_MOTOR_MOVES:
                # Motor Moves: delegate to begin_moves
                self.current_state = self.countdown_next_state
                if self._motor_moves_mgr is not None:
                    self._motor_moves_mgr.begin_moves()
                else:
                    self.return_to_menu()
            else:
                # Generic fallback
                self.return_to_menu()
        else:
            # Countdown is still running - update display
            countdown_value = 1 + ((_RUN_COUNTDOWN_MS - self.run_countdown_elapsed_ms) // 1000)
            if self.countdown_value != countdown_value:
                self.countdown_value = countdown_value
                self.refresh = True


    def scroll_mode_enable(self, enable: bool):
        """Enable the potential for scroll mode to be toggled on and off by pressing the "C" button"""
        if enable:
            self.scroll_mode_enabled = True
            self.scroll_ignore_next_c_button = True # we want to ignore the "C" button event that triggered this, otherwise it would immediately toggle scroll mode on
            eventbus.on_async(ButtonUpEvent, self._handle_button_up, self)
        else:
            self.scroll_mode_enabled = False
            eventbus.remove(ButtonUpEvent, self._handle_button_up, self)


    def scroll(self, enable: bool):
        """Enable or disable scroll mode, which allows the user to scroll the display up and downto see hidden content. This is indicated by a green border around the screen."""
        self.is_scroll = enable
        if self.scroll_mode_enabled:
            if enable:
                self._last_scroll = self.scroll_offset
            else:
                self.scroll_offset = self._last_scroll
            # only show notification about scroll mode if the feature is enabled, otherwise it would be confusing to show a notification about a feature that can't be used
            state = "enabled" if enable else "disabled"
            self.notification = Notification(f"Scroll {state}")

    def set_ring_colour(self, colour: tuple[float, float, float] | None = None):
        """Set the colour of the ring drawn around the edge of the display.
           Pass an (r, g, b) tuple (each 0.0-1.0) to show a coloured ring, or None to stop showing the ring (the default).
           Setting a colour flags a ring refresh so the ring is rendered on the next draw regardless of whether a full display refresh is required."""
        self._ring_colour = colour
        self._ring_refresh = True


    def draw(self, ctx):
        """Main draw function called from the main loop. Handles drawing the current state, including any notifications."""
        if 2 == self._performance_mode:
            # drawing the screen takes a VERY long time - so when trying to run robot control algorithms as fast as possible we skip drawing the screen to avoid stalling the background updates
            return

        # diagnostics output for measuring draw time on a scope - pin 2 is high while draw() is running, low when it is finished
        diagnostics_output(2, 1)

        if 1 == self._performance_mode or self.refresh:
            # Clear the Screen
            clear_background(ctx)

        if 1 == self._performance_mode:
            # Now the Screen is cleared, we can switch to performance mode, which will skip drawing the screen in future frames until a refresh is required.
            self._performance_mode = 2
            diagnostics_output(2, 0)
            return

        if self.current_state == STATE_MENU and self.menu is not None:
            # These need to be drawn every frame as they contain animations
            self.menu.draw(ctx)
        else:
            if self._ring_refresh or self.refresh:
                if self._ring_colour is not None:
                    self._ring_refresh = False
                    # The ring can be updated without redrawing the entire display
                    # Draw an 8-pixel colour ring around the edge of the display
                    ctx.line_width = 8
                    ctx.rgb(*self._ring_colour).arc(0, 0, 116, 0, pi * 2, 0).stroke()

            if self.refresh:
                self.refresh = False

                #ctx.save()
                #if in a mode where rotated display is desirable:
                #    ctx.rotate(self.front_face * 2.0 * pi / _FRONT_FACE_NUM_ORIENTATIONS)  # Rotate the entire display based on the front_face setting, so that "forward" is always at the top of the display regardless of how the badge is oriented
                ctx.font_size = label_font_size
                if ctx.text_align != ctx.LEFT:
                    # See https://github.com/emfcamp/badge-2024-software/issues/181
                    ctx.text_align = ctx.LEFT
                ctx.text_baseline = ctx.BOTTOM

                if self.current_state == STATE_LOGO:
                    draw_logo_animated(ctx, self.rpm, self.animation_counter, [self.b_msg, self.t_msg], self.qr_code)
                elif self.scroll_mode_enabled and self.is_scroll:
                    # Scroll mode indicator border
                    ctx.rgb(0,0.2,0).rectangle(     -120,-120, 115+H_START,240).fill()
                    ctx.rgb(0,0  ,0).rectangle(H_START-5,-120,10-2*H_START,240).fill()
                    ctx.rgb(0,0.2,0).rectangle(5-H_START,-120, 115+H_START,240).fill()
                #else:
                #    ctx.rgb(0,0,0).rectangle(-120,-120,240,240).fill()

                # Common states for messages and errors, which can be triggered by any functional area manager and are displayed in a consistent way
                if self.current_state == STATE_MESSAGE:
                    if self.message_colours == []:
                        self.message_colours = [(1,0,0)]*len(self.message)
                    self.draw_message(ctx, self.message, self.message_colours, label_font_size if len(self.message) <= 5 else small_font_size)
                    if self.message_type is None or self.message_type == "warning":
                        button_labels(ctx, confirm_label="OK", cancel_label="Exit")
                elif self.current_state == STATE_COUNTDOWN:
                    self.draw_message(ctx, [str(self.countdown_value)], [(1,1,0)], twentyfour_pt)
                else:
                    # Delegate to functional area managers via dispatch table
                    if self.current_state in self._state_draw_dispatch:
                        draw_fn = self._state_draw_dispatch.get(self.current_state)
                        if draw_fn is not None:
                            draw_fn(ctx)
                #ctx.restore()

        # Notifications are drawn on top of everything else, so that they are visible regardless of the current state.
        # They also contain animations, so need to be drawn every frame when active.
        # As they 'withdraw' they reveal whatever is underneath them so this must be redrawn every frame while they are active to avoid leaving visual glitches on the screen.
        if self.notification:
            self.notification.draw(ctx)

        diagnostics_output(2, 0)



    @staticmethod
    def clear_leds():
        """Utility function to clear all LEDs. This is used when setting direction LEDs to ensure only the relevant ones are lit."""
        for i in range(1,13):
            tildagonos.leds[i] = (0, 0, 0)


    def apply_motor_calibration(self, output: tuple) -> tuple:
        """Negate individual motor outputs as per settings."""
        output1, output2 = output

        # Apply deadband correction
        if abs(output1) < self._motor_deadband:
            # If the absolute value of output1 is less than the deadband threshold, set it to zero to prevent small motor outputs from being sent to the motors
            output1 = 0
        else:
            # Otherwise apply the motor offset to ensure that the motors start moving when a non-zero output is sent. This compensates for any mechanical resistance or friction in the motor system.
            if output1 > 0:
                output1 = self._motor1_min + ((output1 * (65536 - self._motor1_min)) // 65536)
            else:
                output1 = -self._motor1_min - ((-output1 * (65536 - self._motor1_min)) // 65536)
        if abs(output2) < self._motor_deadband:
            output2 = 0
        else:
            if output2 > 0:
                output2 = self._motor2_min + ((output2 * (65536 - self._motor2_min)) // 65536)
            else:
                output2 = -self._motor2_min - ((-output2 * (65536 - self._motor2_min)) // 65536)

        # limit rate of change of motor output to maximum acceleration
        max_delta = self.acceleration # maximum change in motor output per update
        output1 = self._output1 + _clamp(output1 - self._output1, -max_delta, max_delta)
        output2 = self._output2 + _clamp(output2 - self._output2, -max_delta, max_delta)
        self._output1 = output1
        self._output2 = output2

        return (-output1 if self._motor1_reversed else output1, -output2 if self._motor2_reversed else output2)


    def set_direction_leds(self, direction: Button):
        """LED positions rotate based on 'front_face' (0-11, each step = 30° CW).
        Each position p maps to LED pair: (p if p>0 else 12) and (p+1).
        """
        f = self._front_face
        if direction == BUTTON_TYPES["UP"]:
            pos = f % 12
            colour = (0, 255, 255)   # Cyan = forward
        elif direction == BUTTON_TYPES["RIGHT"]:
            pos = (f + 2) % 12
            colour = (0, 255, 0)     # Green = right
        elif direction == BUTTON_TYPES["DOWN"]:
            pos = (f + 6) % 12
            colour = (255, 0, 255)   # Magenta = backward
        elif direction == BUTTON_TYPES["LEFT"]:
            pos = (f + 8) % 12
            colour = (255, 0, 0)     # Red = left
        else:
            return
        led_a = pos if pos > 0 else 12
        led_b = pos + 1
        self.clear_leds()
        tildagonos.leds[led_a] = colour
        tildagonos.leds[led_b] = colour


    @staticmethod
    def draw_message(ctx, message, colours, size=label_font_size):
        """Utility function to draw a multi-line message on the screen, with optional colour for each line. The message is centred on the screen, and the y-position of each line is adjusted based on the total number of lines to ensure it is visually balanced."""
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


    def return_to_menu(self, menu_name: str | None = None):
        """Utility function to return to the main menu from any state. This is used when the user cancels out of a submenu or after acknowledging a warning message."""
        if self._logging:
            print("Returning to menu")
        if menu_name is not None:
            self.set_menu(menu_name)
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        self.current_state = STATE_MENU
        self.refresh = True
        self.update_settings()
        self.fast_settings_update()


    def show_message(self, msg_content, msg_colours, msg_type = None, return_state: int | None = None):
        """Utility function to set the current state to the message display, and populate the message content and colours. The message_type can be used to indicate whether this is an 'error' (red) or 'warning' (green) message, which can affect both the display and the behaviour when the user acknowledges the message."""
        if self._logging:
            print(f"Showing message: '{msg_content}' with type {msg_type}")
        self.animation_counter = 0
        self.message = msg_content
        self.message_colours = msg_colours
        self.message_type = msg_type
        self.message_return_state = return_state
        self.current_state = STATE_MESSAGE
        self.refresh = True


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
                    if self._logging:
                        print(f"Auto Repeat Level: {self.auto_repeat_level}")

            return True
        return False


    def auto_repeat_clear(self):
        """Reset the auto-repeat counters and level. This should be called when a button is released to ensure that the next button press starts with the initial auto-repeat interval and level."""
        self._auto_repeat = 1+ self._auto_repeat_intervals[0] # so that we trigger immediately on next press

        self._auto_repeat_count = 0
        self.auto_repeat_level = 0



### MENU FUNCTIONALITY ###


    def set_menu(self, menu_name: str | None = "main"):  #: Literal["main"]): does it work without the type hint?
        """Set the current menu to the specified menu name, and construct the menu if necessary.
           If menu_name is None, it will clear the current menu and return to the previous state
           (e.g. from a submenu back to the main menu)."""
        if self._logging:
            print(f"B:Set Menu {menu_name}")
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
            if not self.enable_bluetooth and MAIN_MENU_ITEMS[MENU_ITEM_BLUETOOTH] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_BLUETOOTH])
            if not self.enable_servo_test and MAIN_MENU_ITEMS[MENU_ITEM_SERVO_TEST] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_SERVO_TEST])
            if not self.enable_motor_moves and MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_MOVES] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_MOVES])
            if not self.enable_line_follow and MAIN_MENU_ITEMS[MENU_ITEM_LINE_FOLLOWER] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_LINE_FOLLOWER])
            if not self.enable_sensor_test and MAIN_MENU_ITEMS[MENU_ITEM_SENSOR_TEST] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_SENSOR_TEST])
            if not self.enable_autodrive and MAIN_MENU_ITEMS[MENU_ITEM_AUTO_DRIVE] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_AUTO_DRIVE])
            if not self.enable_hexpansion_mgr and MAIN_MENU_ITEMS[MENU_ITEM_HEXPANSION] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_HEXPANSION])
            if self._settings_mgr is None and MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS] in menu_items:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
            self.menu = Menu(
                    self,
                    menu_items,
                    select_handler=self._main_menu_select_handler,
                    back_handler=self._menu_back_handler,
                    position=self._main_menu_position,
                )
        elif menu_name == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS] and self._settings_mgr is not None: # "Settings"
            # construct the settings menu
            _settings_menu_items = ["Default All"]
            for _, setting in enumerate(self.settings):
                _settings_menu_items.append(f"{setting}")
            self.menu = Menu(
                self,
                _settings_menu_items,
                select_handler=self._settings_menu_select_handler,
                back_handler=self._menu_back_handler,
                position=self.settings_menu_position,
                )


    # this appears to be able to be called at any time
    def _main_menu_select_handler(self, item: str, idx: int):
        if self._logging:
            print(f"H:Main Menu {item} at index {idx} position {self.menu.position if self.menu else 'N/A'}")
        self._main_menu_position = self.menu.position if self.menu else 0
        if item == MAIN_MENU_ITEMS[MENU_ITEM_BLUETOOTH]: # Bluetooth
            if self._bluetooth_mgr is not None and self._hexpansion_mgr is not None:
                self._bluetooth_mgr.logging = self._logging
                # Make unique Bluetooth Name from HexDrive Unique ID, so that multiple BadgeBots can be used in the same area without confusion:
                # Name is limited to 8 characters, so we use "BBot" & a three digit decimal number from the unique ID, which is a 32-bit number, so we take the last three digits of the unique ID modulo 1000
                uniqueid = self._hexpansion_mgr.get_active_hexdrive_unique_id()
                if uniqueid is not None:
                    name = f"BBot{uniqueid % 1000:03d}"
                else:
                    name = None
                if self._bluetooth_mgr.start(name = name):
                    self.current_state = STATE_BLUETOOTH
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_LINE_FOLLOWER]: # Line Follower
            # Check for required hardware and show message if not present, otherwise start the line follower manager and switch to follower state
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification("2 Motors Required")
            else:
                if self._line_follow_mgr is not None:
                    self._line_follow_mgr.logging = self._logging # update logging setting in line follow manager based on current app setting, in case it was changed
                    if self._line_follow_mgr.start():
                        self.current_state = STATE_FOLLOWER
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_MOVES]: # Motor Moves
            # Check for required hardware and show message if not present, otherwise start the motor moves manager and switch to motor moves state
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification("2 Motors Required")
            else:
                if self._motor_moves_mgr is not None:
                    self._motor_moves_mgr.logging = self._logging # update logging setting in motor moves manager based on current app setting, in case it was changed
                    if self._motor_moves_mgr.start():
                        self.current_state = STATE_MOTOR_MOVES
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SERVO_TEST]: # Servo Test
            # Check for required hardware and show message if not present, otherwise start the servo test manager and switch to servo test state
            if self.num_servos == 0:
                self.notification = Notification("No Servos")
            else:
                if self._servo_test_mgr is not None:
                    self._servo_test_mgr.logging = self._logging # update logging setting in servo test manager based on current app setting, in case it was changed
                    if self._servo_test_mgr.start():
                        self.current_state = STATE_SERVO
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SENSOR_TEST]: # Sensor Test
            if self._sensor_test_mgr is not None:
                self._sensor_test_mgr.logging = self._logging # update logging setting in sensor test manager based on current app setting, in case it was changed
                if self._sensor_test_mgr.start():
                    self.current_state = STATE_SENSOR
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_AUTO_DRIVE]: # Auto Drive
            if self._autodrive_mgr is not None:
                self._autodrive_mgr.logging = self._logging # update logging setting in autodrive manager based on current app setting, in case it was changed
                if self._autodrive_mgr.start():
                    self.current_state = STATE_AUTODRIVE
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_HEXPANSION]: # Hexpansion Management
            if self._hexpansion_mgr is not None:
                self._hexpansion_mgr.logging = self._logging # update logging setting in hexpansion manager based on current app setting, in case it was changed
                if self._hexpansion_mgr.start():
                    self.current_state = STATE_HEXPANSION
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]:   # Settings
            self.set_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_ABOUT]:      # About
            self.set_menu(None)
            self.button_states.clear()
            self.animation_counter = 0
            self.current_state = STATE_LOGO
            self.refresh = True
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_EXIT]:       # Exit
            #if self._hexpansion_mgr is not None:
            #    self._hexpansion_mgr.unregister_events()
            #eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
            #eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
            eventbus.emit(RequestStopAppEvent(self))


    def _settings_menu_select_handler(self, item: str, idx: int):
        if self._logging:
            print(f"B:Setting {item} @ {idx}")
        if idx == 0: #Default
            if self._logging:
                print("B:Settings Default All")
            for s in self.settings:
                self.settings[s].v = self.settings[s].d
                self.settings[s].persist()
            self.notification = Notification("Settings Defaulted")
            self.set_menu()
        elif self._settings_mgr is not None and self._settings_mgr.start(item):
            self.current_state = STATE_SETTINGS


    def _menu_back_handler(self):
        if self.current_menu == "main":
            self._main_menu_position = self.menu.position if self.menu else 0
            if self._logging:
                print("B:Save Settings")
            platform_settings.save()         # Save settings before minimising
            self.minimise()
        # for submenus, just return to the main menu
        if self.current_menu == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]:
            self.settings_menu_position = self.menu.position if self.menu else 0
        self.set_menu()


#### DIAGNOSTICS OUTPUT ###
# For monitoring with a scope
def diagnostics_output(index: int, value: int):
    """Output diagnostic values to the HS pins on the diagnostics hexpansion, for measurement with an oscilloscope"""
    #print(f"B: Expected Diagnostics Output: index={index}, value={value}, emit_diagnostics_output={emit_diagnostics_output}")
    if emit_diagnostics_output is not None:
        emit_diagnostics_output(index, value)

# This doesn't seem to be working as expected so there is now an explicit call to register the diagnostics output function.
def __app_init__(app_instance):
    """Register the active app instance as the shared diagnostics sink."""
    if set_diagnostics_output is not None:
        set_diagnostics_output(app_instance.diagnostics_output)


__app_export__ = BadgeBotApp
