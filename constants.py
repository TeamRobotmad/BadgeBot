"""BadgeBot application constants and configuration values."""
import sys

CURRENT_APP_VERSION = 6  # HEXDRIVE.PY Integer Version Number

_APP_VERSION = "1.5"  # BadgeBot App Version Number

# If you change the URL then you will need to regenerate the QR code
_QR_CODE = [0x1fcf67f,
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
            0x18bbd7f]

# Screen positioning for movement sequence text
H_START = -63
V_START = -58
_BRIGHTNESS = 1.0

# Motor Driver - Defaults
_MAX_POWER = 65535
_POWER_STEP_PER_TICK = 7500  # effectively the acceleration

# Servo Tester - Defaults
_SERVO_DEFAULT_STEP    = 10         # us per step
_SERVO_DEFAULT_CENTRE  = 1500       # us
_SERVO_DEFAULT_RANGE   = 1000       # +/- 500us from centre
_SERVO_DEFAULT_RATE    = 25         # *10us per s
_SERVO_DEFAULT_MODE    = 0          # Off
_SERVO_DEFAULT_PERIOD  = 20         # ms
_SERVO_MAX_RATE        = 1000       # *10us per s
_SERVO_MIN_RATE        = 1          # *10us per s
_SERVO_MAX_TRIM        = 1000       # us
_MAX_SERVO_RANGE       = 1400       # 1400us either side of centre (VERY WIDE)

# Stepper Tester - Defaults
_STEPPER_MAX_SPEED     = 200        # full steps per second
_STEPPER_MAX_POSITION  = 3100       # full steps from one end to the other end
_STEPPER_DEFAULT_SPEED = 50         # full steps per second
_STEPPER_NUM_PHASES    = 8          # half steps
_STEPPER_DEFAULT_SPR   = 200        # full steps per revolution
_STEPPER_DEFAULT_STEP  = 1          # half steps, (2 = full steps)

# Timings
_TICK_MS       =  10        # Smallest unit of change for power, in ms
_USER_DRIVE_MS =  50        # User specifed drive durations, in ms
_USER_TURN_MS  =  20        # User specifed turn durations, in ms
_LONG_PRESS_MS = 750        # Time for long button press to register, in ms
_RUN_COUNTDOWN_MS = 5000    # Time after running program until drive starts, in ms
_AUTO_REPEAT_MS = 200       # Time between auto-repeats, in ms
_AUTO_REPEAT_COUNT_THRES = 10 # Number of auto-repeats before increasing level
_AUTO_REPEAT_SPEED_LEVEL_MAX = 4  # Maximum level of auto-repeat speed increases
_AUTO_REPEAT_LEVEL_MAX = 3  # Maximum level of auto-repeat digit increases


# App states
STATE_INIT = -1
STATE_WARNING = 0
STATE_MENU = 1
STATE_HELP = 2
STATE_RECEIVE_INSTR = 3
STATE_COUNTDOWN = 4
STATE_RUN = 5
STATE_DONE = 6
STATE_CHECK = 7           # Checks for EEPROMs and HexDrives
STATE_DETECTED = 8        # Hexpansion ready for EEPROM initialisation
STATE_UPGRADE = 9         # Hexpansion ready for EEPROM upgrade
STATE_ERASE = 10          # Hexpansion ready for EEPROM erase
STATE_PROGRAMMING = 11    # Hexpansion EEPROM programming
STATE_REMOVED = 12        # Hexpansion removed
STATE_ERROR = 13          # Hexpansion error
STATE_MESSAGE = 14        # Message display
STATE_LOGO = 15           # Logo display
STATE_SERVO = 16          # Servo test
STATE_STEPPER = 17        # Stepper test
STATE_SETTINGS = 18       # Edit Settings
STATE_SENSOR = 19         # Sensor test
STATE_AUTO = 20           # Autonomous drive

# App states where user can minimise app
_MINIMISE_VALID_STATES = [0, 1, 7, 12, 13, 14, 15]
_LED_CONTROL_STATES    = [0, 3, 4, 5, 6, 12, 13, 14, 15]

# HexDrive Hexpansion constants
_EEPROM_ADDR  = 0x50
_EEPROM_NUM_ADDRESS_BYTES = 2
_EEPROM_PAGE_SIZE = 32
_EEPROM_TOTAL_SIZE = 64 * 1024 // 8

# Miscellaneous Settings
_LOGGING = False
_ERASE_SLOT = 0   # Slot for user to set if they want to erase EEPROMs on HexDrives
_IS_SIMULATOR = sys.platform != "esp32"

# Motor direction setting
_FWD_DIR_DEFAULT = 0
_FWD_DIR_LABELS  = ("Normal", "Reverse")

# Front face setting (LEDs only)
_FRONT_FACE_DEFAULT = 0
_FRONT_FACE_LABELS = (
    "BtnA",   # 0  - corner A: between slot 6 & slot 1  (default top)
    "Slot 1",  # 1  - slot 1 edge
    "BtnB",   # 2  - corner B: between slot 1 & slot 2
    "Slot 2",  # 3  - slot 2 edge
    "BtnC",   # 4  - corner C: between slot 2 & slot 3
    "Slot 3",  # 5  - slot 3 edge
    "BtnD",   # 6  - corner D: between slot 3 & slot 4
    "Slot 4",  # 7  - slot 4 edge
    "BtnE",   # 8  - corner E: between slot 4 & slot 5
    "Slot 5",  # 9  - slot 5 edge
    "BtnF",   # 10 - corner F: between slot 5 & slot 6
    "Slot 6",  # 11 - slot 6 edge
)

# Auto-drive sub-states
_AUTO_SUB_DRIVE     = 0       # driving forward
_AUTO_SUB_SCAN      = 1       # spinning, sampling ToF
_AUTO_SUB_TURN      = 2       # turning to best heading
_AUTO_SUB_TURN_BACK = 3       # sweeping back using sensor feedback
_AUTO_SUB_REVERSE   = 4       # backing away from obstacle before scan

# Backup-before-scan: reverse for a fixed time if obstacle is very close
_AUTO_BACKUP_MM  = 200   # mm - if obstacle closer than this, reverse first
_AUTO_BACKUP_MS  = 600   # ms to reverse before starting scan
_AUTO_BACKUP_SPEED_FRAC = 0.7  # fraction of auto_speed used while reversing

# Auto-drive defaults
_AUTO_DRIVE_SPEED  = 56000    # ~43% max power
_AUTO_OBSTACLE_MM  = 100      # mm — trigger scan below this
_AUTO_SCAN_SLOTS   = 12       # samples per 360° scan
_AUTO_SLOT_MS      = 700      # ms per scan slot
_AUTO_SENSOR_READ_MS = 50     # sensor read interval during auto
_AUTO_MIN_FWD_MS   = 400      # ms minimum forward run before another scan trigger
_AUTO_CRUISE_MIN_PWM = 36000  # minimum sustained PWM while driving forward
_AUTO_SCAN_FORWARD_ONLY = False  # True = scan/turn without reverse motor commands
_AUTO_CLEAR_DIST_MM = 255      # score used when sensor returns None (clear/no object)
_AUTO_SCAN_TIMEOUT_MS = 14000  # safety timeout for the 360° scan (fallback if gyro stalls)
_AUTO_TURN_BACK_TOLERANCE_MM = 35   # ±mm around best_dist to consider heading matched
_AUTO_TURN_BACK_TIMEOUT_MS   = 4000 # safety timeout for the sensor-feedback back-sweep
_AUTO_TURN_BACK_SPEED_FRAC   = 0.5  # fraction of auto_speed used during back-sweep

# IMU-aided turn control
_AUTO_GYRO_AXIS         = 2     # index into gyro_read() tuple for yaw (0=X,1=Y,2=Z)
_AUTO_GYRO_DEADBAND_DPS = 3.0   # ignore gyro readings below this magnitude (noise floor)

_main_menu_items = ["Motor Moves", "Stepper Test", "Servo Test", "Settings", "Sensor Test", "Auto Drive", "About", "Exit"]
