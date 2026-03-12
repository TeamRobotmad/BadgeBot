import asyncio
import sys
import time
from math import cos, pi
import ota
import settings
from app_components.notification import Notification
from app_components.tokens import label_font_size, twentyfour_pt, clear_background, button_labels
from app_components import Menu
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionRemovalEvent)
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent,
                                     RequestStopAppEvent)

from tildagonos import tildagonos

import app

from .utils import chain, draw_logo_animated, parse_version

# Import sub-modules for each functional area
from .hexpansion_mgmt import HexpansionMgr, HexpansionType, _ERASE_SLOT, _EEPROM_ADDR
from .motor_moves import MotorMovesMgr, Instruction
from .servo_test import ServoTestMgr, ServoMode, _SERVO_DEFAULT_CENTRE, _SERVO_DEFAULT_RANGE, _SERVO_DEFAULT_RATE, _MAX_SERVO_RANGE
from .step_test import StepperTestMgr, StepperMode, Stepper
from .badgebot_settings import SettingsMgr, MySetting
from .line_follow import (LineFollowMgr, LineSensors, LineSensor,
                          FOLLOWER_SENSOR_SCAN_PERIOD, DEFAULT_UPDATE_PERIOD,
                          FOLLOWER_PID_KP_DEFAULT, FOLLOWER_PID_KI_DEFAULT,
                          FOLLOWER_PID_KD_DEFAULT, FOLLOWER_FORWARD_POWER,
                          FOLLOWER_MODE_DIFFERENTIAL, FOLLOWER_MODE_BINARY,
                          SENSOR_CTRL_PINS, SENSOR_SIGNAL_PINS, SENSOR_NAMES,
                          _LINE_SENSOR_DEFAULT_THRESHOLD, _NUM_LINE_SENSORS)
from .autotune_mgr import AutotuneMgr

#import micropython
#micropython.alloc_emergency_exception_buf(100)


# Hard coded to talk to EEPROMs on address 0x50 - because we know that is what is on the HexDrive Hexpansion
# makes it a lot more efficient than scanning the I2C bus for devices and working out what they are

CURRENT_HEXDRIVE_APP_VERSION = 6 # HEXDRIVE.PY Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating


_APP_VERSION = "0.1" # BadgeBot App Version Number

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

_BRIGHTNESS = 1.0

# Motor Driver - Defaults (remain here as they are used in settings init)
_MAX_POWER = 30000
_POWER_STEP_PER_TICK = 7500  # effectively the acceleration

# Servo Tester - Defaults (remain here for settings init)
_SERVO_DEFAULT_STEP    = 10
_SERVO_DEFAULT_PERIOD  = 20

# Stepper Tester - Defaults (remain here for settings init)
_STEPPER_MAX_POSITION  = 3100

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
STATE_FOLLOWER = 19       # Line Follower
STATE_AUTOTUNE = 20       # PID Auto Tune

# App states where user can minimise app
_MINIMISE_VALID_STATES = [0, 1, 7, 12, 13, 14, 15]
_LED_CONTROL_STATES    = [0, 3, 4, 5, 6, 12, 13, 14, 15, 19]

#Misceallaneous Settings
_LOGGING = False
_IS_SIMULATOR = sys.platform != "esp32"  # True when running in the simulator, not on real badge hardware

# Main Menu Items
_main_menu_items = ["Line Follower","Motor Moves", "PID Auto Tune", "Stepper Test", "Servo Test", "Settings", "About","Exit"]
MENU_ITEM_LINE_FOLLOWER = 0
MENU_ITEM_MOTOR_MOVES = 1
MENU_ITEM_PID_AUTOTUNE = 2
MENU_ITEM_STEPPER_TEST = 3
MENU_ITEM_SERVO_TEST = 4
MENU_ITEM_SETTINGS = 5
MENU_ITEM_ABOUT = 6
MENU_ITEM_EXIT = 7


class LineFollowerApp(app.App):
    # Expose state constants as class attributes for sub-modules
    STATE_INIT = STATE_INIT
    STATE_WARNING = STATE_WARNING
    STATE_DETECTED = STATE_DETECTED
    STATE_UPGRADE = STATE_UPGRADE
    STATE_CHECK = STATE_CHECK
    STATE_REMOVED = STATE_REMOVED
    STATE_MENU = STATE_MENU

    def __init__(self):
        super().__init__()
        # UI Button Controls
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self.long_press_delta: int = 0
        self._auto_repeat_intervals = [ _AUTO_REPEAT_MS, _AUTO_REPEAT_MS//2, _AUTO_REPEAT_MS//4, _AUTO_REPEAT_MS//8, _AUTO_REPEAT_MS//16] # at the top end the loop is unlikley to cycle this fast
        self._auto_repeat: int = 0
        self._auto_repeat_count: int = 0
        self._auto_repeat_level: int = 0

        # UI Feature Controls
        self._refresh: bool = True
        self.rpm: float = 5                    # logo rotation speed in RPM
        self._animation_counter: float = 0
        self._pattern_status: bool = True     # True = Pattern Enabled, False = Pattern Disabled
        self.qr_code = _QR_CODE
        self._APP_VERSION = _APP_VERSION
        self.b_msg: str = f"BadgeBot V{_APP_VERSION}"
        self.t_msg: str = "RobotMad"
        self.is_scroll: bool = False
        self.scroll_offset: int = 0
        self.notification: Notification = None
        self.error_message = []
        self.current_menu: str = None
        self.menu: Menu = None

        # BadgeBot Control Sequence Variables
        self.run_countdown_elapsed_ms: int = 0
        self._countdown_next_state: int = STATE_RUN  # which state to go to after countdown
        self.instructions = []
        self.current_instruction: Instruction = None
        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])

        # Settings
        self._settings = {}
        self._settings['acceleration']  = MySetting(self._settings, _POWER_STEP_PER_TICK, 1, 65535)
        self._settings['max_power']     = MySetting(self._settings, _MAX_POWER, 1000, 65535)
        self._settings['drive_step_ms'] = MySetting(self._settings, _USER_DRIVE_MS, 5, 200)
        self._settings['turn_step_ms']  = MySetting(self._settings, _USER_TURN_MS, 5, 200)
        self._settings['servo_step']    = MySetting(self._settings, _SERVO_DEFAULT_STEP, 1, 100)
        self._settings['servo_range']   = MySetting(self._settings, _SERVO_DEFAULT_RANGE, 100, _MAX_SERVO_RANGE)  # one setting for all servos
        self._settings['servo_period']  = MySetting(self._settings, _SERVO_DEFAULT_PERIOD, 5, 50)
        self._settings['brightness']    = MySetting(self._settings, _BRIGHTNESS, 0.1, 1.0)
        self._settings['logging']       = MySetting(self._settings, _LOGGING, False, True)
        self._settings['erase_slot']    = MySetting(self._settings, _ERASE_SLOT, 0, 6)
        self._settings['step_max_pos']  = MySetting(self._settings, _STEPPER_MAX_POSITION, 0, 65535)
        self._settings['line_threshold'] = MySetting(self._settings, _LINE_SENSOR_DEFAULT_THRESHOLD, 0, 65535)
        self._settings['pid_kp']         = MySetting(self._settings, FOLLOWER_PID_KP_DEFAULT, 0.0, 1000.0)
        self._settings['pid_ki']         = MySetting(self._settings, FOLLOWER_PID_KI_DEFAULT, 0.0, 1000.0)
        self._settings['pid_kd']         = MySetting(self._settings, FOLLOWER_PID_KD_DEFAULT, 0.0, 1000.0)

        self._edit_setting: int  = None
        self._edit_setting_value = None       
        self.update_settings()   

        # Check what version of the Badge s/w we are running on
        try:
            ver = parse_version(ota.get_version())
            if ver is not None:
                if self._settings['logging'].v:
                    print(f"BadgeSW V{ver}")
                # Potential to do things differently based on badge s/w version
                # e.g. if ver < [1, 9, 0]:
        except:
            pass

        # Hexpansion related
        self._HEXPANSION_TYPES = [HexpansionType(0xCBCB, "HexDrive", motors=2, servos=4, app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                  HexpansionType(0xCBCA, "HexDrive", motors=2,           sub_type="2 Motor", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                  HexpansionType(0xCBCC, "HexDrive", servos=4,           sub_type="4 Servo", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                  HexpansionType(0xCBCD, "HexDrive", motors=1, servos=2, sub_type="1 Mot 2 Srvo", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"),
                                  HexpansionType(0xCBCE, "HexDrive", steppers=1,         sub_type="Stepper", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                  HexpansionType(0xCBCF, "HexSense", sensors=2,          sub_type="2 Line Sensors")] # , app_mpy_name="hexsense.mpy", app_mpy_version=CURRENT_HEXSENSE_APP_VERSION, app_name="HexSenseApp")]  
        self.hexpansion_slot_type = [None]*6
        self.hexpansion_init_type: int = 0
        self.detected_port: int = None
        self.waiting_app_port: int = None
        self.erase_port: int  = None
        self.upgrade_port: int = None
        self.hexdrive_port: int = None
        self.ports_with_blank_eeprom = set()
        self.ports_with_hexdrive = set()
        self.ports_with_hexsense = set()
        self.ports_with_latest_hexdrive = set()
        self.hexdrive_app = None
        self.hexpansion_update_required: bool = False # flag from async to main loop
        self._time_since_last_update: int = 0

        # Functional area managers
        self._hexpansion_mgr = HexpansionMgr(self)
        self._motor_moves_mgr = MotorMovesMgr(self)
        self._servo_test_mgr = ServoTestMgr(self)
        self._stepper_test_mgr = StepperTestMgr(self)
        self._settings_mgr = SettingsMgr(self)
        self._line_follow_mgr = LineFollowMgr(self)
        self._autotune_mgr = AutotuneMgr(self)

        eventbus.on_async(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)

        # Motor Driver
        self.num_motors: int = 2       # Default assumed for a single HexDrive
        self.num_steppers: int = 1       # Default assumed for a single HexDrive
        self._stepper: Stepper = None
        self.stepper_mode = StepperMode()
        self.stepper_pos: int = 0
        self._output = (0,0)

        # Servo Tester
        self._time_since_last_input: int = 0
        self._timeout_period: int = 120000                     # ms (2 minutes - without any user input)       
        self._keep_alive_period: int = 500                     # ms (half the value used in hexdrive.py)  
        self.num_servos: int     = 4                           # Default assumed for a single HexDrive
        self.servo               = [None]*4                    # Servo Positions
        self.servo_centre        = [_SERVO_DEFAULT_CENTRE]*4   # Trim Servo Centre
        self.servo_range         = [_SERVO_DEFAULT_RANGE]*4    # Limit Servo Range
        self.servo_rate          = [_SERVO_DEFAULT_RATE]*4     # Servo Rate of Change
        self.servo_mode          = [ServoMode()]*4             # Servo Mode
        self.servo_selected: int = 0

        # Line Follower
        self._override = False
        self.num_line_sensors: int = 0                          # initialised to 0 until we detect a HexSense Hexpansion and can set this based on the actual number of sensors it has 
        self._s = [False, False]
        self._line_sensors = None                               # Will be a LineSensors instance when active
        self._line_sensors_hexpansion_config  = None            # Store the HexpansionConfig of the HexSense that is providing the line sensors
        self._sample_count: int  = 0
        self._sample_time: int   = 0
        self._rate: int = 0     # sample rate
        self._follower_mode: int = FOLLOWER_MODE_DIFFERENTIAL   # Default follower mode
        self._forward_power: int = -FOLLOWER_FORWARD_POWER      # Default forward power for line follower (sign sets direction)
        self._pid_integral: int = 0                             # Accumulated integral term for PID controller
        self._pid_previous_error: int = 0                       # Previous error for derivative term of PID controller

        # PID Auto Tune
        self._autotuner = None

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_INIT
        self.previous_state = self.current_state
        self._update_period = DEFAULT_UPDATE_PERIOD  # ms

        eventbus.on_async(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self._lose_focus, self)

        # We start with focus on launch, without an event emmited
        # This version is compatible with the simulator
        asyncio.get_event_loop().create_task(self._gain_focus(RequestForegroundPushEvent(self)))  


    ### ASYNC EVENT HANDLERS ###

    async def _handle_hexpansion_removal(self, event: HexpansionRemovalEvent):
        await self._hexpansion_mgr.handle_removal(event)

    async def _handle_hexpansion_insertion(self, event: HexpansionInsertionEvent):
        await self._hexpansion_mgr.handle_insertion(event)


    async def _gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            if self.current_state in _LED_CONTROL_STATES:
                eventbus.emit(PatternDisable())
            elif self.current_state == STATE_RECEIVE_INSTR:
                eventbus.on_async(ButtonUpEvent, self._handle_button_up, self)


    async def _lose_focus(self, event: RequestForegroundPopEvent):
        if event.app is self:
            eventbus.emit(PatternEnable())
            self._pattern_status = True
            if self.current_state == STATE_RECEIVE_INSTR:
                eventbus.remove(ButtonUpEvent, self._handle_button_up, self)            


    async def _handle_button_up(self, event: ButtonUpEvent):
        if self.current_state == STATE_RECEIVE_INSTR and event.button == BUTTONS["C"]:
            self.is_scroll = not self.is_scroll
            state = "yes" if self.is_scroll else "no"
            self.notification = Notification(f"Scroll {state}")


    async def background_task(self):
        # Modiifed background task loop for shorter sleep time
        last_time = time.ticks_ms()
        while True:
            cur_time = time.ticks_ms()
            delta_ticks = time.ticks_diff(cur_time, last_time)
            self.background_update(delta_ticks)
            await asyncio.sleep_ms(self._update_period)
            last_time = cur_time


    ### NON-ASYNC FUCNTIONS ###

    def _compute_differential_output(self):
        """Compute motor output using a full PID controller for differential line following.
        
        Uses the difference between left and right sensor readings as the error signal,
        and applies proportional, integral, and derivative terms to compute a steering correction.
        Returns a tuple of (left_motor, right_motor) power values, clamped to max_power.
        """
        # Calculate the error as the difference between the two sensor readings
        error = self._line_sensors.raw_value(0) - self._line_sensors.raw_value(1)

        # Proportional term
        p_term = self._settings['pid_kp'].v * error

        # Integral term - accumulate error over time with anti-windup clamping
        self._pid_integral += error
        max_pwr = self._settings['max_power'].v
        if self._settings['pid_ki'].v > 0:
            integral_limit = max_pwr // self._settings['pid_ki'].v
            self._pid_integral = max(min(self._pid_integral, integral_limit), -integral_limit)
        i_term = self._settings['pid_ki'].v * self._pid_integral

        # Derivative term - rate of change of error
        d_term = self._settings['pid_kd'].v * (error - self._pid_previous_error)
        self._pid_previous_error = error

        # Combined PID output
        correction = p_term + i_term + d_term

        # Combine correction with base forward power to get output for each motor
        output = (self._forward_power + correction, self._forward_power - correction)

        # Limit output to max power
        output = (max(min(output[0], max_pwr), -max_pwr), max(min(output[1], max_pwr), -max_pwr))

        if self._settings['logging'].v:
            print(f"PID: err={error} P={p_term} I={i_term} D={d_term} corr={correction} out={output}")

        return output

    def background_update(self, delta: int):
        self._motor_moves_mgr.background_update(delta)
        self._line_follow_mgr.background_update(delta)
        self._autotune_mgr.background_update(delta)


    def generate_new_qr(self):
        from .uQR import QRCode
        qr = QRCode(error_correction=1, box_size=10, border=0)
        qr.add_data("https://robotmad.odoo.com")
        self.qr_code = qr.get_matrix()
        # convert QR code made up of True/False into words of 1s and 0s
        if 32 < len(self.qr_code):
            print("QR code too big")
        else:
            qr_code_size = len(self.qr_code)
            print("_QR_CODE = [")
            for row in range(qr_code_size):
                bitfield = 0x00000000
                for col in range(qr_code_size):
                    # LSBit is on the left
                    bitfield = bitfield | (1 << col) if self.qr_code[row][col] else bitfield
                print(f"0x{bitfield:08x},")
            print("]")


    ### HEXPANSION FUNCTIONS - delegated to HexpansionMgr ###


    def update_settings(self):
        for s in self._settings:
            self._settings[s].v = settings.get(f"badgebot.{s}", self._settings[s].d)


    def _pattern_management(self):        
        if self.current_state in _LED_CONTROL_STATES:
            if self._pattern_status:
                eventbus.emit(PatternDisable())
                self._pattern_status = False
                # delay enough to allow the pattern to stop
                time.sleep_ms(500)
        elif self.current_state not in _LED_CONTROL_STATES and not self._pattern_status:
            eventbus.emit(PatternEnable())
            self._pattern_status = True


    ### MAIN APP CONTROL FUNCTIONS ###

    def update(self, delta: int):
        if self.notification:
            self.notification.update(delta)

        # manage PatternEnable/Disable for all states
        self._pattern_management()

        self._update_hexpansion_management(delta)
        self._update_main_application(delta)


        if self.current_state != self.previous_state:
            if self._settings['logging'].v:
                print(f"State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # manage PatternEnable/Disable for all states
            self._pattern_management()
            # something has changed - so worth redrawing
            self._refresh = True

        if self.current_state in _LED_CONTROL_STATES:
            if self._settings['brightness'].v < 1.0:
                # Scale brightness
                for i in range(1,13):
                    tildagonos.leds[i] = tuple(int(j * self._settings['brightness'].v) for j in tildagonos.leds[i])                            
            tildagonos.leds.write()


    ### START UI FOR HEXPANSION INITIALISATION AND UPGRADE ###

    def _update_hexpansion_management(self, delta: int):
        self._hexpansion_mgr.update(delta)

    def _update_main_application(self, delta: int):
        if self.current_state == STATE_MENU:
            if self.current_menu is None:
                self.set_menu("main")
                self._refresh = True
            else:
                self.menu.update(delta)    
                if self.menu.is_animating != "none":
                    if self._settings['logging'].v:
                        print("Menu is animating")
                    self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in _MINIMISE_VALID_STATES:
            self.button_states.clear()
            self.is_scroll = False
            self.minimise()

    ### Shared Countdown (used by Motor Moves and PID AutoTune) ###
        elif self.current_state == STATE_COUNTDOWN:
            self._update_state_countdown(delta)

    ### Delegate to functional area managers ###
        elif self._motor_moves_mgr.update(delta):
            pass
        elif self._line_follow_mgr.update(delta):
            pass
        elif self._autotune_mgr.update(delta):
            pass
        elif self._servo_test_mgr.update(delta):
            pass
        elif self._stepper_test_mgr.update(delta):
            pass
        elif self._settings_mgr.update(delta):
            pass
    ### End of Update ###


    def _update_state_warning(self, delta: int):
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            if self.current_state == STATE_WARNING or self.hexdrive_port is not None:
                # Warning has been acknowledged by the user
                self._animation_counter = 0
                self.current_state = STATE_MENU # allow access to settings and About
            else:
                # Return to Warning screen from Logo when no HexDrive is present
                self.current_state = STATE_WARNING    
        else:
            # "CANCEL" button is handled below in common for all MINIMISE_VALID_STATES 
            # Show the warning screen for 10 seconds
            self._animation_counter += delta/1000
            self._refresh = True
            if self.current_state == STATE_WARNING and self._animation_counter > 10:
                # after 10 seconds show the logo
                self._animation_counter = 0
                self.current_state = STATE_LOGO
            elif self.current_state == STATE_LOGO:
                # LED management - to match rotating logo:
                for i in range(1,13):
                    colour = (255, 241, 0)      # custom Robotmad shade of yellow                                
                    # raised cosine cubed wave
                    wave = self._settings['brightness'].v * pow((1.0 + cos(((i) *  pi / 1.5) - (self.rpm * self._animation_counter * pi / 7.5)))/2.0, 3)    
                    # 4 sides each projecting a pattern of 3 LEDs (12 LEDs in total)
                    tildagonos.leds[i] = tuple(int(wave * j) for j in colour)                                                     
            else: # STATE_WARNING
                for i in range(1,13):
                    tildagonos.leds[i] = (255,0,0)   


    def _update_state_error(self, delta: int):                
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            # Error has been acknowledged by the user
            self.button_states.clear()
            self.current_state = STATE_CHECK
            self.error_message = []
        else:
            for i in range(1,13):
                tildagonos.leds[i] = (0,255,0) if self.current_state == STATE_MESSAGE else (255,0,0)       


    def _update_state_countdown(self, delta: int):            
        self.clear_leds()
        self.run_countdown_elapsed_ms += delta
        if self.run_countdown_elapsed_ms >= _RUN_COUNTDOWN_MS:
            next_state = self._countdown_next_state
            if next_state == STATE_RUN:
                # Motor Moves: build the power plan and start running
                self.power_plan_iter = chain(*(instr.power_plan for instr in self.instructions))
                if self.hexdrive_app is not None:
                    self.hexdrive_app.set_power(True)
                self.current_state = STATE_RUN
                self._update_period = 10
            elif next_state == STATE_AUTOTUNE:
                # PID AutoTune: start the tuner after countdown
                self._autotune_mgr.begin_tuning()
            else:
                # Generic fallback
                self.current_state = next_state









    def draw(self, ctx):
        if self._refresh or self.notification is not None:
            self._refresh = False
            clear_background(ctx)   
            ctx.save()
            ctx.font_size = label_font_size
            if ctx.text_align != ctx.LEFT:
                # See https://github.com/emfcamp/badge-2024-software/issues/181             
                ctx.text_align = ctx.LEFT
            ctx.text_baseline = ctx.BOTTOM            
            if self.current_state == STATE_LOGO:
                draw_logo_animated(ctx, self.rpm, self._animation_counter, [self.b_msg, self.t_msg], self.qr_code)
            # Scroll mode indicator
            elif self.is_scroll:
                ctx.rgb(0,0.2,0).rectangle(     -120,-120, 115+(-63),240).fill()
                ctx.rgb(0,0  ,0).rectangle((-63)-5,-120,10-2*(-63),240).fill()
                ctx.rgb(0,0.2,0).rectangle(5-(-63),-120, 115+(-63),240).fill()
            else:
                ctx.rgb(0,0,0).rectangle(-120,-120,240,240).fill()
            # Common states (kept in main app)
            if   self.current_state == STATE_WARNING:
                self.draw_message(ctx, ["BadgeBot requires","HexDrive hexpansion","from RobotMad","github.com","/TeamRobotmad","/BadgeBot"], [(1,1,1),(1,1,0),(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
            elif self.current_state == STATE_REMOVED:
                self.draw_message(ctx, ["HexDrive","removed.","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], label_font_size)      
            elif self.current_state == STATE_ERROR:
                self.draw_message(ctx, self.error_message, [(1,0,0)]*len(self.error_message), label_font_size)
            elif self.current_state == STATE_MESSAGE:
                self.draw_message(ctx, self.error_message, [(0,1,0)]*len(self.error_message), label_font_size)            
            elif self.current_state == STATE_COUNTDOWN:
                countdown_val = 1 + ((_RUN_COUNTDOWN_MS - self.run_countdown_elapsed_ms) // 1000)
                self.draw_message(ctx, [str(countdown_val)], [(1,1,0)], twentyfour_pt)
            # Delegate to functional area managers
            elif self._hexpansion_mgr.draw(ctx):
                pass
            elif self._motor_moves_mgr.draw(ctx):
                pass
            elif self._line_follow_mgr.draw(ctx):
                pass
            elif self._autotune_mgr.draw(ctx):
                pass
            elif self._servo_test_mgr.draw(ctx):
                pass
            elif self._stepper_test_mgr.draw(ctx):
                pass
            elif self._settings_mgr.draw(ctx):
                pass
            ctx.restore()

        # These need to be drawn every frame as they contain animations
        if self.current_state == STATE_MENU:
            clear_background(ctx)               
            self.menu.draw(ctx)

        if self.notification:
            self.notification.draw(ctx)



    # Value increment/decrement functions for positive integers only
    def _inc(self, v: int, l: int):
        if l==0:
            return v+1
        else:
            d = 10**l
            v = ((v // d) + 1) * d   # round up to the next multiple of 10^l
            return v
    
    def _dec(self, v: int, l: int):
        if l==0:
            return v-1
        else:
            d = 10**l
            v = (((v+(9*(10**(l-1)))) // d) - 1) * d   # round down to the next multiple of 10^l
            return v


    def clear_leds(self):
        for i in range(1,13):
            tildagonos.leds[i] = (0, 0, 0)


    def draw_message(self, ctx, message, colours, size=label_font_size):
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


    def _set_direction_leds(self, direction: Button):
        if direction == BUTTON_TYPES["RIGHT"]:
            # Green = Starboard = Right
            self.clear_leds()
            tildagonos.leds[2]  = (0, 255, 0)
            tildagonos.leds[3]  = (0, 255, 0)                
        elif direction ==BUTTON_TYPES["LEFT"]:
            # Red = Port = Left
            self.clear_leds()
            tildagonos.leds[8]  = (255, 0, 0)
            tildagonos.leds[9]  = (255, 0, 0)                
        elif direction == BUTTON_TYPES["UP"]:
            # Cyan
            self.clear_leds()
            tildagonos.leds[12] = (0, 255, 255)
            tildagonos.leds[1]  = (0, 255, 255)                
        elif direction == BUTTON_TYPES["DOWN"]:
            # Magenta
            self.clear_leds()
            tildagonos.leds[6]  = (255, 0, 255)
            tildagonos.leds[7]  = (255, 0, 255)                


    # multi level auto repeat
    # if speed_up is True, the auto repeat gets faster the longer the button is held
    # otherwise it is a fixed rate, but the level is used to determine the scale of the increase in the setttings inc() and dec() functions
    def _auto_repeat_check(self, delta: int, speed_up: bool = True) -> bool:                
        self._auto_repeat += delta
        # multi stage auto repeat - the repeat gets faster the longer the button is held
        if self._auto_repeat > self._auto_repeat_intervals[self._auto_repeat_level if speed_up else 0]:
            self._auto_repeat = 0
            self._auto_repeat_count += 1
            # variable threshold to count to increase level so that it is not too easy to get to the highest level as the auto repeat period is reduced
            if self._auto_repeat_count > ((_AUTO_REPEAT_COUNT_THRES*_AUTO_REPEAT_MS) // self._auto_repeat_intervals[self._auto_repeat_level if speed_up else 0]):
                self._auto_repeat_count = 0
                if self._auto_repeat_level < (_AUTO_REPEAT_SPEED_LEVEL_MAX if speed_up else _AUTO_REPEAT_LEVEL_MAX):
                    self._auto_repeat_level += 1
                    if self._settings['logging'].v:
                        print(f"Auto Repeat Level: {self._auto_repeat_level}")

            return True
        return False


    def _auto_repeat_clear(self):                
        self._auto_repeat = 1+ self._auto_repeat_intervals[0] # so that we trigger immediately on next press 

        self._auto_repeat_count = 0 
        self._auto_repeat_level = 0



### MENU FUNCTIONALITY ###


    def set_menu(self, menu_name = "main"):  #: Literal["main"]): does it work without the type hint?
        if self._settings['logging'].v:
            print(f"H:Set Menu {menu_name}")
        if self.menu is not None:
            try:
                self.menu._cleanup()
            except:
                # See badge-2024-software PR#168
                # in case badge s/w changes and this is done within the menu s/w
                # and then access to this function is removed
                pass
        self.current_menu = menu_name
        if menu_name == "main":
            # construct the main menu based on template
            menu_items = _main_menu_items.copy()
            if self.num_servos == 0:
                menu_items.remove(_main_menu_items[MENU_ITEM_SERVO_TEST])   
            if self.num_steppers == 0:
                menu_items.remove(_main_menu_items[MENU_ITEM_STEPPER_TEST])   
            if self.num_motors == 0:
                menu_items.remove(_main_menu_items[MENU_ITEM_MOTOR_MOVES])
                menu_items.remove(_main_menu_items[MENU_ITEM_LINE_FOLLOWER])
                menu_items.remove(_main_menu_items[MENU_ITEM_PID_AUTOTUNE])
            if self.num_line_sensors == 0:
                menu_items.remove(_main_menu_items[MENU_ITEM_LINE_FOLLOWER])
                if _main_menu_items[MENU_ITEM_PID_AUTOTUNE] in menu_items:
                    menu_items.remove(_main_menu_items[MENU_ITEM_PID_AUTOTUNE])
            self.menu = Menu(
                    self,
                    menu_items,
                    select_handler=self._main_menu_select_handler,
                    back_handler=self._menu_back_handler,
                )            
        elif menu_name == "Settings":
            # construct the settings menu
            _settings_menu_items = ["SAVE ALL", "DEFAULT ALL"]
            for _, setting in enumerate(self._settings):
                _settings_menu_items.append(f"{setting}")
            self.menu = Menu(
                self,
                _settings_menu_items,
                select_handler=self._settings_menu_select_handler,
                back_handler=self._menu_back_handler,
                )


    # this appears to be able to be called at any time
    def _main_menu_select_handler(self, item: str, idx: int):
        if self._settings['logging'].v:
            print(f"H:Main Menu {item} at index {idx}")
        if   item == _main_menu_items[MENU_ITEM_LINE_FOLLOWER]: # Line Follower
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                self._line_follow_mgr.start()
        elif item == _main_menu_items[MENU_ITEM_MOTOR_MOVES]: # Motor Moves
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                self._motor_moves_mgr.start()
        elif item == _main_menu_items[MENU_ITEM_PID_AUTOTUNE]: # PID Auto Tune
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                self._autotune_mgr.start()
        elif item == _main_menu_items[MENU_ITEM_STEPPER_TEST]: # Stepper Test
            if self.num_steppers == 0:
                self.notification = Notification("No Steppers")
            else:
                self._stepper_test_mgr.start()
        elif item == _main_menu_items[MENU_ITEM_SERVO_TEST]: # Servo Test
            if self.num_servos == 0:
                self.notification = Notification("No Servos")
            else:
                self._servo_test_mgr.start()
        elif item == _main_menu_items[MENU_ITEM_SETTINGS]: # Settings
            self.set_menu(_main_menu_items[MENU_ITEM_SETTINGS])
        elif item == _main_menu_items[MENU_ITEM_ABOUT]: # About
            self.set_menu(None)
            self.button_states.clear()
            self._animation_counter = 0
            self.current_state = STATE_LOGO
            self._refresh = True   
        elif item == _main_menu_items[MENU_ITEM_EXIT]: # Exit
            eventbus.remove(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
            eventbus.remove(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)
            eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
            eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
            eventbus.emit(RequestStopAppEvent(self))

    def _settings_menu_select_handler(self, item: str, idx: int):
        if self._settings['logging'].v:
            print(f"H:Setting {item} @ {idx}")
        if idx == 0: #Save
            if self._settings['logging'].v:
                print("H:Settings Save All")
            settings.save()
            self.notification = Notification("  Settings  Saved")
            self.set_menu("main")
        elif idx == 1: #Default
            if self._settings['logging'].v:
                print("H:Settings Default All")
            for s in self._settings:
                self._settings[s].v = self._settings[s].d
                self._settings[s].persist()
            self.notification = Notification("  Settings Defaulted")

            self.set_menu("main")
        else:
            self.set_menu(None)
            self.button_states.clear()
            self.current_state = STATE_SETTINGS
            self._refresh = True
            self._auto_repeat_clear()
            self._edit_setting = item
            self._edit_setting_value = self._settings[item].v


    def _menu_back_handler(self):
        if self.current_menu == "main":
            self.minimise()
        # There are only two menus so this is the only other option    
        self.set_menu("main")


### BADGEBOT DEMO FUNCTIONALITY ###



__app_export__ = LineFollowerApp
