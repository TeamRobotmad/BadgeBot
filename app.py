""" Main Application File for BadgeBot."""
import asyncio
import sys
import time
from math import cos, pi
import ota
import settings
from app_components.notification import Notification
from app_components.tokens import label_font_size, twentyfour_pt, clear_background
from app_components import Menu
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from system.eventbus import eventbus
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent,
                                     RequestStopAppEvent)
from tildagonos import tildagonos

import app

from .utils import draw_logo_animated, parse_version

# If you could use hard=True in setting up a Pin IRQ hander, which you can't as of BadgeOS V1.10, then it is recommended to 
# allocate the emergency exception buffer to prevent crashes due to OSError: Out of memory when an interrupt occurs and 
# there is no memory available to handle the exception.
#import micropython
#micropython.alloc_emergency_exception_buf(100)


CURRENT_HEXDRIVE_APP_VERSION = 7 # HEXDRIVE.PY Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating


APP_VERSION = "1.5" # BadgeBot App Version Number

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

_BRIGHTNESS = 1.0

# Screen positioning constant for scroll mode display
H_START = -63

# Timings
_LONG_PRESS_MS = 750        # Time for long button press to register, in ms
_RUN_COUNTDOWN_MS = 5000    # Time after running program until drive starts, in ms
_AUTO_REPEAT_MS = 200       # Time between auto-repeats, in ms
_AUTO_REPEAT_COUNT_THRES = 10 # Number of auto-repeats before increasing level
_AUTO_REPEAT_SPEED_LEVEL_MAX = 4  # Maximum level of auto-repeat speed increases
_AUTO_REPEAT_LEVEL_MAX = 3  # Maximum level of auto-repeat digit increases
DEFAULT_BACKGROUND_UPDATE_PERIOD = 1000    # mS when not moving

# App states
STATE_HEXPANSION = -1     # Hexpansion Management (sub-states managed by HexpansionMgr)
STATE_MENU = 0
STATE_MESSAGE = 1         # Message display
STATE_LOGO =  2           # Logo display
STATE_COUNTDOWN = 3       # Shared countdown (Motor Moves & PID AutoTune)
STATE_SETTINGS = 4        # Edit Settings
STATE_MOTOR_MOVES = 5     # Motor Moves (sub-states managed by MotorMovesMgr)
STATE_SERVO = 6           # Servo test
STATE_STEPPER = 7         # Stepper test
STATE_FOLLOWER = 8        # Line Follower
STATE_AUTOTUNE = 9        # PID Auto Tune
STATE_SENSOR = 10         # Sensor Test
STATE_AUTO = 11           # Autonomous Drive

# App states where user can minimise app (Menu, Message, Logo)
MINIMISE_VALID_STATES = [STATE_MENU, STATE_MESSAGE, STATE_LOGO]
 
# App states where BadgeBot directly controls the badge LEDs (Motor Moves, Countdown, Message, Logo, Line Follower, AutoTune)
_LED_CONTROL_STATES    = [STATE_MOTOR_MOVES, STATE_COUNTDOWN, STATE_MESSAGE, STATE_LOGO, STATE_FOLLOWER, STATE_AUTOTUNE, STATE_AUTO]

#Misceallaneous Settings
_LOGGING = False
_IS_SIMULATOR = sys.platform != "esp32"  # True when running in the simulator, not on real badge hardware

# Main Menu Items
MAIN_MENU_ITEMS = ["Line Follower","Motor Moves", "Stepper Test", "Servo Test", "PID Auto Tune", "Sensor Test", "Auto Drive", "Settings", "About","Exit"]
MENU_ITEM_LINE_FOLLOWER = 0
MENU_ITEM_MOTOR_MOVES = 1
MENU_ITEM_STEPPER_TEST = 2
MENU_ITEM_SERVO_TEST = 3
MENU_ITEM_PID_AUTOTUNE = 4
MENU_ITEM_SENSOR_TEST = 5
MENU_ITEM_AUTO_DRIVE = 6
MENU_ITEM_SETTINGS = 7
MENU_ITEM_ABOUT = 8
MENU_ITEM_EXIT = 9


# Front face direction labels (0=BtnA corner between slots 6 & 1, each step = 30° CW)
_FRONT_FACE_DEFAULT = 0
_FRONT_FACE_LABELS = (
    "BtnA", "Slot 1", "BtnB", "Slot 2", "BtnC", "Slot 3",
    "BtnD", "Slot 4", "BtnE", "Slot 5", "BtnF", "Slot 6",
)
_FWD_DIR_DEFAULT = 0
_FWD_DIR_LABELS = ("Normal", "Reverse")


# Import sub-modules after constants are defined so they can safely
# `from .app import STATE_*` without circular-import timing issues.
from .hexpansion_mgr import HexpansionMgr, HexpansionType
from .settings_mgr import SettingsMgr, MySetting
from .motor_moves import MotorMovesMgr
from .servo_test import ServoTestMgr
from .stepper_test import StepperTestMgr
from .line_follow import  LineFollowMgr
from .autotune_mgr import AutotuneMgr
from .sensor_test import SensorTestMgr
from .autodrive import AutoDriveMgr

# Each module registers its own settings via init_settings()
from .hexpansion_mgr import init_settings as _hexpansion_init_settings
from .motor_moves import init_settings as _motor_moves_init_settings
from .servo_test import init_settings as _servo_test_init_settings
from .stepper_test import init_settings as _stepper_test_init_settings
from .line_follow import init_settings as _line_follow_init_settings
from .sensor_test import init_settings as _sensor_test_init_settings
from .autodrive import init_settings as _autodrive_init_settings


class BadgeBotApp(app.App):         # pylint: disable=no-member
    """Main application class for BadgeBot.  Manages overall state, user input, and delegates to functional area managers for specific features."""
    def __init__(self):
        super().__init__()

        # UI Button Controls
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self._auto_repeat_intervals = [ _AUTO_REPEAT_MS, _AUTO_REPEAT_MS//2, _AUTO_REPEAT_MS//4, _AUTO_REPEAT_MS//8, _AUTO_REPEAT_MS//16] # at the top end the loop is unlikley to cycle this fast
        self._auto_repeat: int = 0
        self._auto_repeat_count: int = 0
        self.auto_repeat_level: int = 0

        # UI Feature Controls
        self.refresh: bool = True
        self.rpm: int = 5                    # logo rotation speed in RPM
        self.animation_counter: int = 0
        self.pattern_status: bool = True     # True = Pattern Enabled, False = Pattern Disabled
        self.qr_code = _QR_CODE
        self.APP_VERSION = APP_VERSION
        self.b_msg: str = f"BadgeBot V{APP_VERSION}"
        self.t_msg: str = "RobotMad"
        self.notification: Notification = None
        self.message = []
        self.message_colours = []
        self.message_type = None
        self.current_menu: str = None
        self.menu: Menu = None
        self.scroll_mode_enabled: bool = False  # Whether pressing the "C" button can toggle scroll mode on/off, which allows the user to scroll through lines on the display.
        self.scroll_ignore_next_c_button: bool = False # Used to ignore the "C" button event that triggers scroll mode on, otherwise it would immediately toggle scroll mode off again
        self.is_scroll: bool = False        # Whether we are in scroll mode - this is displayed by a green border around the screen 
        self.scroll_offset: int = 0

        # Shared countdown
        self.run_countdown_elapsed_ms: int = 0
        self.countdown_next_state: int = None  # which state to go to after countdown

        # Settings - common settings first, then each module registers its own
        self.settings = {}
        self.settings['brightness']    = MySetting(self.settings, _BRIGHTNESS, 0.1, 1.0)
        self.settings['logging']       = MySetting(self.settings, _LOGGING, False, True)
        # Module-specific settings
        _motor_moves_init_settings(self.settings, MySetting)
        _servo_test_init_settings(self.settings, MySetting)
        _stepper_test_init_settings(self.settings, MySetting)
        _hexpansion_init_settings(self.settings, MySetting)
        _line_follow_init_settings(self.settings, MySetting)
        _sensor_test_init_settings(self.settings, MySetting)
        _autodrive_init_settings(self.settings, MySetting)
        # Direction settings
        self.settings['fwd_dir']       = MySetting(self.settings, _FWD_DIR_DEFAULT, 0, 1)
        self.settings['front_face']    = MySetting(self.settings, _FRONT_FACE_DEFAULT, 0, 11)

        self.edit_setting: int  = None
        self.edit_setting_value = None       
        self.update_settings()   

        # Check what version of the Badge s/w we are running on
        try:
            ver = parse_version(ota.get_version())
            if ver is not None:
                if self.settings['logging'].v:
                    print(f"BadgeSW V{ver}")
                # Potential to do things differently based on badge s/w version
                # e.g. if ver < [1, 9, 0]:
        except Exception: # pylint: disable=broad-exception-caught  
            pass

        # Hexpansion related
        self.HEXPANSION_TYPES = [HexpansionType(0xCBCB, "HexDrive", motors=2, servos=4, app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                 HexpansionType(0xCBCA, "HexDrive", motors=2,           sub_type="2 Motor", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                 HexpansionType(0xCBCC, "HexDrive", servos=4,           sub_type="4 Servo", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                 HexpansionType(0xCBCD, "HexDrive", motors=1, servos=2, sub_type="1 Mot 2 Srvo", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"),
                                 HexpansionType(0xCBCE, "HexDrive", steppers=1,         sub_type="Stepper", app_mpy_name="hexdrive.mpy", app_mpy_version=CURRENT_HEXDRIVE_APP_VERSION, app_name="HexDriveApp"), 
                                 HexpansionType(0xCBCF, "HexSense", sensors=2,          sub_type="2 Line Sensors")] # , app_mpy_name="hexsense.mpy", app_mpy_version=CURRENT_HEXSENSE_APP_VERSION, app_name="HexSenseApp")]  
        self.HEXDRIVE_HEXPANSION_INDEX = 0
        self.HEXSENSE_HEXPANSION_INDEX = 5
        self.hexpansion_update_required: bool = False # flag from async to main loop
        self.hexdrive_app = None
        self.hexsense_app = None
        self.line_sensors_hexpansion_config  = None            # Store the HexpansionConfig of the HexSense that is providing the line sensors
        self.time_since_last_update: int = 0

        # High-level motor controller (created when HexDrive is found)
        self.motor_controller = None

        # Functional area managers
        self._hexpansion_mgr   = HexpansionMgr(self)
        self._motor_moves_mgr  = MotorMovesMgr(self)
        self._servo_test_mgr   = ServoTestMgr(self)
        self._stepper_test_mgr = StepperTestMgr(self)
        self._settings_mgr     = SettingsMgr(self)
        self._line_follow_mgr  = LineFollowMgr(self)
        self._autotune_mgr     = AutotuneMgr(self, self._line_follow_mgr)
        self._sensor_test_mgr  = SensorTestMgr(self)
        self._autodrive_mgr    = AutoDriveMgr(self)

        # State → manager dispatch table (for efficient update/draw routing)
        self._state_update_dispatch = {
            STATE_HEXPANSION:  self._hexpansion_mgr.update,
            STATE_MOTOR_MOVES: self._motor_moves_mgr.update,
            STATE_FOLLOWER:    self._line_follow_mgr.update,
            STATE_AUTOTUNE:    self._autotune_mgr.update,
            STATE_SERVO:       self._servo_test_mgr.update,
            STATE_STEPPER:     self._stepper_test_mgr.update,
            STATE_SETTINGS:    self._settings_mgr.update,
            STATE_SENSOR:      self._sensor_test_mgr.update,
            STATE_AUTO:        self._autodrive_mgr.update,
        }
        self._state_draw_dispatch = {
            STATE_HEXPANSION:  self._hexpansion_mgr.draw,
            STATE_MOTOR_MOVES: self._motor_moves_mgr.draw,
            STATE_FOLLOWER:    self._line_follow_mgr.draw,
            STATE_AUTOTUNE:    self._autotune_mgr.draw,
            STATE_SERVO:       self._servo_test_mgr.draw,
            STATE_STEPPER:     self._stepper_test_mgr.draw,
            STATE_SETTINGS:    self._settings_mgr.draw,
            STATE_SENSOR:      self._sensor_test_mgr.draw,
            STATE_AUTO:        self._autodrive_mgr.draw,
        }
        self._BG_DISPATCH = {
            STATE_MOTOR_MOVES: self._motor_moves_mgr.background_update,
            STATE_FOLLOWER:    self._line_follow_mgr.background_update,
            STATE_AUTOTUNE:    self._autotune_mgr.background_update,
            STATE_AUTO:        self._autodrive_mgr.background_update,
        }

        # Motor Driver Hardware
        self.num_motors: int = 2         # Default assumed for a single HexDrive
        self.num_steppers: int = 1       # Default assumed for a single HexDrive

        # Line Sensors Hardware
        self.num_line_sensors: int = 0                         # initialised to 0 until we detect a HexSense Hexpansion and can set this based on the actual number of sensors it has 

        # Servo Hardware
        self.num_servos: int     = 4                           # Default assumed for a single HexDrive

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_HEXPANSION
        self.previous_state = self.current_state
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD   # mS

        # Countdown timer value 
        self.countdown_value: int = 0

        # Hexpansion event handlers registered directly by hexpansion_mgr
        self._hexpansion_mgr.register_events()

        # 
        eventbus.on_async(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self._lose_focus, self)

        # We start with focus on launch, without an event emmited
        # This version is compatible with the simulator
        asyncio.get_event_loop().create_task(self._gain_focus(RequestForegroundPushEvent(self)))  


    ### ASYNC EVENT HANDLERS ###

    @property
    def hexdrive_port(self):
        """Proxy to HexpansionMgr.hexdrive_port for convenience."""
        return self._hexpansion_mgr.hexdrive_port

    @property
    def sensor_test_mgr(self):
        """Public access to the SensorTestMgr, used by AutoDriveMgr to share the sensor manager."""
        return self._sensor_test_mgr

    async def _gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            if self.current_state in _LED_CONTROL_STATES:
                eventbus.emit(PatternDisable())
            if self.scroll_mode_enabled:
                eventbus.on_async(ButtonUpEvent, self.handle_button_up, self)


    async def _lose_focus(self, event: RequestForegroundPopEvent):
        if event.app is self:
            eventbus.emit(PatternEnable())
            self.pattern_status = True
            if self.scroll_mode_enabled:
                eventbus.remove(ButtonUpEvent, self.handle_button_up, self)            


    async def handle_button_up(self, event: ButtonUpEvent):
        if self.scroll_mode_enabled and event.button == BUTTONS["C"]:
            if self.scroll_ignore_next_c_button:
                self.scroll_ignore_next_c_button = False
                return
            # Toggle scroll mode on/off when "C" button is released
            self.scroll(not self.is_scroll)


    async def background_task(self):
        """Background task loop for handling time-based updates. This runs independently of the main update/draw loop 
           and is suitable for tasks that need to run at a consistent interval regardless of the current state or drawing performance."""
        last_time = time.ticks_ms()
        while True:
            cur_time = time.ticks_ms()
            delta_ticks = time.ticks_diff(cur_time, last_time)
            self.background_update(delta_ticks)
            await asyncio.sleep_ms(self.update_period)
            last_time = cur_time


    ### NON-ASYNC FUNCTIONS ###

    def background_update(self, delta: int):
        """Background update function that is called at a regular interval from the background task loop.
           It dispatches to the appropriate manager based on the current state, and if motor outputs are returned, it sends them to the HexDrive app."""
        bg_fn = self._BG_DISPATCH.get(self.current_state)
        if bg_fn is not None:
            output = bg_fn(delta)
            if output is not None and self.hexdrive_app is not None:
                self.hexdrive_app.set_motors(self.apply_fwd_dir(output))
            #else:
            #    if self.settings['logging'].v:
            #        print(f"No motor output from background function for state {self.current_state}")    
        #else:
        #    if self.settings['logging'].v:
        #        print(f"Error: No background function found for state {self.current_state}")

# Manual Override                       
#        else:
#            if app.override == BUTTON_TYPES["UP"]:
#                output = (app.settings['max_power'].v, app.settings['max_power'].v)
#            elif app.override == BUTTON_TYPES["DOWN"]:
#                output = (-app.settings['max_power'].v, -app.settings['max_power'].v)
#            elif app.override == BUTTON_TYPES["LEFT"]:
#                output = (-app.settings['max_power'].v, app.settings['max_power'].v)
#            elif app.override == BUTTON_TYPES["RIGHT"]:
#                output = (app.settings['max_power'].v, -app.settings['max_power'].v)


    def update_settings(self):
        """Update settings from EEPROM and apply any necessary changes to the app's behaviour or appearance."""
        for s in self.settings:
            self.settings[s].v = settings.get(f"badgebot.{s}", self.settings[s].d)


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
        if self.notification:
            self.notification.update(delta)

        # manage LED PatternEnable/Disable for all states
        #self._pattern_management()

        # Update Hexpansion management if something 'hexpansion' related has changed
        if self.hexpansion_update_required:
            if self.current_state != STATE_HEXPANSION:
                # Trigger an update cycle for hexpansion_mgr even though it is not currently active
                self._hexpansion_mgr.update(delta)

        # Update the main application state (menus, countdowns, and delegating to functional area managers)
        self._update_main_application(delta)

        if self.current_state != self.previous_state:
            if self.settings['logging'].v:
                print(f"State: {self.previous_state} -> {self.current_state}")
            self.previous_state = self.current_state
            # manage LED PatternEnable/Disable for all states
            self._pattern_management()
            # something has changed - so worth redrawing
            self.refresh = True

        if self.current_state in _LED_CONTROL_STATES:
            if self.current_state in [STATE_FOLLOWER, STATE_AUTOTUNE]:
                # For Line Follower and AutoTune, set LEDs based on the line sensor readings
                # could be optimised to only update LEDs when sensor readings change, rather than every update cycle
                # nothing while we try to optimise the sensor reading rate
                pass
            else:
                if self.settings['brightness'].v < 1.0:
                    # Scale brightness
                    for i in range(1,13):
                        tildagonos.leds[i] = tuple(int(j * self.settings['brightness'].v) for j in tildagonos.leds[i])                            
                try:
                    # saw this crash randomly - hence protected by try/except to prevent whole app crashing, and added logging to investigate further
                    tildagonos.leds.write()
                except OSError as e:
                    if self.settings['logging'].v:
                        print(f"Error writing to LEDs: {e}")


    def _update_main_application(self, delta: int):
        if self.current_state == STATE_MENU:
            if self.current_menu is None:
                self.set_menu("main")
                self.refresh = True
            else:
                self.menu.update(delta)    
                if self.menu.is_animating != "none":
                    if self.settings['logging'].v:
                        print("Menu is animating")
                    self.refresh = True
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
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
            if self.message_type == "error":
                # Error has been acknowledged by the user
                self.button_states.clear()
                # Recheck Hexpansions - in case the issue is resolved
                self.current_state = STATE_HEXPANSION
            elif self.message_type == "warning" or self.current_state == STATE_LOGO:
                # Warning has been acknowledged by the user - allow access to the menu
                self.button_states.clear()
                self.current_state = STATE_MENU
            self.message = []
            self.message_colours = []
            self.message_type = None
        else:
            # "CANCEL" button is handled in common for all MINIMISE_VALID_STATES so no custom code here
            # Show the warning screen for 10 seconds
            self.animation_counter += delta
            self.refresh = True
            if self.message_type == "warning" and self.animation_counter > 10000:
                # For Warnings, after 10 seconds show the logo
                self.animation_counter = 0
                self.current_state = STATE_LOGO
                self.message = []
                self.message_colours = []
                self.message_type = None
            elif self.current_state == STATE_LOGO:
                # LED management - to match rotating logo:
                for i in range(1,13):
                    colour = (255, 241, 0)      # custom Robotmad shade of yellow                                
                    # raised cosine cubed wave
                    wave = self.settings['brightness'].v * pow((1.0 + cos(((i) *  pi / 1.5) - (self.rpm * self.animation_counter * pi / 7500)))/2.0, 3)    
                    # 4 sides each projecting a pattern of 3 LEDs (12 LEDs in total)
                    tildagonos.leds[i] = tuple(int(wave * j) for j in colour)                                                     
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
                self._motor_moves_mgr.begin_moves()
            elif self.countdown_next_state == STATE_AUTOTUNE:
                # PID AutoTune: start the tuner after countdown
                self.current_state = self.countdown_next_state
                self._autotune_mgr.begin_tuning()
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
            eventbus.on_async(ButtonUpEvent, self.handle_button_up, self)
        else:
            self.scroll_mode_enabled = False
            eventbus.remove(ButtonUpEvent, self.handle_button_up, self)


    def scroll(self, enable: bool):
        """Enable or disable scroll mode, which allows the user to scroll the display up and downto see hidden content. This is indicated by a green border around the screen."""
        self.is_scroll = enable
        self.scroll_offset = 0
        if self.scroll_mode_enabled:
            # only show notification about scroll mode if the feature is enabled, otherwise it would be confusing to show a notification about a feature that can't be used
            state = "enabled" if enable else "disabled"
            self.notification = Notification(f"    Scroll    {state}")


    def draw(self, ctx):
        """Main draw function called from the main loop. Handles drawing the current state, including any notifications."""
        if self.refresh or self.notification is not None:
            self.refresh = False
            clear_background(ctx)
            ctx.save()
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
            else:
                ctx.rgb(0,0,0).rectangle(-120,-120,240,240).fill()

            # Common states for messages and errors, which can be triggered by any functional area manager and are displayed in a consistent way
            if self.current_state == STATE_MESSAGE:
                if self.message_colours == []:
                    self.message_colours = [(1,0,0)]*len(self.message)
                self.draw_message(ctx, self.message, self.message_colours, label_font_size)
            elif self.current_state == STATE_COUNTDOWN:
                self.draw_message(ctx, [str(self.countdown_value)], [(1,1,0)], twentyfour_pt)
            else:
                # Delegate to functional area managers via dispatch table
                if self.current_state in self._state_draw_dispatch:
                    draw_fn = self._state_draw_dispatch.get(self.current_state)
                    if draw_fn is not None:
                        draw_fn(ctx)
            ctx.restore()

        # These need to be drawn every frame as they contain animations
        if self.current_state == STATE_MENU and self.menu is not None:
            clear_background(ctx)
            self.menu.draw(ctx)

        # Notifications are drawn on top of everything else, so that they are visible regardless of the current state. 
        # They also contain animations, so need to be drawn every frame when active.
        if self.notification:
            self.notification.draw(ctx)


    def clear_leds(self):
        """Utility function to clear all LEDs. This is used when setting direction LEDs to ensure only the relevant ones are lit."""
        for i in range(1,13):
            tildagonos.leds[i] = (0, 0, 0)

    # todo - merge with motor_controller.apply_fwd_dir if we keep the motor_controller abstraction
    def apply_fwd_dir(self, output: tuple) -> tuple:
        """Negate all motor outputs when fwd_dir=1 (HexDrive mounted facing front)."""
        if self.settings['fwd_dir'].v:
            return tuple(-v for v in output)
        return output


    def set_direction_leds(self, direction: Button):
        """LED positions rotate based on 'front_face' (0-11, each step = 30° CW).
        Each position p maps to LED pair: (p if p>0 else 12) and (p+1).
        This is independent of motor direction (fwd_dir)."""
        f = self.settings['front_face'].v
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


    def draw_message(self, ctx, message, colours, size=label_font_size):
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


    def return_to_menu(self):
        """Utility function to return to the main menu from any state. This is used when the user cancels out of a submenu or after acknowledging a warning message."""
        if self.settings['logging'].v:
            print("Returning to menu")
        self.update_period = DEFAULT_BACKGROUND_UPDATE_PERIOD
        self.current_state = STATE_MENU
        self.refresh = True


    def show_message(self, msg_content, msg_colours, msg_type = None):
        """Utility function to set the current state to the message display, and populate the message content and colours. The message_type can be used to indicate whether this is an 'error' (red) or 'warning' (green) message, which can affect both the display and the behaviour when the user acknowledges the message."""
        self.animation_counter = 0
        self.message = msg_content
        self.message_colours = msg_colours
        self.message_type = msg_type
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
                    if self.settings['logging'].v:
                        print(f"Auto Repeat Level: {self.auto_repeat_level}")

            return True
        return False


    def auto_repeat_clear(self):
        """Reset the auto-repeat counters and level. This should be called when a button is released to ensure that the next button press starts with the initial auto-repeat interval and level."""                
        self._auto_repeat = 1+ self._auto_repeat_intervals[0] # so that we trigger immediately on next press 

        self._auto_repeat_count = 0 
        self.auto_repeat_level = 0



### MENU FUNCTIONALITY ###


    def set_menu(self, menu_name = "main"):  #: Literal["main"]): does it work without the type hint?
        """Set the current menu to the specified menu name, and construct the menu if necessary. 
           If menu_name is None, it will clear the current menu and return to the previous state 
           (e.g. from a submenu back to the main menu)."""
        if self.settings['logging'].v:
            print(f"H:Set Menu {menu_name}")
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
            if self.num_servos == 0:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_SERVO_TEST])   
            if self.num_steppers == 0:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_STEPPER_TEST])   
            if self.num_motors == 0:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_MOVES])
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_LINE_FOLLOWER])
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_PID_AUTOTUNE])
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_AUTO_DRIVE])
            if self.num_line_sensors == 0:
                menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_LINE_FOLLOWER])
                if MAIN_MENU_ITEMS[MENU_ITEM_PID_AUTOTUNE] in menu_items:
                    menu_items.remove(MAIN_MENU_ITEMS[MENU_ITEM_PID_AUTOTUNE])
            self.menu = Menu(
                    self,
                    menu_items,
                    select_handler=self._main_menu_select_handler,
                    back_handler=self._menu_back_handler,
                )            
        elif menu_name == "Settings":
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
        if self.settings['logging'].v:
            print(f"H:Main Menu {item} at index {idx}")
        if   item == MAIN_MENU_ITEMS[MENU_ITEM_LINE_FOLLOWER]: # Line Follower
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                if self._line_follow_mgr.start():
                    self.current_state = STATE_FOLLOWER
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_MOTOR_MOVES]: # Motor Moves
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                if self._motor_moves_mgr.start():
                    self.current_state = STATE_MOTOR_MOVES
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_PID_AUTOTUNE]: # PID Auto Tune
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                if self._autotune_mgr.start():
                    self.current_state = STATE_AUTOTUNE
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_STEPPER_TEST]: # Stepper Test
            if self.num_steppers == 0:
                self.notification = Notification("No Steppers")
            else:
                if self._stepper_test_mgr.start():
                    self.current_state = STATE_STEPPER
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SERVO_TEST]: # Servo Test
            if self.num_servos == 0:
                self.notification = Notification("No Servos")
            else:
                if self._servo_test_mgr.start():
                    self.current_state = STATE_SERVO
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SENSOR_TEST]: # Sensor Test
            if self._sensor_test_mgr.start():
                self.current_state = STATE_SENSOR
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_AUTO_DRIVE]: # Auto Drive
            if self._autodrive_mgr.start():
                self.current_state = STATE_AUTO
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS]:   # Settings
            self.set_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_ABOUT]:      # About
            self.set_menu(None)
            self.button_states.clear()
            self.animation_counter = 0
            self.current_state = STATE_LOGO
            self.refresh = True   
        elif item == MAIN_MENU_ITEMS[MENU_ITEM_EXIT]:       # Exit
            self._hexpansion_mgr.unregister_events()
            eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
            eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
            eventbus.emit(RequestStopAppEvent(self))


    def _settings_menu_select_handler(self, item: str, idx: int):
        if self.settings['logging'].v:
            print(f"H:Setting {item} @ {idx}")
        if idx == 0: #Save
            if self.settings['logging'].v:
                print("H:Settings Save All")
            settings.save()
            self.notification = Notification("  Settings  Saved")
            self.set_menu("main")
        elif idx == 1: #Default
            if self.settings['logging'].v:
                print("H:Settings Default All")
            for s in self.settings:
                self.settings[s].v = self.settings[s].d
                self.settings[s].persist()
            self.notification = Notification("  Settings Defaulted")

            self.set_menu("main")
        else:
            self.set_menu(None)
            self.button_states.clear()
            self.current_state = STATE_SETTINGS
            self.refresh = True
            self.auto_repeat_clear()
            self.edit_setting = item
            self.edit_setting_value = self.settings[item].v


    def _menu_back_handler(self):
        if self.current_menu == "main":
            self.minimise()
        # There are only two menus so this is the only other option    
        self.set_menu("main")


__app_export__ = BadgeBotApp
