import asyncio
import os
import time
from math import cos, pi
import ota
import settings
import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size, twentyfour_pt, clear_background, button_labels
from app_components import Menu
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from machine import I2C
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionRemovalEvent)
from system.hexpansion.header import HexpansionHeader
from system.hexpansion.util import get_hexpansion_block_devices
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler import scheduler
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent,
                                     RequestStopAppEvent)

from tildagonos import tildagonos

import app

from .utils import chain, draw_logo_animated, parse_version

# Hard coded to talk to EEPROMs on address 0x50 - because we know that is what is on the HexDrive Hexpansion
# makes it a lot more efficient than scanning the I2C bus for devices and working out what they are

CURRENT_APP_VERSION = 4 # HEXDRIVE.PY Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating

_APP_VERSION = "1.2" # BadgeBot App Version Number

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
STATE_SETTINGS = 17       # Edit Settings

# App states where user can minimise app
_MINIMISE_VALID_STATES = [0, 1, 7, 12, 13, 14, 15]
_LED_CONTROL_STATES    = [0, 3, 4, 5, 6, 12, 13, 14, 15]

# HexDrive Hexpansion constants
_EEPROM_ADDR  = 0x50
_EEPROM_NUM_ADDRESS_BYTES = 2
_EEPROM_PAGE_SIZE = 32
_EEPROM_TOTAL_SIZE = 64 * 1024 // 8


#Misceallaneous Settings
_LOGGING = False
_ERASE_SLOT = 0   # Slot for user to set if they want to erase EEPROMs on HexDrives

# 
_main_menu_items = ["Motor Moves", "Servo Test", "Settings", "About","Exit"]


class BadgeBotApp(app.App):
    def __init__(self):
        super().__init__()
        # UI Button Controls
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0
        self._auto_repeat_intervals = [ _AUTO_REPEAT_MS, _AUTO_REPEAT_MS//2, _AUTO_REPEAT_MS//4, _AUTO_REPEAT_MS//8, _AUTO_REPEAT_MS//16] # at the top end the loop is unlikley to cycle this fast
        self._auto_repeat = 0
        self._auto_repeat_count = 0
        self._auto_repeat_level = 0

        # UI Feature Controls
        self._refresh = True
        self.rpm = 5                    # logo rotation speed in RPM
        self._animation_counter = 0
        self._pattern_status = True     # True = Pattern Enabled, False = Pattern Disabled
        self.qr_code = _QR_CODE
        self.b_msg = f"BadgeBot V{_APP_VERSION}"
        self.t_msg = "RobotMad"
        self.is_scroll = False
        self.scroll_offset = 0
        self.notification = None
        self.error_message = []
        self.current_menu = None
        self.menu = None

        # BadgeBot Control Sequence Variables
        self.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
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

        self._edit_setting = None
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
        self._HEXDRIVE_TYPES = [HexDriveType(0xCBCB, motors=2, servos=4), 
                                HexDriveType(0xCBCA, motors=2, name="2 Motor"), 
                                HexDriveType(0xCBCC, servos=4, name="4 Servo"), 
                                HexDriveType(0xCBCD, motors=1, servos=2, name = "1 Mot 2 Srvo")]  
        self.hexpansion_type = None
        self.hexpansion_init_type = 0
        self.detected_port = None
        self.waiting_app_port = None
        self.erase_port = None
        self.upgrade_port = None
        self.hexdrive_port = None
        self.ports_with_blank_eeprom = set()
        self.ports_with_hexdrive = set()
        self.ports_with_latest_hexdrive = set()
        self.hexdrive_app = None
        self.hexpansion_update_required = False # flag from async to main loop
        eventbus.on_async(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)

        # Motor Driver
        self.num_motors   = 2       # Default assumed for a single HexDrive

        # Servo Tester
        self.num_servos   = 4       # Default assumed for a single HexDrive
        self.servo        = [None]*4                    # Servo Positions
        self.servo_centre = [_SERVO_DEFAULT_CENTRE]*4   # Trim Servo Centre
        self.servo_range  = [_SERVO_DEFAULT_RANGE]*4    # Limit Servo Range
        self.servo_rate   = [_SERVO_DEFAULT_RATE]*4     # Servo Rate of Change
        self.servo_mode   = [_SERVO_DEFAULT_MODE]*4     # Servo Mode [0:Position, 1: Scan]
        self.servo_selected = 0
        self._servo_modes = ['Off','Trim','Position','Scanning']

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_INIT
        self.previous_state = self.current_state


        eventbus.on_async(RequestForegroundPushEvent, self._gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self._lose_focus, self)
        #eventbus.on_async(ButtonUpEvent, self._handle_button_up, self)

        # We start with focus on launch, without an event emmited
        self._gain_focus(RequestForegroundPushEvent(self))
   


    ### ASYNC EVENT HANDLERS ###

    async def _handle_hexpansion_removal(self, event: HexpansionRemovalEvent):
        if event.port in self.ports_with_blank_eeprom:
            if self._settinfs['logging'].v:
                print(f"H:EEPROM removed from port {event.port}")
            self.ports_with_blank_eeprom.remove(event.port)
        if event.port in self.ports_with_hexdrive:
            if self._settings['logging'].v:
                print(f"H:HexDrive removed from port {event.port}")
            self.ports_with_hexdrive.remove(event.port)
        if event.port in self.ports_with_latest_hexdrive:
            if self._settings['logging'].v:
                print(f"H:HexDrive V{_APP_VERSION} removed from port {event.port}")
            self.ports_with_latest_hexdrive.remove(event.port)
        if self.current_state == STATE_DETECTED and event.port == self.detected_port:
            self.hexpansion_update_required = True
        elif self.current_state == STATE_UPGRADE and event.port == self.upgrade_port:
            self.hexpansion_update_required = True
        elif self.hexdrive_port is not None and event.port == self.hexdrive_port:
            self.hexpansion_update_required = True
        elif self.waiting_app_port is not None and event.port == self.waiting_app_port:
            self.hexpansion_update_required = True
        elif self.erase_port is not None and event.port == self.erase_port:
            self.hexpansion_update_required = True    


    async def _handle_hexpansion_insertion(self, event: HexpansionInsertionEvent):
        if self.check_port_for_hexdrive(event.port):
            self.hexpansion_update_required = True


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
            s = 10 if self.current_state == STATE_RUN else 50
            await asyncio.sleep_ms(s)
            last_time = cur_time


    ### NON-ASYNC FUCNTIONS ###

    def background_update(self, delta):
        if self.current_state == STATE_RUN:
            output = self.get_current_power_level(delta)
            if output is None:
                self.current_state = STATE_DONE
            elif self.hexdrive_app is not None:
                self.hexdrive_app.set_motors(output)


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


    ### HEXPANSION FUNCTIONS ###

    # Scan the Hexpansion ports for EEPROMs and HexDrives in case they are already plugged in when we start
    def scan_ports(self):
        for port in range(1, 7):
            self.check_port_for_hexdrive(port)


    def check_port_for_hexdrive(self, port) -> bool:
        # avoiding use of badge read_hexpansion_header as this triggers a full i2c scan each time
        # we know the EEPROM address so we can just read the header directly
        if port not in range(1, 7):
            return False
        try:
            #i2c = I2C(port)
            #i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
            #header_bytes = i2c.readfrom(_EEPROM_ADDR, 32)
            header_bytes = I2C(port).readfrom_mem(_EEPROM_ADDR, 0, 32, addrsize = (8*_EEPROM_NUM_ADDRESS_BYTES))
        except OSError:
            # no EEPROM on this port
            return False
        try:
            read_header = HexpansionHeader.from_bytes(header_bytes)
        except Exception:
            # not a valid header
            if self._settings['logging'].v:
                print(f"H:Found EEPROM on port {port}")
            self.ports_with_blank_eeprom.add(port)
            return True
        # check is this is a HexDrive header by scanning the _HEXDRIVE_TYPES list
        for hexpansion_type in self._HEXDRIVE_TYPES:
            if read_header.vid == hexpansion_type.vid and read_header.pid == hexpansion_type.pid:
                if self._settings['logging'].v:
                    print(f"H:Found '{hexpansion_type.name}' HexDrive on port {port}")
                self.ports_with_hexdrive.add(port)
                self.hexpansion_type = hexpansion_type
                self.num_motors = hexpansion_type.motors
                self.num_servos = hexpansion_type.servos
                return True
        # we are not interested in this type of hexpansion
        return False


    def update_app_in_eeprom(self, port, addr) -> bool:
        # Copy hexdrive.mpy to EEPROM as app.mpy
        if self._settings['logging'].v:
            print(f"H:Updating HexDrive app.mpy on port {port}")
        try:
            i2c = I2C(port)
        except Exception as e:
            print(f"H:Error opening I2C port {port}: {e}")
            return False
        header = self.read_hexpansion_header(i2c=i2c)
        if header is None:
            if self._settings['logging'].v:
                print(f"H:Error reading header on port {port}")
            return False
        try:
            _, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False              
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
            if self._settings['logging'].v:
                print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=False)
            except OSError as e:
                if e.args[0] == 1:
                    already_mounted = True
                else:
                    print(f"H:Error mounting: {e}")
            except Exception as e:
                print(f"H:Error mounting: {e}")
        source_path = "/" + __file__.rsplit("/", 1)[0] + "/hexdrive.mpy"
        dest_path   = f"{mountpoint}/app.mpy"
        try:
            # delete the existing app.mpy file
            if self._settings['logging'].v:
                print(f"H:Deleting {dest_path}")
            os.remove(dest_path)
        except Exception as e:
            if e.args[0] != 2:
                # ignore errors which will happen if the file does not exist
                print(f"H:Error deleting {dest_path}: {e}")
        if self._settings['logging'].v:
            print(f"H:Copying {source_path} to {dest_path}")

        try:
            appfile = open(dest_path, "wb")
        except Exception as e:
            print(f"H:Error opening {dest_path}: {e}")
            return False   
        try:        
            template = open(source_path, "rb")
        except Exception as e:
            print(f"H:Error opening {source_path}: {e}")
            return False   
        try:    
            appfile.write(template.read())                           
        except Exception as e:
            print(f"H:Error updating HexDrive: {e}")
            return False   
        try:
            appfile.close()
            template.close()     
        except Exception as e:
            print(f"H:Error closing files: {e}")
            return False
        if not already_mounted:
            try:
                vfs.umount(mountpoint)
                if self._settings['logging'].v:
                    print(f"H:Unmounted {mountpoint}")                    
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
                return False 
        if self._settings['logging'].v:
            print(f"H:HexDrive app.mpy updated to version {CURRENT_APP_VERSION}")            
        return True
    

    def prepare_eeprom(self, port, addr) -> bool:
        if self._settings['logging'].v:
            print(f"H:Initialising EEPROM on port {port}")
        hexdrive_header = HexpansionHeader(
            manifest_version="2024",
            fs_offset=32,
            eeprom_page_size=_EEPROM_PAGE_SIZE,
            eeprom_total_size=_EEPROM_TOTAL_SIZE,
            vid=self._HEXDRIVE_TYPES[self.hexpansion_init_type].vid,
            pid=self._HEXDRIVE_TYPES[self.hexpansion_init_type].pid,
            unique_id=0x0,
            friendly_name="HexDrive",
        )        
        # Write and read back header efficiently
        try:
            i2c = I2C(port)
            i2c.writeto(addr, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES) + hexdrive_header.to_bytes())
        except Exception as e:
            print(f"H:Error writing header: {e}")
            return False
        # Poll ACK
        while True:
            try:
                if i2c.writeto(addr, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES)):  # Poll ACK
                    break
            except OSError:
                pass
            finally:
                time.sleep_ms(1)
        try:
            #i2c.writeto(addr, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
            #header_bytes = i2c.readfrom(addr, 32)
            header_bytes = i2c.readfrom_mem(addr, 0, 32, addrsize = (8*_EEPROM_NUM_ADDRESS_BYTES))
        except Exception as e:
            print(f"H:Error reading header back: {e}")
            return False
        try:
            read_header = HexpansionHeader.from_bytes(header_bytes)
        except Exception as e:
            print(f"H:Error parsing header: {e}")
            return False
        try:
            # Get block devices
            _, partition = get_hexpansion_block_devices(i2c, read_header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False           
        try:
            # Format
            vfs.VfsLfs2.mkfs(partition)
            if self._settings['logging'].v:
                print("H:EEPROM formatted")
        except Exception as e:
            print(f"H:Error formatting: {e}")
            return False
        try:
            # And mount!
            mountpoint = '/hexpansion_' + str(port)
            vfs.mount(partition, mountpoint, readonly=False)
            if self._settings['logging'].v:
                print("H:EEPROM initialised")
        except OSError as e:
            if e.args[0] == 1:
                #already_mounted
                if self._settings['logging'].v:
                    print("H:EEPROM initialised")                
            else:
                print(f"H:Error mounting: {e}")
                return False
        except Exception as e:
            print(f"H:Error mounting: {e}")                
            return False
        return True 


    def erase_eeprom(self, port, addr) -> bool:
        if self._settings['logging'].v:
            print(f"H:Erasing EEPROM on port {port}")
        try:
            i2c = I2C(port)
            i2c.writeto(addr, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))
            # loop through all pages and erase them
            for page in range(_EEPROM_TOTAL_SIZE // _EEPROM_PAGE_SIZE):
                i2c.writeto(addr, bytes([page >> 8, page & 0xFF]) + bytes([0xFF]*_EEPROM_PAGE_SIZE))
                # check Ack
                while True:
                    try:
                        if i2c.writeto(addr, bytes([page >> 8, page & 0xFF])):  # Poll ACK
                            break
                    except OSError:
                        pass
                    finally:
                        time.sleep_ms(1)
        except Exception as e:
            print(f"H:Error erasing EEPROM: {e}")
            return False
        return True


    def read_hexpansion_header(self, i2c=None, port=None) -> HexpansionHeader:                
        try:
            if i2c is None:
                if port is None:
                    return None
                i2c = I2C(port)
            #i2c.writeto(_EEPROM_ADDR, bytes([0]*_EEPROM_NUM_ADDRESS_BYTES))  # Read header @ address 0                
            #header_bytes = i2c.readfrom(_EEPROM_ADDR, 32)
            header_bytes = i2c.readfrom_mem(_EEPROM_ADDR, 0, 32, addrsize = (8*_EEPROM_NUM_ADDRESS_BYTES))
            return HexpansionHeader.from_bytes(header_bytes)
        except OSError:     
            return None   


    def find_hexdrive_app(self, port) -> app:                    
        for an_app in scheduler.apps:
            if hasattr(an_app, "config") and hasattr(an_app.config, "port") and  an_app.config.port == port:
                return an_app
        return None


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

    def update(self, delta):
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

    def _update_hexpansion_management(self, delta):
        if self.current_state == STATE_INIT:
            # One Time initialisation
            self.scan_ports()
            if (len(self.ports_with_hexdrive) == 0) and (len(self.ports_with_blank_eeprom) == 0):
                # There are currently no possible HexDrives plugged in
                self._animation_counter = 0
                self.current_state = STATE_WARNING
            else:      
                self.current_state = STATE_CHECK
            return
        
        if self.hexpansion_update_required:
            # something has changed in the hexpansion ports            
            self.hexpansion_update_required = False
            if self.current_state != STATE_CHECK:
                print("H:Hexpansion Check")
                self.set_menu(None)
                self.current_state = STATE_CHECK
        
        if self.current_state == STATE_WARNING or self.current_state == STATE_LOGO:
            self._update_state_warning(delta)                    
        elif self.current_state == STATE_ERROR or self.current_state == STATE_MESSAGE or self.current_state == STATE_REMOVED: 
            self._update_state_error(delta)
        elif self.current_state == STATE_PROGRAMMING:
            # Programming the Hexpansion
            self._update_state_programming(delta)      
        elif self.current_state == STATE_DETECTED:
            # We have detected a Hexpansion with a blank EEPROM - asking the user if they want to initialise it
            self._update_state_detected(delta)
        elif self.current_state == STATE_ERASE:
            self._update_state_erase(delta)                      
        elif self.current_state == STATE_UPGRADE:
            # We are currently asking the user if they want hexpansion App upgrading with latest App.mpy
            self._update_state_upgrade(delta)
        elif self.current_state in _MINIMISE_VALID_STATES:                          
            if self._check_hexpansion_ports(delta):
                pass     
            elif self._check_hexdrive_ports(delta):
                pass
            elif self.current_state == STATE_CHECK:
                self._update_state_check(delta)


    def _update_main_application(self, delta):
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

    ### Motor Moves Application ###            
        elif self.current_state == STATE_HELP:
            self._update_state_help(delta)
        elif self.current_state == STATE_RECEIVE_INSTR:
            self._update_state_receive_instr(delta)
        elif self.current_state == STATE_COUNTDOWN:
            self._update_state_countdown(delta)
        elif self.current_state == STATE_RUN:
            self.clear_leds()
            # Run is primarily managed in the background update
        elif self.current_state == STATE_DONE:
            self._update_state_done(delta)

    ### Servo Tester Application ###
        elif self.current_state == STATE_SERVO:
            self._update_state_servo(delta)

    ### Settings Capability ###
        elif self.current_state == STATE_SETTINGS:
            self._update_state_settings(delta)
    ### End of Update ###


    def _update_state_warning(self, delta):
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


    def _update_state_error(self, delta):                
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            # Error has been acknowledged by the user
            self.button_states.clear()
            self.current_state = STATE_CHECK
            self.error_message = []
        else:
            for i in range(1,13):
                tildagonos.leds[i] = (0,255,0) if self.current_state == STATE_MESSAGE else (255,0,0)       


    def _update_state_programming(self, delta):        
        if self.upgrade_port is not None:
            if self.update_app_in_eeprom(self.upgrade_port, _EEPROM_ADDR):
                self.notification = Notification("Upgraded", port = self.upgrade_port)
                self.ports_with_latest_hexdrive.add(self.upgrade_port)
                self.error_message = ["Upgraded:","Please","reboop"]
                self.current_state = STATE_MESSAGE                                     
                if self._settings['logging'].v:
                    print(f"H:HexDrive on port {self.upgrade_port} upgraded please reboop")
            else:
                self.notification = Notification("Failed", port = self.upgrade_port)
                self.error_message = ["HexDrive","programming","failed"]
                self.current_state = STATE_ERROR
            self.upgrade_port = None
        elif self.detected_port is not None:
            if self.prepare_eeprom(self.detected_port, _EEPROM_ADDR):
                self.notification = Notification("Initialised", port = self.detected_port)
                self.upgrade_port = self.detected_port
                self.current_state = STATE_UPGRADE                      
            else:
                self.notification = Notification("Failed", port = self.detected_port)
                self.error_message = ["EEPROM","initialisation","failed"]
                self.current_state = STATE_ERROR
            self.detected_port = None
        elif self._settings['logging'].v:
            print("H:Error - no port to program")    


    def _update_state_detected(self, delta):            
        # We are currently asking the user if they want hexpansion EEPROM initialising
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            self.current_state = STATE_PROGRAMMING        
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            if self._settings['logging'].v:
                print("H:Initialise Cancelled")
            self.detected_port = None
            self.current_state = STATE_CHECK
        elif self.button_states.get(BUTTON_TYPES["UP"]):
            self.button_states.clear()
            self.hexpansion_init_type = (self.hexpansion_init_type + 1) % len(self._HEXDRIVE_TYPES)
            self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            self.button_states.clear()
            self.hexpansion_init_type = (self.hexpansion_init_type - 1) % len(self._HEXDRIVE_TYPES)
            self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["LEFT"]):
            self.button_states.clear()
            self.hexpansion_init_type = 1
            self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
            self.button_states.clear()
            self.hexpansion_init_type = 2
            self._refresh = True


    def _update_state_erase(self, delta):
        # We are currently asking the user if they want hexpansion EEPROM Erased                
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            # Yes
            self.button_states.clear()
            if self.erase_eeprom(self.erase_port, _EEPROM_ADDR):
                self.error_message = ["Erased:","Please","reboop"]
                self.notification = Notification("Erased", port = self.erase_port)
                self.erase_port = None
                self.current_state = STATE_MESSAGE                  
            else:
                self.notification = Notification("Failed", port = self.erase_port)
                self.error_message = ["EEPROM","erasure","failed"]
                self.current_state = STATE_ERROR                       
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
            # No
            if self._settings['logging'].v:
                print("H:Erase Cancelled")
            self.button_states.clear()
            self.erase_port = None
            self.current_state = STATE_CHECK  


    def _update_state_upgrade(self, delta):                
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            # Yes
            self.button_states.clear()
            self.notification = Notification("Upgrading", port = self.upgrade_port)
            self.current_state = STATE_PROGRAMMING
        elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
            # No
            if self._settings['logging'].v:
                print("H:Upgrade Cancelled")
            self.button_states.clear()
            self.upgrade_port = None
            self.current_state = STATE_CHECK


    def _update_state_check(self, delta):
        #print(f"Check: {self.ports_with_latest_hexdrive}")
        if 0 < len(self.ports_with_latest_hexdrive):
            # We have at least one HexDrive with the latest App.mpy
            if self.hexdrive_port is not None and self.hexdrive_port not in self.ports_with_latest_hexdrive:
                print(f"Check: {self.hexdrive_port} lost")
                self.hexdrive_port = None
                self.hexdrive_app = None
            if self.hexdrive_port is None:
                valid_port = next(iter(self.ports_with_latest_hexdrive))
                # Find our running hexdrive app
                hexdrive_app = self.find_hexdrive_app(valid_port)
                if hexdrive_app is not None:
                    self.hexdrive_port = valid_port
                    self.hexdrive_app = hexdrive_app
                    # only intended for use with a single active HexDrive at once at present
                    if self.hexdrive_app.get_status():
                        if self._settings['logging'].v:
                            print(f"H:HexDrive [{valid_port}] OK")
                        self.current_state = STATE_MENU
                        self._animation_counter = 0
                    else:
                        if self._settings['logging'].v:    
                            print(f"H:HexDrive {valid_port}: Failed to initialise PWM resources")
                        self.error_message = [f"HexDrive {valid_port}","PWM Init","Failed","Please","Reboop"]
                        self.current_state = STATE_ERROR
                else:
                    if self._settings['logging'].v:
                        print(f"H:HexDrive {valid_port}: App not found, please reboop")
                    self.error_message = [f"HexDrive {valid_port}","App not found.","Please","reboop"]
                    self.current_state = STATE_ERROR
            else:
                # Still have hexdrive on original port
                self.current_state = STATE_MENU        
        elif self.hexdrive_port is not None:
            print(f"Check: {self.hexdrive_port} lost")
            self.hexdrive_port = None
            self.hexdrive_app = None                      
            self.current_state = STATE_REMOVED
        else:
            self._animation_counter = 0                   
            self.current_state = STATE_WARNING


    def _check_hexpansion_ports(self, delta) -> bool:
        if 0 < len(self.ports_with_blank_eeprom):
            # if there are any ports with blank eeproms
            # Show the UI prompt and wait for button press
            self.detected_port = self.ports_with_blank_eeprom.pop()
            self.notification = Notification("Initialise?", port = self.detected_port)
            self.current_state = STATE_DETECTED
            return True     
        return False


    def _check_hexdrive_ports(self, delta) -> bool:
        #print(f"Check HexDrive Ports: {self.waiting_app_port} {self.ports_with_hexdrive}")   
        if self.waiting_app_port is not None or (0 < len(self.ports_with_hexdrive)):
            # if there are any ports with HexDrives - check if they need upgrading/erasing
            if self.waiting_app_port is None:
                self.waiting_app_port = self.ports_with_hexdrive.pop()
                self._animation_counter = 0  #timeout
            if self._settings['erase_slot'].v == self.waiting_app_port:
                # if the user has set a port to erase EEPROMs on
                # Show the UI prompt and wait for button press
                if self._settings['logging'].v:
                    print(f"H:HexDrive on port {self.waiting_app_port} Erase?")
                self.erase_port = self.waiting_app_port
                self.notification = Notification("Erase?", port = self.erase_port)
                self.current_state = STATE_ERASE
            else:                           
                hexdrive_app = self.find_hexdrive_app(self.waiting_app_port)
                # the scheduler is updated asynchronously from hexpansion insertion so we may not find the app immediately
                if hexdrive_app is not None:
                    try:
                        hexdrive_app_version = hexdrive_app.get_version()
                    except Exception as e:
                        hexdrive_app_version = 0
                        print(f"H:Error getting HexDrive app version - assume old: {e}")
                elif 5.0 < self._animation_counter:
                    if self._settings['logging'].v:
                        print("H:Timeout waiting for HexDrive app to be started - assume it needs upgrading")
                    hexdrive_app_version = 0
                else:
                    if 0 == self._animation_counter:
                        if self._settings['logging'].v:
                            print(f"H:No app found on port {self.waiting_app_port} - WAITING for app to appear in Scheduler")
                    self.notification = Notification("Checking...", port = self.waiting_app_port)                            
                    self._animation_counter += delta/1000
                    return True                    
                if hexdrive_app_version == CURRENT_APP_VERSION:    
                    if self._settings['logging'].v:
                        print(f"H:HexDrive on port {self.waiting_app_port} has latest App")
                    self.ports_with_latest_hexdrive.add(self.waiting_app_port)
                    self.current_state = STATE_CHECK
                else:    
                    # Show the UI prompt and wait for button press
                    if self._settings['logging'].v:
                        print(f"H:HexDrive on port {self.waiting_app_port} needs upgrading from version {hexdrive_app_version}")
                    self.upgrade_port = self.waiting_app_port
                    self.notification = Notification("Upgrade?", port = self.upgrade_port)
                    self.current_state = STATE_UPGRADE                             
            self.waiting_app_port = None
            self._animation_counter = 0
            return True
        return False


    def _update_state_help(self, delta):            
        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            self.current_state = STATE_MENU
        elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            self.is_scroll = True   # so that release of this button will CLEAR Scroll mode
            eventbus.on_async(ButtonUpEvent, self._handle_button_up, self)
            self.current_state = STATE_RECEIVE_INSTR
        else:            
            # Show the help for 10 seconds
            self._animation_counter += delta/1000
            if self._animation_counter > 10:
                # after 10 seconds show the logo
                self._animation_counter = 0
                self.current_state = STATE_LOGO


    def _update_state_receive_instr(self, delta):            
        # Enable/disable scrolling and check for long press
        if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.long_press_delta += delta
            if self.long_press_delta >= _LONG_PRESS_MS:
                # if there are no steps saved in the power plan then return to HELP, otherwise go to COUNTDOWN
                if self.power_plan_iter is None:
                    self.current_state = STATE_HELP
                else:                            
                    self.finalize_instruction()
                    self.current_state = STATE_COUNTDOWN
                self.is_scroll = False
                eventbus.remove(ButtonUpEvent, self._handle_button_up, self)            
        else:
            # Confirm is not pressed. Reset long_press state
            self.long_press_delta = 0
            if self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                self._animation_counter = 0
                self.is_scroll = False
                self.current_state = STATE_HELP
                eventbus.remove(ButtonUpEvent, self._handle_button_up, self)            
                return
            # Manage scrolling
            if self.is_scroll:
                if self.button_states.get(BUTTON_TYPES["DOWN"]):
                    self.button_states.clear()
                    self.scroll_offset -= 1
                    self._refresh = True                            
                elif self.button_states.get(BUTTON_TYPES["UP"]):
                    self.button_states.clear()
                    self.scroll_offset += 1
                    self._refresh = True
            # Instruction button presses
            elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
                self._handle_instruction_press(BUTTON_TYPES["RIGHT"])
                self.button_states.clear()
                self._set_direction_leds(BUTTON_TYPES["RIGHT"])              
                self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["LEFT"]):
                self._handle_instruction_press(BUTTON_TYPES["LEFT"])
                self.button_states.clear()
                self._set_direction_leds(BUTTON_TYPES["LEFT"])            
                self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["UP"]):
                self._handle_instruction_press(BUTTON_TYPES["UP"])
                self.button_states.clear()
                self._set_direction_leds(BUTTON_TYPES["UP"])               
                self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["DOWN"]):
                self._handle_instruction_press(BUTTON_TYPES["DOWN"])
                self.button_states.clear()
                self._set_direction_leds(BUTTON_TYPES["DOWN"])                 
                self._refresh = True
            else:
                self._set_direction_leds(self.last_press)


    def _set_direction_leds(self, direction):
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


    def _update_state_countdown(self, delta):            
        self.clear_leds()
        self.run_countdown_elapsed_ms += delta
        if self.run_countdown_elapsed_ms >= _RUN_COUNTDOWN_MS:
            self.power_plan_iter = chain(*(instr.power_plan for instr in self.instructions))
            if self.hexdrive_app is not None:
                self.hexdrive_app.set_power(True)
            self.current_state = STATE_RUN


    def _update_state_done(self, delta):
        if self.button_states.get(BUTTON_TYPES["CANCEL"]):
            self.button_states.clear()
            if self.hexdrive_app is not None:
                self.hexdrive_app.set_power(False)
            self.reset_robot()
        elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
            self.button_states.clear()
            if self.hexdrive_app is not None:
                self.hexdrive_app.set_power(False)
            self.run_countdown_elapsed_ms = 1   # avoid "6" appearing on screen at all
            self.current_power_duration = ((0,0,0,0), 0)
            self.current_state = STATE_COUNTDOWN


    def _update_state_servo(self, delta):            
        # Servo Tester:
        # Up/Down to select Servo
        # Left/Right to adjust position
        if self.button_states.get(BUTTON_TYPES["RIGHT"]):
            if self._auto_repeat_check(delta, not (self.servo_mode[self.servo_selected] == 3)):
                if self.servo_mode[self.servo_selected] == 1:    # Trim mode:
                    # adjust the servo centre position
                    self.servo_centre[self.servo_selected] += self._settings['servo_step'].v
                    if  self.servo_centre[self.servo_selected] > (_SERVO_DEFAULT_CENTRE + _SERVO_MAX_TRIM):
                        self.servo_centre[self.servo_selected] = _SERVO_DEFAULT_CENTRE + _SERVO_MAX_TRIM
                    if self.hexdrive_app is not None:
                        if not self.hexdrive_app.set_servocentre(self.servo_centre[self.servo_selected], self.servo_selected):
                            print("H:Failed to set servo centre")
                elif self.servo_mode[self.servo_selected] == 3: # Scanning Mode
                    # as the rate changes sign when it reaches the range, we must be careful to modify it in the correct direction
                    if self.servo_rate[self.servo_selected] < 0:
                        negative = True    
                        rate = -self.servo_rate[self.servo_selected]
                    else:
                        negative = False
                        rate = self.servo_rate[self.servo_selected]
                    rate = self._inc(rate, self._auto_repeat_level)
                    if _SERVO_MAX_RATE < rate:
                        rate = _SERVO_MAX_RATE
                    if negative:
                        self.servo_rate[self.servo_selected] = -rate
                    else:
                        self.servo_rate[self.servo_selected] = rate
                else:                                            # Position Mode
                    if  self.servo[self.servo_selected] is None:
                        self.servo[self.servo_selected] = 0
                    self.servo_mode[self.servo_selected] = 2    
                    self.servo[self.servo_selected] += self._settings['servo_step'].v
                if self.servo[self.servo_selected] is not None:
                    if self.servo_range[self.servo_selected] < (self.servo[self.servo_selected] + (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)):
                        self.servo[self.servo_selected] = self.servo_range[self.servo_selected] - (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)
                self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["LEFT"]):
            if self._auto_repeat_check(delta, not (self.servo_mode[self.servo_selected] == 3)):
                if self.servo_mode[self.servo_selected] == 1:    # Trim mode:
                    # adjust the servo centre position
                    self.servo_centre[self.servo_selected] -= self._settings['servo_step'].v
                    if  self.servo_centre[self.servo_selected] < (_SERVO_DEFAULT_CENTRE - _SERVO_MAX_TRIM):
                        self.servo_centre[self.servo_selected] = _SERVO_DEFAULT_CENTRE - _SERVO_MAX_TRIM
                    if self.hexdrive_app is not None:
                        if not self.hexdrive_app.set_servocentre(self.servo_centre[self.servo_selected], self.servo_selected):
                            print("H:Failed to set servo centre")
                elif self.servo_mode[self.servo_selected] == 3: # Scanning Mode
                    # as the rate changes sign when it reaches the range, we must be careful to modify it in the correct direction
                    if self.servo_rate[self.servo_selected] < 0:
                        negative = True    
                        rate = -self.servo_rate[self.servo_selected]
                    else:
                        negative = False
                        rate = self.servo_rate[self.servo_selected]
                    rate = self._dec(rate, self._auto_repeat_level)
                    if _SERVO_MIN_RATE > rate:
                        rate = _SERVO_MIN_RATE
                    if negative:
                        self.servo_rate[self.servo_selected] = -rate
                    else:
                        self.servo_rate[self.servo_selected] = rate
                else:                                           # Position Mode
                    if  self.servo[self.servo_selected] is None:
                        self.servo[self.servo_selected] = 0                        
                    self.servo_mode[self.servo_selected] = 2    
                    self.servo[self.servo_selected] -= self._settings['servo_step'].v
                if self.servo[self.servo_selected] is not None:
                    if -self.servo_range[self.servo_selected] > (self.servo[self.servo_selected] + (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)):
                        self.servo[self.servo_selected] = -self.servo_range[self.servo_selected] - (self.servo_centre[self.servo_selected] - _SERVO_DEFAULT_CENTRE)
                self._refresh = True
        else:
            self._auto_repeat_clear()    
            # non auto-repeating buttons
            if self.button_states.get(BUTTON_TYPES["UP"]):
                self.button_states.clear()
                self.servo_selected = (self.servo_selected - 1) % self.num_servos
                self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["DOWN"]):
                self.button_states.clear()
                self.servo_selected = (self.servo_selected + 1) % self.num_servos
                self._refresh = True
            elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                if self.hexdrive_app is not None:
                    self.hexdrive_app.set_power(False)
                    self.hexdrive_app.set_servoposition()   # All Off
                self.current_state = STATE_MENU
                return
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]): #Cycle Through Modes
                self.button_states.clear()
                self.servo_mode[self.servo_selected] = (self.servo_mode[self.servo_selected] + 1) % 4
                if self.servo_mode[self.servo_selected] == 0:
                    if self.hexdrive_app is not None:
                        self.hexdrive_app.set_servoposition(self.servo_selected, None)
                else:
                    self._refresh = True
                self.notification = Notification(f"  Servo {self.servo_selected}:\n {self._servo_modes[self.servo_mode[self.servo_selected]]}")

        for i in range(self.num_servos):
            if self.servo_mode[i] == 3:
                # for any servo set to Scan mode, update the position
                if self.servo[self.servo_selected] is None:
                    self.servo[self.servo_selected] = 0                        
                self.servo[i] = self.servo[i] + (10 * self.servo_rate[i] * delta / 1000)
                if self.servo_range[i] < (self.servo[i] + (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)):
                    # swap direction
                    self.servo_rate[i] = -self.servo_rate[i]
                    self.servo[i] = self.servo_range[i] - (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)
                elif -self.servo_range[i] > (self.servo[i] + (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)):
                    # swap direction
                    self.servo_rate[i] = -self.servo_rate[i]
                    self.servo[i] = -self.servo_range[i] - (self.servo_centre[i] - _SERVO_DEFAULT_CENTRE)
                self._refresh = True
            if self._refresh and self.hexdrive_app is not None and self.servo_mode[i] != 0 and self.servo[i] is not None:
                # scanning servo or the selected servo
                self.hexdrive_app.set_servoposition(i, int(self.servo[i]))


    def _update_state_settings(self, delta):    
        if self.button_states.get(BUTTON_TYPES["UP"]):
            if self._auto_repeat_check(delta, False):
                self._edit_setting_value = self._settings[self._edit_setting].inc(self._edit_setting_value, self._auto_repeat_level)
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} (+) Value: {self._edit_setting_value}")
                self._refresh = True
        elif self.button_states.get(BUTTON_TYPES["DOWN"]):
            if self._auto_repeat_check(delta, False):
                self._edit_setting_value = self._settings[self._edit_setting].dec(self._edit_setting_value, self._auto_repeat_level)  
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} (-) Value: {self._edit_setting_value}")
                self._refresh = True            
        else:
            # non auto-repeating buttons
            self._auto_repeat_clear()                           
            if self.button_states.get(BUTTON_TYPES["RIGHT"]) or self.button_states.get(BUTTON_TYPES["LEFT"]):
                self.button_states.clear() 
                # Force default value    
                self._edit_setting_value = self._settings[self._edit_setting].d
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} Default: {self._edit_setting_value}")
                self._refresh = True
                self.notification = Notification("Default")
            elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                # leave setting unchanged
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} Cancelled")
                self.set_menu(_main_menu_items[2])
                self.current_state = STATE_MENU
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.button_states.clear()
                # set setting
                if self._settings['logging'].v:
                    print(f"Setting: {self._edit_setting} = {self._edit_setting_value}")
                self._settings[self._edit_setting].v = self._edit_setting_value
                self._settings[self._edit_setting].persist()
                self.notification = Notification(f"  Setting:   {self._edit_setting}={self._edit_setting_value}")
                self.set_menu(_main_menu_items[2])
                self.current_state = STATE_MENU


    def draw(self, ctx):
        if self._refresh or self.notification is not None:
            self._refresh = False
            clear_background(ctx)   
            ctx.save()
            ctx.font_size = label_font_size
            ctx.text_align = ctx.LEFT
            ctx.text_baseline = ctx.BOTTOM            
            if self.current_state == STATE_LOGO:
                draw_logo_animated(ctx, self.rpm, self._animation_counter, [self.b_msg, self.t_msg], self.qr_code)
            # Scroll mode indicator
            elif self.is_scroll:
                ctx.rgb(0,0.2,0).rectangle(     -120,-120, 115+H_START,240).fill()
                ctx.rgb(0,0  ,0).rectangle(H_START-5,-120,10-2*H_START,240).fill()
                ctx.rgb(0,0.2,0).rectangle(5-H_START,-120, 115+H_START,240).fill()
            else:
                ctx.rgb(0,0,0).rectangle(-120,-120,240,240).fill()
            # Main screen content 
            if   self.current_state == STATE_WARNING:
                self.draw_message(ctx, ["BadgeBot requires","HexDrive hexpansion","from RobotMad","github.com","/TeamRobotmad","/BadgeBot"], [(1,1,1),(1,1,0),(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
            elif self.current_state == STATE_REMOVED:
                self.draw_message(ctx, ["HexDrive","removed.","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], label_font_size)      
            elif self.current_state == STATE_DETECTED:
                hexdrive_type = self._HEXDRIVE_TYPES[self.hexpansion_init_type].name
                self.draw_message(ctx, ["Hexpansion",f"in slot {self.detected_port}:","Init EEPROM as",hexdrive_type,"HexDrive?"], [(1,1,1),(1,1,1),(1,1,1),(0,0,1),(1,1,0)], label_font_size)
                button_labels(ctx, confirm_label="Yes", up_label="^", down_label="\u25BC", left_label=self._HEXDRIVE_TYPES[1].name, right_label=self._HEXDRIVE_TYPES[2].name,  cancel_label="No")
            elif self.current_state == STATE_ERASE:
                self.draw_message(ctx, ["HexDrive",f"in slot {self.erase_port}:","Erase EEPROM?"], [(1,1,0),(1,1,1),(1,0,0)], label_font_size)
                button_labels(ctx, confirm_label="Yes", cancel_label="No")
            elif self.current_state == STATE_UPGRADE:
                self.draw_message(ctx, ["HexDrive",f"in slot {self.upgrade_port}:","Upgrade","HexDrive app?"], [(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)             
                button_labels(ctx, confirm_label="Yes", cancel_label="No")
            elif self.current_state == STATE_PROGRAMMING:
                self.draw_message(ctx, ["HexDrive:","Programming","EEPROM","Please wait..."], [(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)            
            elif self.current_state == STATE_HELP:                
                self.draw_message(ctx, ["BadgeBot","To program:","Press C","When finished:","Long press C"], [(1,1,0),(1,1,1),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
            elif self.current_state == STATE_ERROR:
                self.draw_message(ctx, self.error_message, [(1,0,0)]*len(self.error_message), label_font_size)
            elif self.current_state == STATE_MESSAGE:
                self.draw_message(ctx, self.error_message, [(0,1,0)]*len(self.error_message), label_font_size)            
            elif self.current_state == STATE_RECEIVE_INSTR:
                self._draw_receive_instr(ctx)
                # button labels clash with the instruction list - so not shown
                #button_labels(ctx, confirm_label="Scroll", up_label="Fwd", down_label="Rev", left_label="Left", right_label="Right",  cancel_label="Cancel")
            elif self.current_state == STATE_COUNTDOWN:
                countdown_val = 1 + ((_RUN_COUNTDOWN_MS - self.run_countdown_elapsed_ms) // 1000)
                self.draw_message(ctx, [str(countdown_val)], [(1,1,0)], twentyfour_pt)
            elif self.current_state == STATE_RUN:
                # convert current_power_duration to string, dividing all four values down by 655 (to get a value from 0-100)
                current_power, _ = self.current_power_duration
                power_str = str(tuple([int(x/(self._settings['max_power'].v//100)) for x in current_power]))
                self.draw_message(ctx, ["Running...",power_str], [(1,1,1),(1,1,0)], label_font_size)
            elif self.current_state == STATE_DONE:
                #self.draw_message(ctx, ["Program","complete!","Replay:Press C","Restart:Press F"], [(0,1,0),(0,1,0),(1,1,0),(0,1,1)], label_font_size)
                self.draw_message(ctx, ["Program","complete!"], [(0,1,0),(0,1,0)], label_font_size)
                button_labels(ctx, confirm_label="Replay", cancel_label="Restart")
            elif self.current_state == STATE_SERVO:
                self._draw_state_servo(ctx)
            elif self.current_state == STATE_SETTINGS:
                self.draw_message(ctx, ["Edit Setting",f"{self._edit_setting}:",f"{self._edit_setting_value}"], [(1,1,1),(0,0,1),(0,1,0)], label_font_size)
                button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
            ctx.restore()

        # These need to be drawn every frame as they contain animations
        if self.current_state == STATE_MENU:
            clear_background(ctx)               
            self.menu.draw(ctx)

        if self.notification:
            self.notification.draw(ctx)


    def _draw_receive_instr(self, ctx):                
        # Display list of movements
        for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
            # map the instruction to a colour & change language from up/down to fwd/rev
            colour = (1,1,1)
            if instr is not None:
                direction = str(instr).split()[0]
                #if self._settings['logging'].v:
                #    print(direction)
                if   direction == "UP":
                    instr = "FWD " + str(instr).split()[1]
                    colour = (0,1,1)
                elif direction == "DOWN":
                    instr = "REV " + str(instr).split()[1]
                    colour = (1,0,1)
                elif direction == "LEFT":
                    colour = (1,0,0)
                elif direction == "RIGHT":
                    colour = (0,1,0)
                elif direction == "START" or direction == "END":
                    colour = (0.5,0.5,0.5)            
            ctx.rgb(*colour).move_to(H_START, V_START + label_font_size * (self.scroll_offset + i_num)).text(str(instr))


    def _draw_state_servo(self, ctx):                 
        servo_text         = ["S"]*(1+self.num_servos)              # Servo Text
        servo_text_colours = [(0.4,0.0,0.0)]*(1+self.num_servos)    # Red
        servo_text[0]      = "Servo Test"
        servo_text_colours[0] = (1,1,1)                       # Title - White
        for i in range(self.num_servos):

            # Select Colour according to mode
            if self.servo[i] is None or self.servo_mode[i] == 0:
                body_colour = (0.2,0.2,0.2)                    # Not activated - Grey
                bar_colour  = (0.4,0.4,0.4)                    # Not activated - Grey
            elif self.servo_mode[i] == 3:
                body_colour = (0.1,0.5,0.1)                    # Scanning - Green 
                bar_colour  = (0.1,1.0,0.1)                    # Scanning - Green
                servo_text_colours[1+i] = (0.4,0.0,0.4)        # Scanning - Magenta
            else:
                body_colour = (0.1,0.1,0.5)                    # Active - Blue                    
                bar_colour  = (0.1,0.1,1.0)                    # Active - Blue
                servo_text_colours[1+i] = (0.4,0.4,0.0)        # Active - Yellow                        

            # draw the servo positions
            ctx.save()
            # y = i-1.5 for 4 servos, y = i-0.5 for 2 servos
            ctx.translate(0, (i-(self.num_servos/2)+0.5) * label_font_size)
            # background for the servo position - grey
            background_colour = (0.1,0.1,0.1) if i != self.servo_selected else (0.15,0.15,0.15)                        
            ctx.rgb(*background_colour).rectangle(-100,1,200,label_font_size-2).fill() 
            c = 100 * (self.servo_centre[i]-_SERVO_DEFAULT_CENTRE) / self.servo_range[i]
            if self.servo[i] is not None:
                #TODO refactor this into a reusable function for drawing sliders
                # draw the servo position
                x = 100 * (self.servo[i] + self.servo_centre[i] - _SERVO_DEFAULT_CENTRE) / self.servo_range[i]

                # vertical bar at servo position
                ctx.rgb(*bar_colour).rectangle(x-2,1,5,label_font_size-2).fill()
                # horizontal bar from 0 to servo position, not covering the centre marker or the servo position bar
                ctx.rgb(*body_colour)                        
                if   x > (c+4):
                    ctx.rectangle(c+1, 3, x-c-4, label_font_size-6).fill()
                elif x < (c-4):
                    ctx.rectangle(x+4, 3, c-x-4, label_font_size-6).fill()
            # marker for the centre - black (drawn last as it may have to go through the servo position bar)
            ctx.rgb(0,0,0).move_to(c,0).line_to(c,label_font_size).stroke()                            
            ctx.restore()
            if self.servo_mode[i] == 3:
                servo_text[i+1] = f"{int(abs(self.servo_rate[i])):4}/s"   # Scanning Rate
            else:                                                   # Position
                servo_text[i+1] = "Off" if (self.servo[i] is None or self.servo_mode[i] == 0) else f"{int(self.servo[i]):+5} "
        # Selected Servo - Brighter Text
        servo_text_colours[1+self.servo_selected] = tuple(int(j * 2.5) for j in servo_text_colours[1+self.servo_selected])                            
        self.draw_message(ctx, servo_text, servo_text_colours, label_font_size)
        if self.servo_mode[self.servo_selected] == 3:
            # Scanning mode
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="Slower", right_label="Faster")
        elif self.servo_mode[self.servo_selected] == 1:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="Trim-", right_label="+Trim")
        else:
            #Position mode
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="<--", right_label="-->")
        # NB characters \u25B2, \u25C0, \u25BA, \u21A9, \u2611 do not exist, so ii seems \u25BC has been included as a special case


    # Value increment/decrement functions for positive integers only
    def _inc(self, v, l):
        if l==0:
            return v+1
        else:
            d = 10**l
            v = ((v // d) + 1) * d   # round up to the next multiple of 10^l
            return v
    
    def _dec(self, v, l):
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


    def reset_servo(self):
        # re-initialise the servo range for the servos
        if self.hexdrive_app is not None:
            self.hexdrive_app.set_power(True)
            self.hexdrive_app.set_freq(1000 // self._settings['servo_period'].v)
        # initialise the 4 servos
        for i in range(4):
            if self.hexdrive_app is not None:    # Apply Trim
                self.hexdrive_app.set_servocentre(self.servo_centre[self.servo_selected], self.servo_selected)                            

            # update the servo range in case settigns have changed
            self.servo_range[i] = self._settings['servo_range'].v     # only 1 setting actually for all servos at present
            # check that the current position is within the new range
            if self.servo[i] is not None:
                if self.servo[i] > self.servo_range[i]:
                    self.servo[i] = self.servo_range[i]
                elif self.servo[i] < -self.servo_range[i]:
                    self.servo[i] = -self.servo_range[i]
                # leave the servo positions etc... as they are. but turn them back on
                if self.hexdrive_app is not None:
                    self.hexdrive_app.set_servoposition(i, int(self.servo[i]))
            # leave the servo modes as they are
        self.servo_selected = 0


### MENU FUNCTIONALITY ###


    def set_menu(self, menu_name = "main"):  #: Literal["main"]): does it work without the type hint?
        if self._settings['logging'].v:
            print(f"H:Set Menu {menu_name}")
        if self.menu is not None:
            self.menu._cleanup()
        self.current_menu = menu_name
        if menu_name == "main":
            # construct the main menu based on template
            menu_items = _main_menu_items.copy()
            if self.num_servos == 0:
                menu_items.remove("Servo Test")   
            if self.num_motors == 0:
                menu_items.remove("Motor Moves")
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
    def _main_menu_select_handler(self, item, idx):
        if self._settings['logging'].v:
            print(f"H:Main Menu {item} at index {idx}")
        if   item == _main_menu_items[0]: # Motor Test - Turtle/Logo mode
            if self.num_motors == 0:
                self.notification = Notification("No Motors")
            elif self.num_motors == 1:
                self.notification = Notification(" 2 Motors  Required")
            else:
                self.set_menu(None)
                self.button_states.clear()
                self._animation_counter = 0
                self.current_state = STATE_HELP
                self._refresh = True
        elif item == _main_menu_items[1]: # Servo Test
            if self.num_servos == 0:
                self.notification = Notification("No Servos")
            else:
                self.set_menu(None)
                self.button_states.clear()
                self.reset_servo()
                self.current_state = STATE_SERVO
                self._refresh = True
                self._auto_repeat_clear()
        elif item == _main_menu_items[2]: # Settings
            self.set_menu(_main_menu_items[2])
        elif item == _main_menu_items[3]: # About
            self.set_menu(None)
            self.button_states.clear()
            self._animation_counter = 0
            self.current_state = STATE_LOGO
            self._refresh = True   
        elif item == _main_menu_items[4]: # Exit
            eventbus.remove(HexpansionInsertionEvent, self._handle_hexpansion_insertion, self)
            eventbus.remove(HexpansionRemovalEvent, self._handle_hexpansion_removal, self)
            eventbus.remove(RequestForegroundPushEvent, self._gain_focus, self)
            eventbus.remove(RequestForegroundPopEvent, self._lose_focus, self)
            eventbus.emit(RequestStopAppEvent(self))

    def _settings_menu_select_handler(self, item, idx):
        if self._settings['logging'].v:
            print(f"H:Setting {item} at index {idx}")
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

    def _handle_instruction_press(self, press_type: Button):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type


    # multi level auto repeat
    # if speed_up is True, the auto repeat gets faster the longer the button is held
    # otherwise it is a fixed rate, but the level is used to determine the scale of the increase in the setttings inc() and dec() functions
    def _auto_repeat_check(self, delta, speed_up = True) -> bool:                
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


    def reset_robot(self):
        self.current_state = STATE_HELP
        self.last_press = BUTTON_TYPES["CONFIRM"]
        self._animation_counter = 0
        self.long_press_delta = 0
        self.is_scroll = False
        self.scroll_offset = 0
        self.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])


    def get_current_power_level(self, delta) -> int:
        # takes in delta as ms since last call
        # if delta was > 10... what to do
        if delta >= _TICK_MS:
            delta = _TICK_MS-1

        current_power, current_duration = self.current_power_duration

        updated_duration = current_duration - delta
        if updated_duration <= 0:
            try:
                next_power, next_duration = next(self.power_plan_iter)
            except StopIteration:
                # returns None when complete
                return None
            next_duration += updated_duration
            self.current_power_duration = next_power, next_duration
            return next_power
        else:
            self.current_power_duration = current_power, updated_duration
            return current_power


    def finalize_instruction(self):
        if self.current_instruction is not None:
            self.current_instruction.make_power_plan(self._settings)
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None


class HexDriveType:
    def __init__(self, pid, vid=0xCAFE, motors=0, servos=0, name="Unknown"):
        self.vid = vid
        self.pid = pid
        self.name = name
        self.motors = motors
        self.servos = servos


class MySetting:
    def __init__(self, container, default, minimum, maximum):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum


    def __str__(self):
        return str(self.v)


    def _index(self):
        for k,v in self._container.items():
            if v == self:
                return k
        return None

        
    # This returns an increase in the value passed in - subject to max and with scale of increase depending on level
    # based on the type of the setting
    # it does not affect the current value of the setting
    def inc(self, v, l=0):            
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l==0:
                v += 1
            else:
                d = 10**l
                v = ((v // d) + 1) * d   # round up to the next multiple of 10^l, being very careful not to cause big jumps when value was nearly at the next multiple 

            if v > self._max:
                v = self._max
        elif isinstance(self.v, float):
            # only float at present is brightness from 0.0 to 1.0
            v += 0.1            
            if v > self._max:
                v = self._max  
        elif self._container['logging'].v:
            print(f"H:inc type: {type(self.v)}")                               
        return v

    # This returns a decrease in the value passed in - subject to min and with scale of increase depending on level
    # based on the type of the setting
    # it does not affect the current value of the setting
    def dec(self, v, l=0):            
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l==0:
                v -= 1
            else:
                d = 10**l
                v = (((v+(9*(10**(l-1)))) // d) - 1) * d   # round down to the next multiple of 10^l

            if v < self._min:
                v = self._min       
        elif isinstance(self.v, float):
            # only float at present is brightness from 0.0 to 1.0
            v -= 0.1            
            if v < self._min:
                v = self._min
        elif self._container['logging'].v:
            print(f"H: dec type: {type(self.v)}") 
        return v
    

    def persist(self):
        # only save non-default settings to the settings store
        try:
            if self.v != self.d:
                settings.set(f"badgebot.{self._index()}", self.v)
            else:
                settings.set(f"badgebot.{self._index()}", None)
        except Exception as e:
            print(f"H:Failed to persist setting {self._index()}: {e}")


class Instruction:
    def __init__(self, press_type: Button) -> None:
        self._press_type = press_type
        self._duration: int = 1
        self.power_plan = []


    @property
    def press_type(self) -> Button:
        return self._press_type


    def inc(self):
        self._duration += 1


    def __str__(self):
        return f"{self.press_type.name} {self._duration}"


    def directional_power_tuple(self, power):
        if   self._press_type == BUTTON_TYPES["UP"]:
            return ( power,  power)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (-power, -power)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (-power,  power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return ( power, -power)


    def directional_duration(self, mysettings):
        if   self._press_type == BUTTON_TYPES["UP"] or self._press_type == BUTTON_TYPES["DOWN"]:
            return (mysettings['drive_step_ms'].v)            
        elif self._press_type == BUTTON_TYPES["LEFT"] or self._press_type == BUTTON_TYPES["RIGHT"]:
            return (mysettings['turn_step_ms'].v)
        

    def make_power_plan(self, mysettings):
        # return collection of tuples of power and their duration
        curr_power = 0
        ramp_up = []
        for i in range(1*(self._duration+3)):
            ramp_up.append((self.directional_power_tuple(curr_power), _TICK_MS))
            curr_power += mysettings['acceleration'].v
            if curr_power >= mysettings['max_power'].v:
                ramp_up.append((self.directional_power_tuple(mysettings['max_power'].v), _TICK_MS))
                break
        user_power_duration = (self.directional_duration(mysettings) * self._duration)-(2*(i+1)*_TICK_MS)
        power_durations = ramp_up.copy()
        if user_power_duration > 0:
            power_durations.append((self.directional_power_tuple(mysettings['max_power'].v), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        if mysettings['logging'].v:
            print("Power durations:")
            print(power_durations)
        self.power_plan = power_durations


__app_export__ = BadgeBotApp
