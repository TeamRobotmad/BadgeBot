import asyncio
import os
import time

import settings
import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size, twentyfour_pt
from events.input import BUTTON_TYPES, Button, Buttons, ButtonUpEvent
from frontboards.twentyfour import BUTTONS
from machine import I2C
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionMountedEvent,
                                      HexpansionRemovalEvent)
from system.hexpansion.header import HexpansionHeader
from system.hexpansion.util import get_hexpansion_block_devices
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler import scheduler
from system.scheduler.events import (RequestForegroundPopEvent,
                                     RequestForegroundPushEvent)
from tildagonos import tildagonos

import app

# Hard coded to talk to 16bit address EEPROM on address 0x50 - because we know that is what is on the HexDrive Hexpansion
# makes it a lot more efficient than scanning the I2C bus for devices and working out what they are

CURRENT_APP_VERSION = 2648 # Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating

# Motor Driver - Defaults
MAX_POWER = 65535
POWER_STEP_PER_TICK = 7500

# Screen positioning for text
VERTICAL_OFFSET = label_font_size
H_START = -78
V_START = -58

# Timings
TICK_MS       =  10 # Smallest unit of change for power, in ms
USER_DRIVE_MS =  50 # User specifed drive durations, in ms
USER_TURN_MS  =  20 # User specifed turn durations, in ms
LONG_PRESS_MS = 750 # Time for long button press to register, in ms
RUN_COUNTDOWN_MS = 5000 # Time after running program until drive starts, in ms

# App states
STATE_INIT = -1
STATE_WARNING = 0
STATE_MENU = 1
STATE_RECEIVE_INSTR = 2
STATE_COUNTDOWN = 3
STATE_RUN = 4
STATE_DONE = 5
STATE_WAIT = 6            # Between Hexpansion initialisation and upgrade steps  
STATE_DETECTED = 7        # Hexpansion ready for EEPROM initialisation
STATE_UPGRADE = 8         # Hexpansion ready for EEPROM upgrade
STATE_PROGRAMMING = 9     # Hexpansion EEPROM programming
STATE_REMOVED = 10        # Hexpansion removed
STATE_ERROR = 11          # Hexpansion error

# App states where user can minimise app
MINIMISE_VALID_STATES = [0, 1, 2, 5, 6, 7, 8, 10, 11]

POWER_ENABLE_PIN_INDEX = 0	# First LS pin used to enable the SMPSU
POWER_DETECT_PIN_INDEX = 1  # Second LS pin used to sense if the SMPSU has a source of power

# Hexdrive constants
EEPROM_ADDR  = 0x50
HEXDRIVE_VID = 0xCAFE
HEXDRIVE_PID = 0xCBCB

class BadgeBotApp(app.App):
    def __init__(self):
        super().__init__()
        self.button_states = Buttons(self)
        self.last_press: Button = BUTTON_TYPES["CANCEL"]
        self.long_press_delta = 0

        self.is_scroll = False
        self.scroll_offset = 0

        self.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
        
        self._settings = {}
        self._settings['acceleration'] = POWER_STEP_PER_TICK
        self._settings['max_power'] = MAX_POWER
        self._settings['drive_step_ms'] = USER_DRIVE_MS
        self._settings['turn_step_ms'] = USER_TURN_MS
        self.update_settings()   

        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])

        self.notification = None
        self.error_message = []
        self.we_have_focus = False

        # Hexpansion related
        self.hexdrive_seen = False
        self.detected_port = None
        self.upgrade_port = None
        self.ports_with_blank_eeprom = set()
        self.ports_with_hexdrive = set()
        self.ports_with_upgraded_hexdrive = set()
        self.hexdrive_app = None
        eventbus.on_async(HexpansionInsertionEvent, self.handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent, self.handle_hexpansion_removal, self)
        eventbus.on_async(HexpansionMountedEvent, self.handle_hexpansion_mounted, self)

        # Overall app state (controls what is displayed and what user inputs are accepted)
        self.current_state = STATE_INIT

        eventbus.on_async(RequestForegroundPushEvent, self.gain_focus, self)
        eventbus.on_async(RequestForegroundPopEvent, self.lose_focus, self)
        eventbus.on_async(ButtonUpEvent, self.handle_button_up, self)

        # We start with focus on launch, without an event emmited
        self.gain_focus(RequestForegroundPushEvent(self))
   
    ### ASYNC EVENT HANDLERS ###

    async def handle_hexpansion_mounted(self, event: HexpansionMountedEvent):
        # To learn if and when we get this event 
        print(f"H:Mounted {event.port} at {event.mountpoint}")

    async def handle_hexpansion_removal(self, event: HexpansionRemovalEvent):
        #await asyncio.sleep(1)
        if event.port in self.ports_with_blank_eeprom:
            print(f"H:EEPROM removed from port {event.port}")
            self.ports_with_blank_eeprom.remove(event.port)
        if event.port in self.ports_with_hexdrive:
            print(f"H:HexDrive removed from port {event.port}")
            self.ports_with_hexdrive.remove(event.port)
        if event.port in self.ports_with_upgraded_hexdrive:
            print(f"H:HexDrive removed from port {event.port}")
            self.ports_with_upgraded_hexdrive.remove(event.port)
        if self.current_state == STATE_DETECTED and event.port == self.detected_port:
            self.current_state = STATE_WAIT
        if self.current_state == STATE_UPGRADE and event.port == self.upgrade_port:
            self.current_state = STATE_WAIT

    async def handle_hexpansion_insertion(self, event: HexpansionInsertionEvent):
        #await asyncio.sleep(1)
        self.check_port_for_hexdrive(event.port)

    async def gain_focus(self, event: RequestForegroundPushEvent):
        if event.app is self:
            self.we_have_focus = True
            eventbus.emit(PatternDisable())
            self.clear_leds()

    async def lose_focus(self, event: RequestForegroundPopEvent):
        if event.app is self:
            self.we_have_focus = False
            eventbus.emit(PatternEnable())

    async def background_task(self):
        # Modiifed background task loop for shorter sleep time
        last_time = time.ticks_ms()
        while True:
            cur_time = time.ticks_ms()
            delta_ticks = time.ticks_diff(cur_time, last_time)
            self.background_update(delta_ticks)
             # If we want to be kind we could make this variable depending on app state
             # I.e on transition into run set this lower
            await asyncio.sleep(0.01)
            last_time = cur_time

    async def handle_button_up(self, event: ButtonUpEvent):
        if self.current_state == STATE_RECEIVE_INSTR and event.button == BUTTONS["C"]:
            self.is_scroll = not self.is_scroll
            self.notification = Notification(f"Scroll {"yes" if self.is_scroll else "no"}")

    def background_update(self, delta):
        if self.current_state == STATE_RUN:
            power = self.get_current_power_level(delta)
            if power is None:
                self.current_state = STATE_DONE
            else:
                self.hexdrive_app.set_pwm(power)


    def check_port_for_hexdrive(self, port):
        # avoiding use of read_hexpansion_header as this triggers a full i2c scan each time
        # we know the EEPROM address so we can just read the header directly
        if port not in range(1, 7):
            return
        try:
            i2c = I2C(port)
            i2c.writeto(EEPROM_ADDR, bytes([0,0]))  # Read header @ address 0                
            header_bytes = i2c.readfrom(EEPROM_ADDR, 32)
        except OSError:
            # no EEPROM on this port
            #print(f"H:No compatible EEPROM on port {port}")
            return
        try:
            header = HexpansionHeader.from_bytes(header_bytes)
        except RuntimeError as e:
            # not a valid header
            print(f"H:Found EEPROM on port {port}")
            self.ports_with_blank_eeprom.add(port)
            return
        if header.vid == HEXDRIVE_VID and header.pid == HEXDRIVE_PID:
            print(f"H:Found HexDrive on port {port}")
            self.ports_with_hexdrive.add(port)


    def get_app_version_in_eeprom(self, port, header, i2c, addr) -> int:
        try:
            _, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return 0
        version = 0
        already_mounted = False
        mountpoint = '/hexpansion_' + str(port)
        try:
            vfs.mount(partition, mountpoint, readonly=True)
            print(f"H:Mounted {partition} at {mountpoint}")
        except OSError as e:
            if e.args[0] == 1:
                already_mounted = True
            else:
                print(f"H:Error mounting: {e}")
        except Exception as e:
            print(f"H:Error mounting: {e}")
        print("H:Reading app.mpy")
        try:
            appfile = open(f"{mountpoint}/app.mpy", "rb")
            app = appfile.read()
            appfile.close()
        except OSError as e:
            if e.args[0] == 2:
                # file does not exist is not an error
                print(f"H:No app.mpy found")
            else:    
                print(f"H:Error reading HexDrive app.mpy: {e}")
        except Exception as e:
            print(f"H:Error reading HexDrive app.mpy: {e}")            
        try:
            #version = app.split("APP_VERSION = ")[1].split("\n")[0]
            # TODO - means of identifying the version number in the app.mpy file 
            # quick hack - lets use the length of the file as a version number
            version = len(app)
        except Exception as e:
            version = 0
            pass                 
        if not already_mounted:
            print(f"H:Unmounting {mountpoint}")                    
            try:
                vfs.umount(mountpoint)
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
        print(f"H:HexDrive app.mpy version:{version}")
        return int(version)


    def update_app_in_eeprom(self, port, header, i2c, addr) -> bool:
        # Copy hexdreive.py to EEPROM as app.mpy
        print(f"H:Updating HexDrive app.mpy on port {port}")
        try:
            _, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False              
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
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
        source_path = f"/" + __file__.rsplit("/", 1)[0] + f"/hexdrive.mpy"
        dest_path   = f"{mountpoint}/app.mpy"
        try:
            # delete the existing app.mpy file
            print(f"H:Deleting {dest_path}")
            os.remove(f"{mountpoint}/app.py")
            os.remove(dest_path)
        except Exception as e:
            # ignore errors which will happen if the file does not exist
            pass
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
                print(f"H:Unmounted {mountpoint}")                    
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
                return False 
        print(f"H:HexDrive app.mpy updated to version {CURRENT_APP_VERSION}")            
        return True
    



    def prepare_eeprom(self, port, i2c) -> bool:
        print(f"H:Initialising EEPROM on port {port}")
        header = HexpansionHeader(
            manifest_version="2024",
            fs_offset=32,
            eeprom_page_size=32,
            eeprom_total_size=64 * 1024 // 8,
            vid=HEXDRIVE_VID,
            pid=HEXDRIVE_PID,
            unique_id=0x0,
            friendly_name="HexDrive",
        )
        # Write and read back header
        # write_header is broken for our type of EEPROM so we do it manually
        #write_header(port, header, addr=addr, addr_len=addr_len, page_size=header.eeprom_page_size)
        try:
            i2c.writeto(EEPROM_ADDR, bytes([0, 0]) + header.to_bytes())
        except Exception as e:
            print(f"H:Error writing header: {e}")
            return False
        #header = read_hexpansion_header(i2c, EEPROM_ADDR, set_read_addr=True, addr_len=2)
        try:
            i2c.writeto(EEPROM_ADDR, bytes([0,0]))  # Read header @ address 0                
            header_bytes = i2c.readfrom(EEPROM_ADDR, 32)
        except Exception as e:
            print(f"H:Error reading header back: {e}")
            #return False
        try:
            header = HexpansionHeader.from_bytes(header_bytes)
        except RuntimeError as e:
            print(f"H:Error parsing header: {e}")
            #return False
        except Exception as e:
            print(f"H:Error parsing header: {e}")
            #return False
        try:
            # Get block devices
            _, partition = get_hexpansion_block_devices(i2c, header, EEPROM_ADDR)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False           
        try:
            # Format
            vfs.VfsLfs2.mkfs(partition)
            print(f"H:EEPROM formatted")
        except Exception as e:
            print(f"H:Error formatting: {e}")
            return False
        try:
            # And mount!
            mountpoint = '/hexpansion_' + str(port)
            vfs.mount(partition, mountpoint, readonly=False)
            print(f"H:EEPROM initialised")
        except Exception as e:
            print(f"H:Error mounting: {e}")
            return False
        return True 


    # Scan the Hexpansion ports for EEPROMs and HexDrives in case they are already plugged in when we start
    def scan_ports(self):
        for port in range(1, 7):
            self.check_port_for_hexdrive(port)


    def update_settings(self):
        # use latest settings
        print("Updating settings:")
        settings.load()
        self._settings['acceleration']  = settings.get("badgebot_acceleration",  POWER_STEP_PER_TICK)
        self._settings['max_power']     = settings.get("badgebot_max_power",     MAX_POWER)
        self._settings['drive_step_ms'] = settings.get("badgebot_drive_step_ms", USER_DRIVE_MS)
        self._settings['turn_step_ms']  = settings.get("badgebot_turn_step_ms",  USER_TURN_MS)
        if (self._settings['acceleration'] != POWER_STEP_PER_TICK):
            print(f"Power step per tick: {self._settings['acceleration']}")
        if (self._settings['max_power'] != MAX_POWER):
            print(f"Max power: {self._settings['max_power']}")
        if (self._settings['drive_step_ms'] != USER_DRIVE_MS):  
            print(f"Drive step ms: {self._settings['drive_step_ms']}")
        if (self._settings['turn_step_ms'] != USER_TURN_MS):
            print(f"Turn step ms: {self._settings['turn_step_ms']}")    


    def update(self, delta):
        self.clear_leds()
        if self.notification:
            self.notification.update(delta)

        if self.current_state == STATE_INIT:
            # One Time initialisation
            eventbus.emit(PatternDisable())
            self.scan_ports()
            if (len(self.ports_with_hexdrive) == 0) and (len(self.ports_with_blank_eeprom) == 0):
                # There are currently no possible HexDrives plugged in
                self.current_state = STATE_WARNING
            else:
                self.current_state = STATE_WAIT
            return    
        elif self.current_state == STATE_ERROR or self.current_state == STATE_REMOVED: 
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]) or self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.button_states.clear()
                self.current_state = STATE_WAIT
                self.error_message = []
        elif self.current_state in MINIMISE_VALID_STATES:
            if self.current_state == STATE_DETECTED:
                # EEPROM initialisation        
                if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                    self.button_states.clear()
                    self.current_state = STATE_PROGRAMMING
                    if self.prepare_eeprom(self.detected_port, I2C(self.detected_port)):
                        self.notification = Notification("Initialised", port = self.detected_port)
                        self.ports_with_hexdrive.add(self.detected_port)
                        self.current_state = STATE_WAIT
                    else:
                        self.notification = Notification("Failed", port = self.detected_port)
                        self.error_message = ["EEPROM","Initialisation","Failed"]
                        self.current_state = STATE_ERROR          
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print("H:Cancelled")
                    self.button_states.clear()
                    self.current_state = STATE_WAIT
                return           
            elif self.current_state == STATE_UPGRADE:
                # EEPROM programming with latest App.py                
                if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                    self.button_states.clear()
                    self.current_state = STATE_PROGRAMMING
                    try:
                        i2c = I2C(self.upgrade_port)
                        i2c.writeto(EEPROM_ADDR, bytes([0,0]))  # Read header @ address 0                
                        header_bytes = i2c.readfrom(EEPROM_ADDR, 32)
                        header = HexpansionHeader.from_bytes(header_bytes)
                    except:           
                        header = False                        
                    if header and header.vid == HEXDRIVE_VID and header.pid == HEXDRIVE_PID:
                        if self.update_app_in_eeprom(self.upgrade_port, header, i2c, EEPROM_ADDR):
                            self.notification = Notification("Upgraded", port = self.upgrade_port)
                            self.ports_with_upgraded_hexdrive.add(self.upgrade_port)
                            self.error_message = ["Upgraded","Please","Reboop"]
                            self.current_state = STATE_ERROR                                     
                        else:
                            self.notification = Notification("Failed", port = self.upgrade_port)
                            self.error_message = ["HexDrive","Programming","Failed"]
                            self.current_state = STATE_ERROR
                    else:
                        self.error_message = ["HexDrive","Read","Failed"]
                        self.current_state = STATE_ERROR
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print("H:Cancelled")
                    self.button_states.clear()
                    self.current_state = STATE_WAIT
                return
            elif 0 < len(self.ports_with_blank_eeprom):
                # if there are any ports with blank eeproms
                # Show the UI prompt and wait for button press
                self.detected_port = self.ports_with_blank_eeprom.pop()
                self.notification = Notification("Initialise?", port = self.detected_port)
                self.current_state = STATE_DETECTED          
            elif 0 < len(self.ports_with_hexdrive):
                # if there are any ports with HexDrives - check if they need upgrading
                port = self.ports_with_hexdrive.pop()
                try:
                    i2c = I2C(port)
                    i2c.writeto(EEPROM_ADDR, bytes([0,0]))  # Read header @ address 0                
                    header_bytes = i2c.readfrom(EEPROM_ADDR, 32)
                    header = HexpansionHeader.from_bytes(header_bytes)
                except:           
                    header = False                  
                if header and header.vid == 0xCAFE and header.pid == HEXDRIVE_PID:
                    print(f"H:HexDrive on port {port}")
                    if self.get_app_version_in_eeprom(port, header, i2c, EEPROM_ADDR) == CURRENT_APP_VERSION:
                        print(f"H:HexDrive on port {port} has latest App")
                        self.ports_with_upgraded_hexdrive.add(port)
                        self.current_state = STATE_WAIT
                    else:    
                        # Show the UI prompt and wait for button press
                        self.upgrade_port = port
                        self.notification = Notification("Upgrade?", port = self.upgrade_port)
                        self.current_state = STATE_UPGRADE
                else:
                    print(f"H:Error reading Hexpansion header")
                    self.notification = Notification("Error", port = port)
                    self.error_message = ["Hexpansion","Read","Failed"]
                    self.current_state = STATE_ERROR        
            elif self.current_state == STATE_WAIT: 
                if 0 < len(self.ports_with_upgraded_hexdrive):
                    valid_port = next(iter(self.ports_with_upgraded_hexdrive))
                    # We have at least one HexDrive with the latest App.py
                    self.hexdrive_seen = True
                    # Find our running hexdrive app
                    for app in scheduler.apps:
                        if hasattr(app, "config") and app.config.port == valid_port:
                            self.hexdrive_app = app
                            print(f"H:Found app on port {valid_port}")
                            if self.hexdrive_app.get_status():
                                print(f"H:HexDrive [{valid_port}] OK")
                                self.current_state = STATE_MENU
                            else:
                                print(f"H:HexDrive {valid_port}: Failed to initialise PWM resources")
                                self.error_message = ["HexDrive {valid_port}","PWM Init","Failed","Please","Reboop"]
                                self.current_state = STATE_ERROR
                            break
                    else:
                        print(f"H:HexDrive {valid_port}: App not found, please restart")
                        self.error_message = ["HexDrive {vaild_port}","App not found","Please","Reboop"]
                        self.current_state = STATE_ERROR                           
                elif self.hexdrive_seen:
                    self.hexdrive_seen = False
                    self.current_state = STATE_REMOVED
                else:                   
                    self.current_state = STATE_WARNING


        if self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES and self.current_state != STATE_DONE:
            self.button_states.clear()
            self.minimise()
        elif self.current_state == STATE_MENU:
            # Exit start menu
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = STATE_RECEIVE_INSTR
                self.button_states.clear()
        elif self.current_state == STATE_WARNING:
            # Exit warning screen
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = STATE_MENU
                self.button_states.clear()
        elif self.current_state == STATE_RECEIVE_INSTR:
            # Enable/disable scrolling and check for long press
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.long_press_delta += delta
                if self.long_press_delta >= LONG_PRESS_MS:
                    self.finalize_instruction()
                    self.current_state = STATE_COUNTDOWN
            else:
                # Confirm is not pressed. Reset long_press state
                self.long_press_delta = 0
                # Manage scrolling
                if self.is_scroll:
                    if self.button_states.get(BUTTON_TYPES["DOWN"]):
                        self.scroll_offset -= 1
                    elif self.button_states.get(BUTTON_TYPES["UP"]):
                        self.scroll_offset += 1
                    self.button_states.clear()
                # Instruction button presses
                elif self.button_states.get(BUTTON_TYPES["RIGHT"]):
                    self._handle_instruction_press(BUTTON_TYPES["RIGHT"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["LEFT"]):
                    self._handle_instruction_press(BUTTON_TYPES["LEFT"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["UP"]):
                    self._handle_instruction_press(BUTTON_TYPES["UP"])
                    self.button_states.clear()
                elif self.button_states.get(BUTTON_TYPES["DOWN"]):
                    self._handle_instruction_press(BUTTON_TYPES["DOWN"])
                    self.button_states.clear()
            # LED management
            if self.last_press == BUTTON_TYPES["RIGHT"]:
                tildagonos.leds[2] = (255, 0, 0)
                tildagonos.leds[3] = (255, 0, 0)
            elif self.last_press == BUTTON_TYPES["LEFT"]:
                tildagonos.leds[8] = (0, 255, 0)
                tildagonos.leds[9] = (0, 255, 0)
            elif self.last_press == BUTTON_TYPES["UP"]:
                tildagonos.leds[12] = (0, 0, 255)
                tildagonos.leds[1] = (0, 0, 255)
            elif self.last_press == BUTTON_TYPES["DOWN"]:
                tildagonos.leds[6] = (255, 255, 0)
                tildagonos.leds[7] = (255, 255, 0)
            tildagonos.leds.write()

        elif self.current_state == STATE_COUNTDOWN:
            self.run_countdown_elapsed_ms += delta
            if self.run_countdown_elapsed_ms >= RUN_COUNTDOWN_MS:
                self.power_plan_iter = chain(*(instr.power_plan for instr in self.instructions))
                self.hexdrive_app.set_power(True)
                self.current_state = STATE_RUN

        elif self.current_state == STATE_RUN:
            # Run is primarily managed in the background update
            pass
                
        elif self.current_state == STATE_DONE:
            if self.button_states.get(BUTTON_TYPES["CANCEL"]):
                self.hexdrive_app.set_power(False)
                self.reset()
                self.button_states.clear()
            elif self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.hexdrive_app.set_power(False)
                self.run_countdown_elapsed_ms = 0
                self.current_power_duration = ((0,0,0,0), 0)
                self.current_state = STATE_COUNTDOWN                       
                self.button_states.clear()


    def _handle_instruction_press(self, press_type: Button):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type

    def reset(self):
        self.current_state = STATE_MENU
        self.last_press = BUTTON_TYPES["CONFIRM"]
        self.long_press_delta = 0
        self.is_scroll = False
        self.scroll_offset = 0
        self.run_countdown_elapsed_ms = 0
        self.instructions = []
        self.current_instruction = None
        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])


    def draw(self, ctx):
        ctx.save()
        ctx.font_size = label_font_size
        # Scroll mode indicator
        if self.is_scroll:
            ctx.rgb(0,0.2,0).rectangle(-120,-120,240,240).fill()
        else:
            ctx.rgb(0,0,0.2).rectangle(-120,-120,240,240).fill()

        if   self.current_state == STATE_WARNING:
            self.draw_message(ctx, ["BadgeBot requires","HexDrive hexpansion","from RobotMad","github.com/TeamRobotmad","/BadgeBot"], [(1,1,1),(1,1,0),(1,1,0),(1,1,1),(1,1,1)], label_font_size)
        elif self.current_state == STATE_REMOVED:
            self.draw_message(ctx, ["HexDrive","removed","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], label_font_size)      
        elif self.current_state == STATE_DETECTED:
            self.draw_message(ctx, ["Hexpansion","detected in",f"Slot {self.detected_port}","Init EEPROM","as HexDrive?"], [(1,1,1),(1,1,1),(0,0,1),(1,1,1),(1,1,0)], label_font_size)
        elif self.current_state == STATE_UPGRADE:
            self.draw_message(ctx, ["HexDrive","detected in",f"Slot {self.upgrade_port}","Upgrade","HexDrive App?"], [(1,1,0),(1,1,1),(0,0,1),(1,1,1),(1,1,1)], label_font_size)             
        elif self.current_state == STATE_PROGRAMMING:
            self.draw_message(ctx, ["HexDrive:","Programming","EEPROM","Please wait..."], [(1,1,0),(1,1,1),(1,1,1),(1,1,1)], label_font_size)            
        elif self.current_state == STATE_MENU:
            self.draw_message(ctx, ["BadgeBot","To Program:","Press C","When finished:","Long press C"], [(1,1,0),(1,1,1),(1,1,1),(1,1,1),(1,1,1)], label_font_size)
        elif self.current_state == STATE_ERROR:
            self.draw_message(ctx, self.error_message, [(1,0,0)]*len(self.error_message), label_font_size)
        elif self.current_state == STATE_RECEIVE_INSTR:
            for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
                ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET * (self.scroll_offset + i_num)).text(str(instr))
        elif self.current_state == STATE_COUNTDOWN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running in:")
            countdown_val = 1 + ((RUN_COUNTDOWN_MS - self.run_countdown_elapsed_ms) // 1000)
            self.draw_message(ctx, [str(countdown_val)], [(1,1,0)], twentyfour_pt)
            #ctx.rgb(1,1,0).move_to(H_START, V_START+VERTICAL_OFFSET).text(str(countdown_val))
        elif self.current_state == STATE_RUN:
            #ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running...")
            # convert current_power_duration to string, dividing all four values down by 655 (to get a value from 0-100)
            current_power, _ = self.current_power_duration
            power_str = str(tuple([x//655 for x in current_power]))
            #TODO - remember the directon to be shown: direction_str = str(self.current_instruction.press_type)
            #ctx.rgb(1,0,0).move_to(H_START-30, V_START + 2*VERTICAL_OFFSET).text(power_str)
            self.draw_message(ctx, ["Running...",power_str], [(1,1,1),(1,1,0)], label_font_size)
        elif self.current_state == STATE_DONE:
            self.draw_message(ctx, ["Program","Complete!","Replay:Press C","Restart:Press F"], [(0,1,0),(0,1,0),(1,1,0),(0,1,1)], label_font_size)
        if self.notification:
            self.notification.draw(ctx)
        ctx.restore()


    def draw_message(self, ctx, message, colours, size):
        ctx.font_size = size
        num_lines = len(message)
        for i_num, instr in enumerate(message):
            text_line = str(instr)
            width = ctx.text_width(text_line)
            try:
                colour = colours[i_num]
            except IndexError:
                colour = None
            if colour == None:
                colour = (1,1,1)
            ctx.rgb(*colour).move_to(-width//2, ctx.font_size*(i_num-((num_lines-1)/2))).text(text_line)


    def get_current_power_level(self, delta) -> int:
        # takes in delta as ms since last call
        # if delta was > 10... what to do
        if delta >= TICK_MS:
            delta = TICK_MS-1

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


    def clear_leds(self):
        for i in range(1,13):
            tildagonos.leds[i] = (0, 0, 0)
            tildagonos.leds.write()




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
        if self._press_type == BUTTON_TYPES["UP"]:
            return (power, 0, power, 0)
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (0, power, 0, power)
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (power, 0, 0, power)
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (0, power, power, 0)

    def directional_duration(self, mysettings):
        if self._press_type == BUTTON_TYPES["UP"]:
            return (mysettings['drive_step_ms'])
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (mysettings['drive_step_ms'])            
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (mysettings['turn_step_ms'])
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return (mysettings['turn_step_ms'])
        
    def make_power_plan(self, mysettings):
        # return collection of tuples of power and their duration
        curr_power = 0
        ramp_up = []
        for i in range(1*(self._duration+3)):
            ramp_up.append((self.directional_power_tuple(curr_power), TICK_MS))
            curr_power += mysettings['acceleration']
            if curr_power >= mysettings['max_power']:
                ramp_up.append((self.directional_power_tuple(mysettings['max_power']), TICK_MS))
                break
    
        user_power_duration = (self.directional_duration(mysettings) * self._duration)-(2*(i+1)*TICK_MS)
        
        print(f"Ramp up: {ramp_up}")
        power_durations = ramp_up.copy()
        if user_power_duration > 0:
            power_durations.append((self.directional_power_tuple(mysettings['max_power']), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        print("Power durations:")
        print(power_durations)
        self.power_plan = power_durations



def chain(*iterables):
    for iterable in iterables:
        yield from iterable

__app_export__ = BadgeBotApp
