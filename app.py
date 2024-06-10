import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size
from events.input import BUTTON_TYPES, Button, Buttons
from machine import I2C
from system.eventbus import eventbus
from system.hexpansion.events import (HexpansionInsertionEvent,
                                      HexpansionRemovalEvent)
from system.hexpansion.header import HexpansionHeader
from system.hexpansion.util import get_hexpansion_block_devices
from system.patterndisplay.events import PatternDisable, PatternEnable
from system.scheduler import scheduler
from tildagonos import tildagonos

import app

CURRENT_APP_VERSION = 2 # Integer Version Number - checked against the EEPROM app.py version to determine if it needs updating

# Motor Driver
MAX_POWER = 65535
POWER_STEP_PER_TICK = 5000

# Screen positioning for text
VERTICAL_OFFSET = label_font_size
H_START = -78
V_START = -58

# Timings
TICK_MS = 20 # Smallest unit of change for power, in ms
USER_DRIVE_MS = 100 # User specifed drive durations, in ms
LONG_PRESS_MS = 750 # Time for long button press to register, in ms
RUN_COUNTDOWN_MS = 3000 # Time after running program until drive starts, in ms

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

class Instruction:
    def __init__(self, press_type: Button) -> None:
        self._press_type = press_type
        self._duration = 1
        self.power_plan_iterator = iter([])

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

    def make_power_plan(self):
        # return collection of tuples of power and their duration
        ramp_up = [(self.directional_power_tuple(p), TICK_MS)
                   for p in range(0, MAX_POWER, POWER_STEP_PER_TICK)]
        power_durations = ramp_up.copy()
        user_power_duration = USER_DRIVE_MS * (self._duration-1)
        power_durations.append((self.directional_power_tuple(MAX_POWER), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        self.power_plan_iterator = iter(power_durations)


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

        self.current_power_duration = ((0,0,0,0), 0)
        self.power_plan_iter = iter([])

        self.notification = None
        self.error_message = []

        # Hexpansion related
        self.hexdrive_seen = False
        self.detected_port = None
        self.upgrade_port = None
        self.ports_with_blank_eeprom = set()
        self.ports_with_hexdrive = set()
        self.ports_with_upgraded_hexdrive = set()
        eventbus.on_async(HexpansionInsertionEvent, self.handle_hexpansion_insertion, self)
        eventbus.on_async(HexpansionRemovalEvent,   self.handle_hexpansion_removal,   self)

        self.hexdrive_app = None
    
        # Overall app state (controls what is displayed)
        self.current_state = STATE_INIT

    async def handle_hexpansion_removal(self, event):
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

    async def handle_hexpansion_insertion(self, event):
        #await asyncio.sleep(1)
        self.check_port_for_hexdrive(event.port)
    
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
            eep, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return 0
        version = 0
        already_mounted = False
        mountpoint = '/hexpansion_' + str(port)
        try:
            vfs.mount(partition, mountpoint, readonly=True)
            print(f"H:Mounted {partition} at {mountpoint}")
        except Exception as e:
            if e.args[0] == 1:
                already_mounted = True
            else:
                print(f"H:Error mounting: {e}")
        print("H:Reading app.py")
        try:
            with open(f"{mountpoint}/app.py", "rt") as appfile:
                app = appfile.read()
                version = app.split("APP_VERSION = ")[1].split("\n")[0]
                #matching = re.match(".*^app_version = (\d*).*", app, flags=re.MULTILINE+re.DOTALL)
                #if matching:
                #    version = int(matching.group(1))
            if not already_mounted:
                print(f"H:Unmounting {mountpoint}")                    
                vfs.umount(mountpoint)
            print(f"H:HexDrive app.py version is {version}")
            return int(version)
        except Exception as e:
            print(f"H:Error reading HexDrive app.py: {e}")
            return 0

    def update_app_in_eeprom(self, port, header, i2c, addr) -> bool:
        # Copy hexdreive.py to EEPROM as app.py
        print(f"H:Updating HexDrive app.py on port {port}")
        try:
            eep, partition = get_hexpansion_block_devices(i2c, header, addr)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False              
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
            print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=False)
            except Exception as e:
                if e.args[0] == 1:
                    already_mounted = True
                else:
                    print(f"H:Error mounting: {e}")
        try:
            path = "/" + __file__.rsplit("/", 1)[0] + "/hexdrive.py"
            print(f"H:Copying {path} to {mountpoint}/app.py")
            with open(f"{mountpoint}/app.py", "wt") as appfile:
                with open(path, "rt") as template:
                    appfile.write(template.read())
        except Exception as e:
            print(f"H:Error updating HexDrive app.py: {e}")
            return False   
        if not already_mounted:
            print(f"H:Unmounting {mountpoint}")                    
            try:
                vfs.umount(mountpoint)
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
                return False 
        print(f"H:HexDrive app.py updated to version {CURRENT_APP_VERSION}")            
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
        except OSError as e:
            print(f"H:Error writing header: {e}")
            return False
        #header = read_hexpansion_header(i2c, EEPROM_ADDR, set_read_addr=True, addr_len=2)
        try:
            i2c.writeto(EEPROM_ADDR, bytes([0,0]))  # Read header @ address 0                
            header_bytes = i2c.readfrom(EEPROM_ADDR, 32)
        except OSError as e:
            print(f"H:Error reading header back: {e}")
            return False
        try:
            header = HexpansionHeader.from_bytes(header_bytes)
        except RuntimeError as e:
            print(f"H:Error parsing header: {e}")
            return False
        try:
            # Get block devices
            eep, partition = get_hexpansion_block_devices(i2c, header, EEPROM_ADDR)
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


    def update(self, delta):
        if self.notification:
            self.notification.update(delta)

        if self.current_state == STATE_INIT:
            # One Time initialisation
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
                        self.notification = Notification(f"Initialised", port = self.detected_port)
                        self.ports_with_hexdrive.add(self.detected_port)
                        self.current_state = STATE_WAIT
                    else:
                        self.notification = Notification(f"Failed", port = self.detected_port)
                        self.error_message = ["EEPROM","Initialisation","Failed"]
                        self.current_state = STATE_ERROR          
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print(f"H:Cancelled")
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
                            self.notification = Notification(f"Upgraded", port = self.upgrade_port)
                            self.ports_with_upgraded_hexdrive.add(self.upgrade_port)
                            self.error_message = ["Upgraded","Please","Reboop"]
                            self.current_state = STATE_ERROR                                     
                        else:
                            self.notification = Notification(f"Failed", port = self.upgrade_port)
                            self.error_message = ["HexDrive","Programming","Failed"]
                            self.current_state = STATE_ERROR
                    else:
                        self.error_message = ["HexDrive","Read","Failed"]
                        self.current_state = STATE_ERROR
                elif self.button_states.get(BUTTON_TYPES["CANCEL"]):
                    print(f"H:Cancelled")
                    self.button_states.clear()
                    self.current_state = STATE_WAIT
                return
            elif 0 < len(self.ports_with_blank_eeprom):
                # if there are any ports with blank eeproms
                # Show the UI prompt and wait for button press
                self.detected_port = self.ports_with_blank_eeprom.pop()
                self.notification = Notification(f"Initialise?", port = self.detected_port)
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
                        self.notification = Notification(f"Upgrade?", port = self.upgrade_port)
                        self.current_state = STATE_UPGRADE
                else:
                    print(f"H:Error reading Hexpansion header")
                    self.notification = Notification(f"Error", port = port)
                    self.error_message = ["Hexpansion","Read","Failed"]
                    self.current_state = STATE_ERROR        
            elif self.current_state == STATE_WAIT: 
                if 0 < len(self.ports_with_upgraded_hexdrive):
                    valid_port = next(iter(self.ports_with_upgraded_hexdrive))
                    # We have at least one HexDrive with the latest App.py
                    self.hexdrive_seen = True
                    # Find our running hexdrive app
                    for app in scheduler.apps:
                        if hasattr(app, "config"):
                            app.config.port == valid_port
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


        if self.button_states.get(BUTTON_TYPES["CANCEL"]) and self.current_state in MINIMISE_VALID_STATES:
            self.button_states.clear()
            eventbus.emit(PatternEnable()) # TODO replace with on lose focus on gain focus
            self.minimise()
        elif self.current_state == STATE_MENU:
            # Exit start menu
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                eventbus.emit(PatternDisable())
                self.current_state = STATE_RECEIVE_INSTR
                self.button_states.clear()
        elif self.current_state == STATE_WARNING:
            # Exit warning screen
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.current_state = STATE_MENU
                self.button_states.clear()
        elif self.current_state == STATE_RECEIVE_INSTR:
            self.clear_leds()
            # Enable/disable scrolling and check for long press
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                if self.long_press_delta == 0:
                    # TODO Move to button up event
                    self.is_scroll = not self.is_scroll
                    self.notification = Notification(f"Scroll {self.is_scroll}")
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
                self.power_plan_iter = chain(*(instr.power_plan_iterator for instr in self.instructions))
                self.hexdrive_app.set_power(True)
                self.current_state = STATE_RUN

        elif self.current_state == STATE_RUN:
            print(delta)
            power = self.get_current_power_level(delta)
            if power is None:
                eventbus.emit(PatternEnable())
                self.current_state = STATE_DONE
            else:
                print(f"Using power: {power}")
                self.hexdrive_app.set_pwm(power)

                
        elif self.current_state == STATE_DONE:
            if self.button_states.get(BUTTON_TYPES["CONFIRM"]):
                self.hexdrive_app.set_power(False)
                self.reset()

    def _handle_instruction_press(self, press_type: BUTTON_TYPES):
        if self.last_press == press_type:
            self.current_instruction.inc()
        else:
            self.finalize_instruction()
            self.current_instruction = Instruction(press_type)
        self.last_press = press_type

    def reset(self):
        self.current_state = STATE_MENU
        self.button_states.clear()
        self.last_press: BUTTON_TYPES = BUTTON_TYPES["CANCEL"]
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
            ctx.rgb(0.1,0,0).rectangle(-120,-120,240,240).fill()
        else:
            ctx.rgb(0,0,0.1).rectangle(-120,-120,240,240).fill()

        if self.current_state == STATE_WARNING:
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"Please buy")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"Hexpansion")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text(f"from")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 20).text(f"RobotMad")
        elif self.current_state == STATE_REMOVED:
            ctx.rgb(1,1,0).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"removed")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"Please reinsert")            
        elif self.current_state == STATE_DETECTED:  
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 00).text(f"Hexpansion")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 00).text(f"Detected in")
            ctx.rgb(0,0,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 00).text(f"Slot {self.detected_port}")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 00).text(f"Init EEPROM")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 00).text(f"as HexDrive?")
        elif self.current_state == STATE_UPGRADE:  
            ctx.rgb(1,1,0).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 00).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 00).text(f"Detected in")
            ctx.rgb(0,0,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 00).text(f"Slot {self.upgrade_port}")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 00).text(f"Program EEPROM")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 4*VERTICAL_OFFSET + 00).text(f"with new App?")            
        elif self.current_state == STATE_PROGRAMMING:
            ctx.rgb(1,1,0).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 20).text(f"HexDrive")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 20).text(f"Programming")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 20).text(f"EEPROM")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 20).text(f"...")             
        elif self.current_state == STATE_MENU:
            ctx.rgb(1,1,1).move_to(H_START, V_START + 0*VERTICAL_OFFSET + 00).text(f"To Program:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 1*VERTICAL_OFFSET + 00).text(f"Press C")
            ctx.rgb(1,1,1).move_to(H_START, V_START + 2*VERTICAL_OFFSET + 10).text(f"When finished:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 3*VERTICAL_OFFSET + 10).text(f"Long press C")
        elif self.current_state == STATE_ERROR:
            for i_num, instr in enumerate(self.error_message):
                ctx.rgb(1,0,0).move_to(H_START, V_START + VERTICAL_OFFSET * i_num).text(str(instr))
        elif self.current_state == STATE_RECEIVE_INSTR:
            for i_num, instr in enumerate(["START"] + self.instructions + [self.current_instruction, "END"]):
                ctx.rgb(1,1,0).move_to(H_START, V_START + VERTICAL_OFFSET * (self.scroll_offset + i_num)).text(str(instr))
        elif self.current_state == STATE_COUNTDOWN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running in:")
            countdown_val = (RUN_COUNTDOWN_MS - self.run_countdown_elapsed_ms) / 1000
            ctx.rgb(1,1,0).move_to(H_START, V_START+VERTICAL_OFFSET).text(str(countdown_val))
        elif self.current_state == STATE_RUN:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text("Running power")
            ctx.rgb(1,0,0).move_to(H_START-30, V_START + 2*VERTICAL_OFFSET).text(str(self.current_power_duration))
        elif self.current_state == STATE_DONE:
            ctx.rgb(1,1,1).move_to(H_START, V_START).text(f"Complete!")
            ctx.rgb(1,1,1).move_to(H_START, V_START + VERTICAL_OFFSET).text("To restart:")
            ctx.rgb(1,1,0).move_to(H_START, V_START + 2*VERTICAL_OFFSET).text("Press C")
        if self.notification:
            self.notification.draw(ctx)
        ctx.restore()


    def get_current_power_level(self, delta) -> int:
        # takes in delta as ms since last call
        # if delta was > 20... what to do
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
            self.current_instruction.make_power_plan()
            self.instructions.append(self.current_instruction)
            if len(self.instructions) >= 5:
                self.scroll_offset -= 1
            self.current_instruction = None

    def clear_leds(self):
        for i in range(12):
            tildagonos.leds[i+1] = (0, 0, 0)

def chain(*iterables):
    for iterable in iterables:
        yield from iterable

__app_export__ = BadgeBotApp
