# Hexpansion & EEPROM Management Module for BadgeBot
#
# Handles detection, initialisation, programming, upgrading and erasure of
# HexDrive / HexSense hexpansion EEPROMs.
#
# Public interface (called by the main app):
#   __init__(app)       – wire up to the main BadgeBotApp instance
#   register_events()   – register hexpansion insertion/removal handlers on eventbus
#   unregister_events() – unregister hexpansion event handlers
#   scan_ports()        – initial port scan at startup
#   update(delta)       – per-tick hexpansion state-machine update
#   draw(ctx)           – render hexpansion-related UI states

import os
import sys
import time
import vfs
from app_components.notification import Notification
from app_components.tokens import label_font_size, button_labels
from events.input import BUTTON_TYPES
from machine import I2C
from system.eventbus import eventbus
from system.hexpansion.events import HexpansionInsertionEvent
from system.hexpansion.header import HexpansionHeader, write_header, read_header
from system.hexpansion.util import get_hexpansion_block_devices
from system.hexpansion.config import HexpansionConfig
from system.scheduler import scheduler

# HexDrive Hexpansion constants
# EEPROM Constants
_EEPROM_ADDR  = 0x50                # I2C address of the EEPROM on the HexDrive and HexSense Hexpansion
_EEPROM_NUM_ADDRESS_BYTES = 2       # Number of bytes used for the memory address when reading from the EEPROM (e.g. 2 for 16-bit addressing)
_EEPROM_PAGE_SIZE = 32
_EEPROM_TOTAL_SIZE = 64 * 1024 // 8

# Default erase slot
_ERASE_SLOT = 0

_IS_SIMULATOR = sys.platform != "esp32"

# Local sub-states (internal to Hexpansion Mgr)
_SUB_INIT          = 0           # Initial state on app startup, before first port scan
_SUB_CHECK         = 1           # Checks for EEPROMs and HexDrives
_SUB_DETECTED      = 2           # Hexpansion detected & ready for EEPROM initialisation
_SUB_UPGRADE       = 3           # Hexpansion ready for App upgrade
_SUB_ERASE         = 4           # Hexpansion ready for EEPROM erase
_SUB_PROGRAMMING   = 5           # Hexpansion EEPROM programming


_HEXDRIVE_REQUIRED_MESSAGE = ["BadgeBot requires","HexDrive hexpansion","from RobotMad","github.com","/TeamRobotmad","/BadgeBot"]
_HEXDRIVE_REQUIRED_MESSAGE_COLOURS = [(1,1,1),(1,1,0),(1,1,0),(1,1,1),(1,1,1),(1,1,1)]

# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):
    """Register hexpansion-management-specific settings in the shared settings dict."""
    s['erase_slot']    = MySetting(s, _ERASE_SLOT, 0, 6)


# ---- Hexpansion type descriptor -------------------------------------------

class HexpansionType:
    """Descriptor for known hexpansion types, used for detection and EEPROM programming.

    Parameters
    ----------
        pid: the PID value to identify the hexpansion type from its EEPROM header
        name: human-friendly name of the hexpansion type (e.g. "HexDrive")
        vid: the VID value to identify the hexpansion type from its EEPROM header (default 0xCAFE)
        motors, steppers, servos, sensors: the capabilities of this hexpansion type, used to configure the app when detected
        sub_type: a human-friendly string describing the specific variant of this hexpansion type
        app_mpy_name: the filename of the .mpy to copy to the hexpansion EEPROM for this type (if any)
        app_mpy_version: the version string to report for the .mpy copied to the hexpansion EEPROM for this type (if any)
        app_name: the name of the App class to look for when checking if a detected hexpansion's app is running (if any)
    """
    def __init__(self, pid, name, vid=0xCAFE, motors=0, steppers=0, servos=0,
                 sensors=0, sub_type="Unknown", app_mpy_name=None,
                 app_mpy_version=None, app_name=None):
        self.vid = vid
        self.pid = pid
        self.name = name
        self.sub_type = sub_type
        self.motors = motors
        self.servos = servos
        self.steppers = steppers
        self.sensors = sensors
        self.app_mpy_name = app_mpy_name
        self.app_mpy_version = app_mpy_version
        self.app_name = app_name


# ---- Hexpansion management -------------------------------------------------

class HexpansionMgr:
    """Manages hexpansion detection, EEPROM programming, and upgrades.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance so that shared state
        (settings, hexdrive_app, ports_with_*, current_state …) can be
        read and written.
    """

    # Sub-states are defined at module level (_SUB_*); app-level state
    # routing is handled by the dispatch tables in app.py.

    def __init__(self, app):
        self.app = app
        self._sub_state = _SUB_INIT
        self._prev_state = _SUB_INIT        
        self.hexpansion_start_timer: float  = 0.0
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

    def register_events(self):
        """Register hexpansion insertion/removal event handlers directly."""
        from system.hexpansion.events import HexpansionRemovalEvent
        eventbus.on_async(HexpansionInsertionEvent, self._handle_insertion, self.app)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_removal, self.app)

    def unregister_events(self):
        """Unregister hexpansion event handlers."""
        from system.hexpansion.events import HexpansionRemovalEvent
        eventbus.remove(HexpansionInsertionEvent, self._handle_insertion, self.app)
        eventbus.remove(HexpansionRemovalEvent, self._handle_removal, self.app)


    # ------------------------------------------------------------------
    # Async event handlers (registered directly on eventbus)
    # ------------------------------------------------------------------

    async def _handle_removal(self, event):
        app = self.app
        self.hexpansion_slot_type[event.port - 1] = None
        if event.port in self.ports_with_blank_eeprom:
            if app.settings['logging'].v:
                print(f"H:EEPROM removed from port {event.port}")
            self.ports_with_blank_eeprom.remove(event.port)
        if event.port in self.ports_with_hexdrive:
            if app.settings['logging'].v:
                print(f"H:HexDrive removed from port {event.port}")
            self.ports_with_hexdrive.remove(event.port)
        if event.port in self.ports_with_hexsense:
            if app.settings['logging'].v:
                print(f"H:HexSense removed from port {event.port}")
            self.ports_with_hexsense.remove(event.port)
        if event.port in self.ports_with_latest_hexdrive:
            if app.settings['logging'].v:
                print(f"H:HexDrive V{app.APP_VERSION} removed from port {event.port}")
            self.ports_with_latest_hexdrive.remove(event.port)
        if self._sub_state == _SUB_DETECTED and event.port == self.detected_port:
            # The hexpansion that is currently being initialised has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True
        elif self._sub_state == _SUB_UPGRADE and event.port == self.upgrade_port:
            # The hexpansion that is currently being upgraded has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True
        elif self.hexdrive_port is not None and event.port == self.hexdrive_port:
            # The HexDrive that is currently in use has been removed, so update the app state accordingly (which may trigger a fallback to a different HexDrive if available, or a warning if no HexDrive is available).
            app.hexpansion_update_required = True
        elif self.waiting_app_port is not None and event.port == self.waiting_app_port:
            # The hexpansion that is currently waiting for its app to start has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True
        elif self.erase_port is not None and event.port == self.erase_port:
            # The hexpansion that is currently waiting to be erased has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True


    async def _handle_insertion(self, event):
        if self.check_port_for_known_hexpansions(event.port):
            # A known hexpansion type has been detected on the inserted port, so trigger an update of the hexpansion management state machine to handle it.
            self.app.hexpansion_update_required = True


    # ------------------------------------------------------------------
    # Port scanning
    # ------------------------------------------------------------------

    def scan_ports(self):
        """Scan all ports for known hexpansions, and update app state accordingly."""
        for port in range(1, 7):
            self.check_port_for_known_hexpansions(port)


    def check_port_for_known_hexpansions(self, port) -> bool:
        """Check the given port for known hexpansion types by reading the EEPROM header, and update app state accordingly.
           Returns True if a known hexpansion type is detected (even if it was already known), False otherwise."""
        app = self.app
        if port not in range(1, 7):
            return False
        try:
            hexpansion_header = read_header(port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except OSError:
            return False
        except RuntimeError:
            if _IS_SIMULATOR:
                return False
            if app.settings['logging'].v:
                print(f"H:Found EEPROM on port {port}")
            self.ports_with_blank_eeprom.add(port)
            return True
        for index, hexpansion_type in enumerate(app.HEXPANSION_TYPES):
            if hexpansion_header.vid == hexpansion_type.vid and hexpansion_header.pid == hexpansion_type.pid:
                if app.settings['logging'].v:
                    print(f"H:Found '{hexpansion_type.sub_type}' {hexpansion_type.name} on port {port}")
                if hexpansion_type.name == app.HEXPANSION_TYPES[app.HEXDRIVE_HEXPANSION_INDEX].name:
                    if port not in self.ports_with_latest_hexdrive:
                        self.ports_with_hexdrive.add(port)
                elif hexpansion_type.name == app.HEXPANSION_TYPES[app.HEXSENSE_HEXPANSION_INDEX].name:
                    print(f"H:HexSense detected on port {port}, configuring line sensors")
                    app.num_line_sensors = hexpansion_type.sensors
                    app.line_sensors_hexpansion_config = HexpansionConfig(port)
                    self.ports_with_hexsense.add(port)
                self.hexpansion_slot_type[port - 1] = index
                return True
        return False


    # ------------------------------------------------------------------
    # EEPROM operations
    # ------------------------------------------------------------------

    def update_app_in_eeprom(self, port, addr) -> bool:
        """Copy the appropriate .mpy file to the hexpansion EEPROM on the given port, based on the selected hexpansion type.  Returns True if successful, False otherwise."""
        app = self.app
        if app.HEXPANSION_TYPES[self.hexpansion_init_type].app_mpy_name is None:
            if app.settings['logging'].v:
                print(f"H:Hexpansion type {app.HEXPANSION_TYPES[self.hexpansion_init_type].name} does not have an app to copy to EEPROM")
            return False
        source_file = app.HEXPANSION_TYPES[self.hexpansion_init_type].app_mpy_name
        if app.settings['logging'].v:
            print(f"H:Updating app.mpy on port {port} with {source_file}")
        try:
            i2c = I2C(port)
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error opening I2C port {port}: {e}")
            return False
        header = read_header(port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        if header is None:
            if app.settings['logging'].v:
                print(f"H:Error reading header on port {port}")
            return False
        try:
            _, partition = get_hexpansion_block_devices(i2c, header, addr, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
            if app.settings['logging'].v:
                print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=False)
            except OSError as e:
                if e.args[0] == 1:
                    already_mounted = True
                else:
                    print(f"H:Error mounting: {e}")
            except Exception as e:      # pylint: disable=broad-except
                print(f"H:Error mounting: {e}")
        source_path = "/" + __file__.rsplit("/", 1)[0] + f"/{source_file}"
        dest_path = f"{mountpoint}/app.mpy"
        try:
            if app.settings['logging'].v:
                print(f"H:Deleting {dest_path}")
            os.remove(dest_path)
        except Exception as e:          # pylint: disable=broad-except
            if e.args[0] != 2:
                print(f"H:Error deleting {dest_path}: {e}")

        if app.settings['logging'].v:
            print(f"H:Copying {source_path} to {dest_path}")

        try:
            template = open(source_path, "rb")
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error opening {source_path}: {e}")
            return False

        try:
            appfile = open(dest_path, "wb")
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error opening {dest_path}: {e}")
            return False

        try:
            appfile.write(template.read())
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error updating Hexpansion: {e}")
            return False

        try:
            appfile.close()
            template.close()
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error closing files: {e}")
            return False
        if not already_mounted:
            try:
                vfs.umount(mountpoint)
                if app.settings['logging'].v:
                    print(f"H:Unmounted {mountpoint}")
            except Exception as e:    # pylint: disable=broad-except
                print(f"H:Error unmounting {mountpoint}: {e}")
                return False
        if app.settings['logging'].v:
            print(f"H:Hexpansion app.mpy updated to version {app.HEXPANSION_TYPES[self.hexpansion_init_type].app_mpy_version}")
        return True


    def prepare_eeprom(self, port, addr) -> bool:
        """Write the EEPROM header and format the filesystem on the hexpansion EEPROM on the given port, based on the selected hexpansion type.  Returns True if successful, False otherwise."""
        app = self.app
        if app.settings['logging'].v:
            print(f"H:Initialising EEPROM on port {port}")
        hexpansion_header_to_write = HexpansionHeader(
            manifest_version="2024",
            fs_offset=32,
            eeprom_page_size=_EEPROM_PAGE_SIZE,
            eeprom_total_size=_EEPROM_TOTAL_SIZE,
            vid=app.HEXPANSION_TYPES[self.hexpansion_init_type].vid,
            pid=app.HEXPANSION_TYPES[self.hexpansion_init_type].pid,
            unique_id=0x0,
            friendly_name=app.HEXPANSION_TYPES[self.hexpansion_init_type].name,
        )
        try:
            i2c = I2C(port)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error opening I2C port {port}: {e}")
            return False
        try:
            write_header(port, hexpansion_header_to_write, addr_len=_EEPROM_NUM_ADDRESS_BYTES, page_size=_EEPROM_PAGE_SIZE)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error writing header: {e}")
            return False
        try:
            hexpansion_header = read_header(port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error reading header back: {e}")
            return False
        try:
            _, partition = get_hexpansion_block_devices(i2c, hexpansion_header, addr, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False
        try:
            vfs.VfsLfs2.mkfs(partition)
            if app.settings['logging'].v:
                print("H:EEPROM formatted")
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error formatting: {e}")
            return False
        try:
            mountpoint = '/hexpansion_' + str(port)
            vfs.mount(partition, mountpoint, readonly=False)
            if app.settings['logging'].v:
                print("H:EEPROM initialised")
        except OSError as e:
            if e.args[0] == 1:
                if app.settings['logging'].v:
                    print("H:EEPROM initialised")
            else:
                print(f"H:Error mounting: {e}")
                return False
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error mounting: {e}")
            return False
        return True


    def erase_eeprom(self, port, addr) -> bool:
        """Erase the hexpansion EEPROM on the given port.  Returns True if successful, False otherwise."""
        app = self.app
        if app.settings['logging'].v:
            print(f"H:Erasing EEPROM on port {port}")
        try:
            i2c = I2C(port)
            for page in range(_EEPROM_TOTAL_SIZE // _EEPROM_PAGE_SIZE):
                mem_addr = page * _EEPROM_PAGE_SIZE
                mem_addr_mask = 1 << (_EEPROM_NUM_ADDRESS_BYTES * 8) - 1
                i2c.writeto_mem((addr | (mem_addr >> (8 * _EEPROM_NUM_ADDRESS_BYTES))), (mem_addr & mem_addr_mask), bytes([0xFF] * _EEPROM_PAGE_SIZE), addrsize=(8 * _EEPROM_NUM_ADDRESS_BYTES))
                while True:
                    try:
                        if i2c.writeto((addr | (mem_addr >> (8 * _EEPROM_NUM_ADDRESS_BYTES))), bytes([mem_addr & 0xFF]) if _EEPROM_NUM_ADDRESS_BYTES == 1 else bytes([mem_addr >> 8, mem_addr & 0xFF])):
                            break
                    except OSError:
                        pass
                    finally:
                        time.sleep_ms(1)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error erasing EEPROM: {e}")
            return False
        return True

    def find_hexpansion_app(self, port) -> object | None:
        """Find the app instance running from the hexpansion on the given port, if any.  Returns the app instance if found, None otherwise."""
        app = self.app
        expected_app_name = app.HEXPANSION_TYPES[self.hexpansion_slot_type[port - 1]].app_name
        for an_app in scheduler.apps:
            if type(an_app).__name__ == expected_app_name:
                if hasattr(an_app, "config") and hasattr(an_app.config, "port") and an_app.config.port == port:
                    return an_app
        return None

    # ------------------------------------------------------------------
    # Per-tick update  (state machine for hexpansion management)
    # ------------------------------------------------------------------

    def update(self, delta) -> bool:
        app = self.app

        if self._sub_state == _SUB_INIT:
            self.scan_ports()
            if (len(self.ports_with_hexdrive) == 0) and (len(self.ports_with_blank_eeprom) == 0):
                app.show_message(_HEXDRIVE_REQUIRED_MESSAGE, _HEXDRIVE_REQUIRED_MESSAGE_COLOURS, "warning")
            elif not self._check_hexpansion_erase_port(delta):
                self._sub_state = _SUB_CHECK
        elif app.hexpansion_update_required:
            # This flag is set when a hexpansion-related event occurs that should trigger an update of the hexpansion management state machine (e.g. insertion/removal of a hexpansion).
            app.hexpansion_update_required = False
            if self._sub_state != _SUB_CHECK:
                print("H:Hexpansion Check")
                app.set_menu(None)
                self._sub_state = _SUB_CHECK
                # the check will show a message which forces the main App out of the current state.
        elif self._sub_state == _SUB_PROGRAMMING:
            self._update_state_programming(delta)
        elif self._sub_state == _SUB_DETECTED:
            self._update_state_detected(delta)
        elif self._sub_state == _SUB_ERASE:
            self._update_state_erase(delta)
        elif self._sub_state == _SUB_UPGRADE:
            self._update_state_upgrade(delta)
        elif self._check_hexpansion_ports(delta):
            pass
        elif self._check_hexdrive_ports(delta):
            pass
        elif self._sub_state == _SUB_CHECK:
            self._update_state_check(delta)

        if self._sub_state != self._prev_state:
            if app.settings['logging'].v:
                print(f"H:State: {self._prev_state} -> {self._sub_state}")
            self._prev_state = self._sub_state

    # ------------------------------------------------------------------
    # Individual state handlers
    # ------------------------------------------------------------------

    def _update_state_programming(self, delta):     # pylint: disable=unused-argument
        app = self.app

        if self.upgrade_port is not None:
            if app.HEXPANSION_TYPES[self.hexpansion_init_type].app_mpy_name is None:
                app.notification = Notification("No App", port=self.upgrade_port)
                self._sub_state = _SUB_CHECK
                app.show_message(["No App", "for this", "Hexpansion"], [(1,1,0),(1,1,1),(1,1,1)])
            elif self.update_app_in_eeprom(self.upgrade_port, _EEPROM_ADDR):
                app.notification = Notification("Upgraded", port=self.upgrade_port)
                eventbus.emit(HexpansionInsertionEvent(self.upgrade_port))
                if app.settings['logging'].v:
                    print(f"H:Hexpansion on port {self.upgrade_port} upgraded")
                self._sub_state = _SUB_CHECK
                app.show_message(["Upgraded:", "Please", "reboop"], [(0,1,0),(1,1,1),(1,1,1)], "reboop")
            else:
                app.notification = Notification("Failed", port=self.upgrade_port)
                self._sub_state = _SUB_CHECK
                app.show_message(["Hexpansion", "programming", "failed"], [(1,0,0),(1,0,0),(1,0,0)], "error")
            self.upgrade_port = None
        elif self.detected_port is not None:
            if self.prepare_eeprom(self.detected_port, _EEPROM_ADDR):
                app.notification = Notification("Initialised", port=self.detected_port)
                self.upgrade_port = self.detected_port
                self.hexpansion_slot_type[self.detected_port - 1] = self.hexpansion_init_type
                self._sub_state = _SUB_UPGRADE
            else:
                app.notification = Notification("Failed", port=self.detected_port)
                self._sub_state = _SUB_CHECK
                self.hexpansion_slot_type[self.detected_port - 1] = None
                app.show_message(["EEPROM", "initialisation", "failed"], [(1,0,0),(1,0,0),(1,0,0)], "error")
            self.detected_port = None
        elif app.settings['logging'].v:
            print("H:Error - no port to program")
            self._sub_state = _SUB_CHECK


    def _update_state_detected(self, delta):        # pylint: disable=unused-argument
        """ Allow User to select which sub-type they want to initialise the hexpansion as (if there are multiple sub-types with the same PID), and confirm or cancel the initialisation."""
        app = self.app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            self._sub_state = _SUB_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.settings['logging'].v:
                print("H:Initialise Cancelled")
            self.detected_port = None
            self._sub_state = _SUB_CHECK
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            self.hexpansion_init_type = (self.hexpansion_init_type + 1) % len(app.HEXPANSION_TYPES)
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            self.hexpansion_init_type = (self.hexpansion_init_type - 1) % len(app.HEXPANSION_TYPES)
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self.hexpansion_init_type = 1
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self.hexpansion_init_type = 2
            app.refresh = True


    def _update_state_erase(self, delta):       # pylint: disable=unused-argument
        """ Allow User to confirm or cancel EEPROM erasure."""
        app = self.app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            if self.erase_eeprom(self.erase_port, _EEPROM_ADDR):
                app.notification = Notification("Erased", port=self.erase_port)
                self.ports_with_blank_eeprom.add(self.erase_port)                
                self._sub_state = _SUB_CHECK
                app.show_message(["Erased:", "Please", "reboop"], [(0,1,0),(1,1,1),(1,1,1)], "reboop")
            else:
                app.notification = Notification("Failed", port=self.erase_port)
                self._sub_state = _SUB_CHECK
                app.show_message(["EEPROM", "erasure", "failed"], [(1,0,0),(1,0,0),(1,0,0)], "error")
            if self.hexdrive_port == self.erase_port:
                self.hexdrive_port = None
                app.hexdrive_app = None
                app.motor_controller = None
                if app.settings['logging'].v:
                    print(f"H:HexDrive on port {self.erase_port} erased!")
                self.ports_with_hexdrive.discard(self.erase_port)
            if app.line_sensors_hexpansion_config is not None and app.line_sensors_hexpansion_config.port == self.erase_port:
                app.line_sensors_hexpansion_config = None
                self.ports_with_hexsense.discard(self.erase_port)
            self.erase_port = None
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if app.settings['logging'].v:
                print("H:Erase Cancelled")
            app.button_states.clear()
            self.erase_port = None
            self._sub_state = _SUB_CHECK


    def _update_state_upgrade(self, delta):     # pylint: disable=unused-argument
        """ Allow User to confirm or cancel App upgrade."""
        app = self.app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.notification = Notification("Upgrading", port=self.upgrade_port)
            self._sub_state = _SUB_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if app.settings['logging'].v:
                print("H:Upgrade Cancelled")
            app.button_states.clear()
            self.upgrade_port = None
            self._sub_state = _SUB_CHECK


    def _update_state_check(self, delta):       # pylint: disable=unused-argument
        """ Check for HexDrive presence and update app state accordingly."""
        app = self.app
        if 0 < len(self.ports_with_latest_hexdrive):
            if self.hexdrive_port is not None and self.hexdrive_port not in self.ports_with_latest_hexdrive:
                print(f"Check: {self.hexdrive_port} lost")
                self.hexdrive_port = None
                app.hexdrive_app = None
                app.motor_controller = None

            if self.hexdrive_port is None:
                valid_port = next(iter(self.ports_with_latest_hexdrive))
                hexdrive_app = self.find_hexpansion_app(valid_port)
                if hexdrive_app is not None:
                    self.hexdrive_port = valid_port
                    app.hexdrive_app = hexdrive_app
                    if self.hexpansion_slot_type[valid_port - 1] is not None:
                        app.num_motors = app.HEXPANSION_TYPES[self.hexpansion_slot_type[valid_port - 1]].motors
                        app.num_servos = app.HEXPANSION_TYPES[self.hexpansion_slot_type[valid_port - 1]].servos
                        app.num_steppers = app.HEXPANSION_TYPES[self.hexpansion_slot_type[valid_port - 1]].steppers

                    # Create the high-level MotorController for IMU-aided driving
                    # (only when the HexDrive has motors)
                    if app.num_motors > 0:
                        try:
                            from .motor_controller import MotorController
                            app.motor_controller = MotorController(
                                hexdrive_app, app.settings,
                                fwd_dir_setting=app.settings.get('fwd_dir'),
                                front_face_setting=app.settings.get('front_face'),
                            )
                        except Exception as e:      # pylint: disable=broad-except
                            if app.settings['logging'].v:
                                print(f"H:MotorController init failed: {e}")
                            app.motor_controller = None

                    if (0 < app.HEXPANSION_TYPES[self.hexpansion_slot_type[valid_port - 1]].steppers) or app.hexdrive_app.get_status():
                        if app.settings['logging'].v:
                            print(f"H:HexDrive [{valid_port}] OK")
                        self._sub_state = _SUB_CHECK
                        app.return_to_menu()
                    else:
                        if app.settings['logging'].v:
                            print(f"H:HexDrive {valid_port}: Failed to initialise PWM resources")
                        self._sub_state = _SUB_CHECK
                        app.show_message([f"HexDrive {valid_port}", "PWM Init", "Failed", "Please", "reboop"], [(1,0,0),(1,0,0),(1,0,0),(1,1,1),(1,1,1)], "reboop")
                else:
                    if app.settings['logging'].v:
                        print(f"H:HexDrive {valid_port}: App not found, please reboop")
                    self._sub_state = _SUB_CHECK
                    app.show_message([f"HexDrive {valid_port}", "App not found.", "Please", "reboop"], [(1,0,0),(1,0,0),(1,1,1),(1,1,1)], "reboop")
            else:
                self._sub_state = _SUB_CHECK
                app.return_to_menu()
        elif self.hexdrive_port is not None:
            print(f"Check: {self.hexdrive_port} lost")
            self.hexdrive_port = None
            app.hexdrive_app = None
            app.motor_controller = None
            self._sub_state = _SUB_CHECK
            app.show_message(["HexDrive","removed.","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], "error")
        else:
            self._sub_state = _SUB_CHECK
            app.show_message(_HEXDRIVE_REQUIRED_MESSAGE, _HEXDRIVE_REQUIRED_MESSAGE_COLOURS, "warning")


    def _check_hexpansion_erase_port(self, delta) -> bool:   # pylint: disable=unused-argument
        """Check if the user has selected a port to erase, and if so, prompt them to confirm or cancel the erasure.
           if there is a blank EEPROM in the selected slot.
           Return True if we are now in the erase confirmation state, False otherwise (e.g. if no erase port selected, or if selected port doesn't have a blank EEPROM). 
        """
        app = self.app
        erase_port = app.settings['erase_slot'].v
        if erase_port != 0 and erase_port not in self.ports_with_blank_eeprom:
            try:
                hexpansion_header = read_header(erase_port, addr_len=_EEPROM_NUM_ADDRESS_BYTES) # pylint: disable=unused-variable
            except OSError:
                return False
            except RuntimeError:
                if _IS_SIMULATOR:
                    return False
            if app.settings['logging'].v:
                print(f"H:Hexpansion on port {erase_port} Erase?")
            self.erase_port = erase_port
            app.notification = Notification("Erase?", port=self.erase_port)
            self._sub_state = _SUB_ERASE
            return True
        return False


    def _check_hexpansion_ports(self, delta) -> bool:       # pylint: disable=unused-argument
        """Check for hexpansion presence in any ports, and if a new hexpansion with a blank EEPROM is detected, prompt the user to initialise it.  
           Returns True if we are now in the initialise confirmation state, False otherwise."""
        app = self.app
        if 0 < len(self.ports_with_blank_eeprom):
            self.detected_port = self.ports_with_blank_eeprom.pop()
            app.notification = Notification("Initialise?", port=self.detected_port)
            self._sub_state = _SUB_DETECTED
            return True
        return False


    def _check_hexdrive_ports(self, delta) -> bool:
        """Check if any ports with Hexpansions contain a HexDrive, and if a new HexDrive is detected, check if it has the latest app version and prompt the user to upgrade if not.
           Returns True if we are now in the upgrade confirmation state, False otherwise."""
        app = self.app
        if self.waiting_app_port is not None or (0 < len(self.ports_with_hexdrive)):
            if self.waiting_app_port is None:
                self.waiting_app_port = self.ports_with_hexdrive.pop()
                self.hexpansion_start_timer = 0
                hexpansion_app = self.find_hexpansion_app(self.waiting_app_port)
                if hexpansion_app is not None:
                    try:
                        hexpansion_app_version = hexpansion_app.get_version()
                    except Exception as e:   # pylint: disable=broad-except
                        hexpansion_app_version = 0
                        print(f"H:Error getting Hexpansion app version - assume old: {e}")
                elif 5.0 < self.hexpansion_start_timer:
                    if app.settings['logging'].v:
                        print("H:Timeout waiting for Hexpansion app to be started - assume it needs upgrading")
                    hexpansion_app_version = 0
                else:
                    if 0 == self.hexpansion_start_timer:
                        if app.settings['logging'].v:
                            print(f"H:No app found on port {self.waiting_app_port} - WAITING for app to appear in Scheduler")
                    app.notification = Notification("Checking...", port=self.waiting_app_port)
                    self.hexpansion_start_timer += delta / 1000
                    return True
                if hexpansion_app_version == app.HEXPANSION_TYPES[self.hexpansion_slot_type[self.waiting_app_port - 1]].app_mpy_version:
                    if app.settings['logging'].v:
                        print(f"H:Hexpansion on port {self.waiting_app_port} has latest App")
                    self.ports_with_latest_hexdrive.add(self.waiting_app_port)
                    self._sub_state = _SUB_CHECK
                else:
                    if app.settings['logging'].v:
                        print(f"H:Hexpansion app on port {self.waiting_app_port} needs upgrading from version {hexpansion_app_version} to {app.HEXPANSION_TYPES[self.hexpansion_slot_type[self.waiting_app_port - 1]].app_mpy_version}")
                    self.upgrade_port = self.waiting_app_port
                    app.notification = Notification("Upgrade?", port=self.upgrade_port)
                    self._sub_state = _SUB_UPGRADE
            self.waiting_app_port = None
            self.hexpansion_start_timer = 0
            return True
        return False


    # ------------------------------------------------------------------
    # Draw hexpansion-related states
    # ------------------------------------------------------------------

    def draw(self, ctx) -> bool:
        """Render UI for hexpansion-related states.  Returns True if handled."""
        app = self.app

        def type_name_for_port(port, fallback_type_idx):
            """Return detected type name for a port, falling back to a selected type index."""
            if port is not None and 1 <= port <= len(self.hexpansion_slot_type):
                slot_idx = self.hexpansion_slot_type[port - 1]
                if slot_idx is not None and 0 <= slot_idx < len(app.HEXPANSION_TYPES):
                    return app.HEXPANSION_TYPES[slot_idx].name
            return app.HEXPANSION_TYPES[fallback_type_idx].name

        if self._sub_state == _SUB_DETECTED:
            hexpansion_type = app.HEXPANSION_TYPES[self.hexpansion_init_type].name
            hexpansion_sub_type = app.HEXPANSION_TYPES[self.hexpansion_init_type].sub_type
            app.draw_message(ctx, ["Hexpansion", f"in slot {self.detected_port}:", "Init EEPROM as", hexpansion_sub_type, f"{hexpansion_type}?"], [(1, 1, 1), (1, 1, 1), (1, 1, 1), (0, 0, 1), (1, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", up_label="^", down_label="\u25BC", left_label=app.HEXPANSION_TYPES[app.HEXDRIVE_HEXPANSION_INDEX].name, right_label=app.HEXPANSION_TYPES[app.HEXSENSE_HEXPANSION_INDEX].name, cancel_label="No")
            return True
        elif self._sub_state == _SUB_ERASE:
            hexpansion_type = type_name_for_port(self.erase_port, self.hexpansion_init_type)
            app.draw_message(ctx, [hexpansion_type, f"in slot {self.erase_port}:", "Erase EEPROM?"], [(1, 1, 0), (1, 1, 1), (1, 0, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif self._sub_state == _SUB_UPGRADE:
            hexpansion_type = type_name_for_port(self.upgrade_port, self.hexpansion_init_type)
            app.draw_message(ctx, [hexpansion_type, f"in slot {self.upgrade_port}:", "Upgrade", f"{hexpansion_type} app?"], [(1, 1, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif self._sub_state == _SUB_PROGRAMMING:
            # During upgrade, show the already-detected type for the selected port.
            hexpansion_type = type_name_for_port(self.upgrade_port, self.hexpansion_init_type)
            app.draw_message(ctx, [f"{hexpansion_type}:", "Programming", "EEPROM", "Please wait..."], [(1, 1, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            return True
        return False
    