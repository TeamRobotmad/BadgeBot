# Hexpansion & EEPROM Management Module for BadgeBot
#
# Handles detection, initialisation, programming, upgrading and erasure of
# HexDrive / HexSense hexpansion EEPROMs.  Extracted from linefollower.py so
# that the hexpansion-specific logic can be maintained independently.
#
# Public interface (called by the main app):
#   __init__(app)            – wire up to the main LineFollowerApp instance
#   register_events()       – register hexpansion insertion/removal handlers on eventbus
#   unregister_events()     – unregister hexpansion event handlers
#   scan_ports()           – initial port scan at startup
#   update(delta)          – per-tick hexpansion state-machine update
#   draw(ctx)              – render hexpansion-related UI states
#
# Constants moved here:  _ERASE_SLOT, _EEPROM_*, HexpansionType
# Constants that stay in linefollower.py:  _LOGGING, _IS_SIMULATOR

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
_EEPROM_ADDR = 0x50
_EEPROM_NUM_ADDRESS_BYTES = 2
_EEPROM_PAGE_SIZE = 32
_EEPROM_TOTAL_SIZE = 64 * 1024 // 8

# Default erase slot (moved from linefollower.py)
_ERASE_SLOT = 0

_IS_SIMULATOR = sys.platform != "esp32"


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):
    """Register hexpansion-management-specific settings in the shared settings dict."""
    s['erase_slot']    = MySetting(s, _ERASE_SLOT, 0, 6)


# ---- Hexpansion type descriptor -------------------------------------------

class HexpansionType:
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
    app : LineFollowerApp
        Reference to the main application instance so that shared state
        (settings, hexdrive_app, ports_with_*, current_state …) can be
        read and written.
    """

    # Import state constants locally to keep the module self-contained.
    # They are still defined in linefollower.py (the canonical location).
    # Methods such as update() / draw() import them on demand via `from .linefollower import ...`.

    def __init__(self, app):
        self.app = app

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
        app.hexpansion_slot_type[event.port - 1] = None
        if event.port in app.ports_with_blank_eeprom:
            if app._settings['logging'].v:
                print(f"H:EEPROM removed from port {event.port}")
            app.ports_with_blank_eeprom.remove(event.port)
        if event.port in app.ports_with_hexdrive:
            if app._settings['logging'].v:
                print(f"H:HexDrive removed from port {event.port}")
            app.ports_with_hexdrive.remove(event.port)
        if event.port in app.ports_with_hexsense:
            if app._settings['logging'].v:
                print(f"H:HexSense removed from port {event.port}")
            app.ports_with_hexsense.remove(event.port)
        if event.port in app.ports_with_latest_hexdrive:
            if app._settings['logging'].v:
                print(f"H:HexDrive V{app._APP_VERSION} removed from port {event.port}")
            app.ports_with_latest_hexdrive.remove(event.port)
        if app.current_state == app.STATE_DETECTED and event.port == app.detected_port:
            app.hexpansion_update_required = True
        elif app.current_state == app.STATE_UPGRADE and event.port == app.upgrade_port:
            app.hexpansion_update_required = True
        elif app.hexdrive_port is not None and event.port == app.hexdrive_port:
            app.hexpansion_update_required = True
        elif app.waiting_app_port is not None and event.port == app.waiting_app_port:
            app.hexpansion_update_required = True
        elif app.erase_port is not None and event.port == app.erase_port:
            app.hexpansion_update_required = True

    async def _handle_insertion(self, event):
        if self.check_port_for_known_hexpansions(event.port):
            self.app.hexpansion_update_required = True

    # ------------------------------------------------------------------
    # Port scanning
    # ------------------------------------------------------------------

    def scan_ports(self):
        for port in range(1, 7):
            self.check_port_for_known_hexpansions(port)

    def check_port_for_known_hexpansions(self, port):
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
            if app._settings['logging'].v:
                print(f"H:Found EEPROM on port {port}")
            app.ports_with_blank_eeprom.add(port)
            return True
        for index, hexpansion_type in enumerate(app._HEXPANSION_TYPES):
            if hexpansion_header.vid == hexpansion_type.vid and hexpansion_header.pid == hexpansion_type.pid:
                if app._settings['logging'].v:
                    print(f"H:Found '{hexpansion_type.sub_type}' {hexpansion_type.name} on port {port}")
                if hexpansion_type.name == app._HEXPANSION_TYPES[0].name:
                    if port not in app.ports_with_latest_hexdrive:
                        app.ports_with_hexdrive.add(port)
                elif hexpansion_type.name == app._HEXPANSION_TYPES[5].name:
                    app.num_line_sensors = hexpansion_type.sensors
                    app._line_sensors_hexpansion_config = HexpansionConfig(port)
                    app.ports_with_hexsense.add(port)
                app.hexpansion_slot_type[port - 1] = index
                return True
        return False

    # ------------------------------------------------------------------
    # EEPROM operations
    # ------------------------------------------------------------------

    def update_app_in_eeprom(self, port, addr):
        app = self.app
        if app._HEXPANSION_TYPES[app.hexpansion_init_type].app_mpy_name is None:
            if app._settings['logging'].v:
                print(f"H:Hexpansion type {app._HEXPANSION_TYPES[app.hexpansion_init_type].name} does not have an app to copy to EEPROM")
            return False
        source_file = app._HEXPANSION_TYPES[app.hexpansion_init_type].app_mpy_name
        if app._settings['logging'].v:
            print(f"H:Updating app.mpy on port {port} with {source_file}")
        try:
            i2c = I2C(port)
        except Exception as e:
            print(f"H:Error opening I2C port {port}: {e}")
            return False
        header = read_header(port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        if header is None:
            if app._settings['logging'].v:
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
            if app._settings['logging'].v:
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
        source_path = "/" + __file__.rsplit("/", 1)[0] + f"/{source_file}"
        dest_path = f"{mountpoint}/app.mpy"
        try:
            if app._settings['logging'].v:
                print(f"H:Deleting {dest_path}")
            os.remove(dest_path)
        except Exception as e:
            if e.args[0] != 2:
                print(f"H:Error deleting {dest_path}: {e}")

        if app._settings['logging'].v:
            print(f"H:Copying {source_path} to {dest_path}")

        try:
            template = open(source_path, "rb")
        except Exception as e:
            print(f"H:Error opening {source_path}: {e}")
            return False

        try:
            appfile = open(dest_path, "wb")
        except Exception as e:
            print(f"H:Error opening {dest_path}: {e}")
            return False

        try:
            appfile.write(template.read())
        except Exception as e:
            print(f"H:Error updating Hexpansion: {e}")
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
                if app._settings['logging'].v:
                    print(f"H:Unmounted {mountpoint}")
            except Exception as e:
                print(f"H:Error unmounting {mountpoint}: {e}")
                return False
        if app._settings['logging'].v:
            print(f"H:Hexpansion app.mpy updated to version {app._HEXPANSION_TYPES[app.hexpansion_init_type].app_mpy_version}")
        return True

    def prepare_eeprom(self, port, addr):
        app = self.app
        if app._settings['logging'].v:
            print(f"H:Initialising EEPROM on port {port}")
        hexpansion_header_to_write = HexpansionHeader(
            manifest_version="2024",
            fs_offset=32,
            eeprom_page_size=_EEPROM_PAGE_SIZE,
            eeprom_total_size=_EEPROM_TOTAL_SIZE,
            vid=app._HEXPANSION_TYPES[app.hexpansion_init_type].vid,
            pid=app._HEXPANSION_TYPES[app.hexpansion_init_type].pid,
            unique_id=0x0,
            friendly_name=app._HEXPANSION_TYPES[app.hexpansion_init_type].name,
        )
        try:
            i2c = I2C(port)
        except Exception as e:
            print(f"H:Error opening I2C port {port}: {e}")
            return False
        try:
            write_header(port, hexpansion_header_to_write, addr_len=_EEPROM_NUM_ADDRESS_BYTES, page_size=_EEPROM_PAGE_SIZE)
        except Exception as e:
            print(f"H:Error writing header: {e}")
            return False
        try:
            hexpansion_header = read_header(port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except Exception as e:
            print(f"H:Error reading header back: {e}")
            return False
        try:
            _, partition = get_hexpansion_block_devices(i2c, hexpansion_header, addr, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False
        try:
            vfs.VfsLfs2.mkfs(partition)
            if app._settings['logging'].v:
                print("H:EEPROM formatted")
        except Exception as e:
            print(f"H:Error formatting: {e}")
            return False
        try:
            mountpoint = '/hexpansion_' + str(port)
            vfs.mount(partition, mountpoint, readonly=False)
            if app._settings['logging'].v:
                print("H:EEPROM initialised")
        except OSError as e:
            if e.args[0] == 1:
                if app._settings['logging'].v:
                    print("H:EEPROM initialised")
            else:
                print(f"H:Error mounting: {e}")
                return False
        except Exception as e:
            print(f"H:Error mounting: {e}")
            return False
        return True

    def erase_eeprom(self, port, addr):
        app = self.app
        if app._settings['logging'].v:
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
        except Exception as e:
            print(f"H:Error erasing EEPROM: {e}")
            return False
        return True

    def find_hexpansion_app(self, port):
        app = self.app
        expected_app_name = app._HEXPANSION_TYPES[app.hexpansion_slot_type[port - 1]].app_name
        for an_app in scheduler.apps:
            if type(an_app).__name__ == expected_app_name:
                if hasattr(an_app, "config") and hasattr(an_app.config, "port") and an_app.config.port == port:
                    return an_app
        return None

    # ------------------------------------------------------------------
    # Per-tick update  (state machine for hexpansion management)
    # ------------------------------------------------------------------

    def update(self, delta):
        app = self.app
        from .linefollower import (STATE_INIT, STATE_WARNING, STATE_LOGO,
            STATE_ERROR, STATE_MESSAGE, STATE_REMOVED, STATE_PROGRAMMING,
            STATE_DETECTED, STATE_ERASE, STATE_UPGRADE, STATE_CHECK,
            STATE_MENU, _MINIMISE_VALID_STATES, _EEPROM_ADDR)

        if app.current_state == STATE_INIT:
            self.scan_ports()
            if (len(app.ports_with_hexdrive) == 0) and (len(app.ports_with_blank_eeprom) == 0):
                app._animation_counter = 0
                app.current_state = STATE_WARNING
            else:
                app.current_state = STATE_CHECK
                if self._check_hexpansion_erase_port(delta):
                    pass
            return

        if app.hexpansion_update_required:
            app.hexpansion_update_required = False
            if app.current_state != STATE_CHECK:
                print("H:Hexpansion Check")
                app.set_menu(None)
                app.current_state = STATE_CHECK

        if app.current_state == STATE_WARNING or app.current_state == STATE_LOGO:
            app._update_state_warning(delta)
        elif app.current_state == STATE_ERROR or app.current_state == STATE_MESSAGE or app.current_state == STATE_REMOVED:
            app._update_state_error(delta)
        elif app.current_state == STATE_PROGRAMMING:
            self._update_state_programming(delta)
        elif app.current_state == STATE_DETECTED:
            self._update_state_detected(delta)
        elif app.current_state == STATE_ERASE:
            self._update_state_erase(delta)
        elif app.current_state == STATE_UPGRADE:
            self._update_state_upgrade(delta)
        elif app.current_state in _MINIMISE_VALID_STATES:
            if self._check_hexpansion_ports(delta):
                pass
            elif self._check_hexdrive_ports(delta):
                pass
            elif app.current_state == STATE_CHECK:
                self._update_state_check(delta)

    # ------------------------------------------------------------------
    # Individual state handlers
    # ------------------------------------------------------------------

    def _update_state_programming(self, delta):
        app = self.app
        from .linefollower import (STATE_UPGRADE, STATE_MESSAGE, STATE_ERROR,
            STATE_CHECK, _EEPROM_ADDR)

        if app.upgrade_port is not None:
            if app._HEXPANSION_TYPES[app.hexpansion_init_type].app_mpy_name is None:
                app.notification = Notification("No App", port=app.upgrade_port)
                app.error_message = ["No App", "for this", "Hexpansion"]
                app.current_state = STATE_MESSAGE
            elif self.update_app_in_eeprom(app.upgrade_port, _EEPROM_ADDR):
                app.notification = Notification("Upgraded", port=app.upgrade_port)
                eventbus.emit(HexpansionInsertionEvent(app.upgrade_port))
                app.error_message = ["Upgraded:", "Please", "reboop"]
                app.current_state = STATE_MESSAGE
                if app._settings['logging'].v:
                    print(f"H:Hexpansion on port {app.upgrade_port} upgraded")
            else:
                app.notification = Notification("Failed", port=app.upgrade_port)
                app.error_message = ["Hexpansion", "programming", "failed"]
                app.current_state = STATE_ERROR
            app.upgrade_port = None
        elif app.detected_port is not None:
            if self.prepare_eeprom(app.detected_port, _EEPROM_ADDR):
                app.notification = Notification("Initialised", port=app.detected_port)
                app.upgrade_port = app.detected_port
                app.hexpansion_slot_type[app.detected_port - 1] = app.hexpansion_init_type
                app.current_state = STATE_UPGRADE
            else:
                app.notification = Notification("Failed", port=app.detected_port)
                app.error_message = ["EEPROM", "initialisation", "failed"]
                app.hexpansion_slot_type[app.detected_port - 1] = None
                app.current_state = STATE_ERROR
            app.detected_port = None
        elif app._settings['logging'].v:
            print("H:Error - no port to program")

    def _update_state_detected(self, delta):
        app = self.app
        from .linefollower import STATE_PROGRAMMING, STATE_CHECK
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.current_state = STATE_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app._settings['logging'].v:
                print("H:Initialise Cancelled")
            app.detected_port = None
            app.current_state = STATE_CHECK
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            app.hexpansion_init_type = (app.hexpansion_init_type + 1) % len(app._HEXPANSION_TYPES)
            app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            app.hexpansion_init_type = (app.hexpansion_init_type - 1) % len(app._HEXPANSION_TYPES)
            app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            app.hexpansion_init_type = 1
            app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            app.hexpansion_init_type = 2
            app._refresh = True

    def _update_state_erase(self, delta):
        app = self.app
        from .linefollower import (STATE_MESSAGE, STATE_ERROR, STATE_CHECK,
            _EEPROM_ADDR)
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            if self.erase_eeprom(app.erase_port, _EEPROM_ADDR):
                app.error_message = ["Erased:", "Please", "reboop"]
                app.notification = Notification("Erased", port=app.erase_port)
                app.ports_with_blank_eeprom.add(app.erase_port)
                app.current_state = STATE_MESSAGE
            else:
                app.notification = Notification("Failed", port=app.erase_port)
                app.error_message = ["EEPROM", "erasure", "failed"]
                app.current_state = STATE_ERROR
            if app.hexdrive_port == app.erase_port:
                app.hexdrive_port = None
                app.hexdrive_app = None
                if app._settings['logging'].v:
                    print(f"H:HexDrive on port {app.erase_port} erased!")
                app.ports_with_hexdrive.discard(app.erase_port)
            if app._line_sensors_hexpansion_config is not None and app._line_sensors_hexpansion_config.port == app.erase_port:
                app._line_sensors_hexpansion_config = None
                app.ports_with_hexsense.discard(app.erase_port)
            app.erase_port = None
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if app._settings['logging'].v:
                print("H:Erase Cancelled")
            app.button_states.clear()
            app.erase_port = None
            app.current_state = STATE_CHECK

    def _update_state_upgrade(self, delta):
        app = self.app
        from .linefollower import STATE_PROGRAMMING, STATE_CHECK
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.notification = Notification("Upgrading", port=app.upgrade_port)
            app.current_state = STATE_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if app._settings['logging'].v:
                print("H:Upgrade Cancelled")
            app.button_states.clear()
            app.upgrade_port = None
            app.current_state = STATE_CHECK

    def _update_state_check(self, delta):
        app = self.app
        from .linefollower import (STATE_MENU, STATE_ERROR, STATE_REMOVED,
            STATE_WARNING, _APP_VERSION)
        if 0 < len(app.ports_with_latest_hexdrive):
            if app.hexdrive_port is not None and app.hexdrive_port not in app.ports_with_latest_hexdrive:
                print(f"Check: {app.hexdrive_port} lost")
                app.hexdrive_port = None
                app.hexdrive_app = None
            if app.hexdrive_port is None:
                valid_port = next(iter(app.ports_with_latest_hexdrive))
                hexdrive_app = self.find_hexpansion_app(valid_port)
                if hexdrive_app is not None:
                    app.hexdrive_port = valid_port
                    app.hexdrive_app = hexdrive_app
                    if app.hexpansion_slot_type[valid_port - 1] is not None:
                        app.num_motors = app._HEXPANSION_TYPES[app.hexpansion_slot_type[valid_port - 1]].motors
                        app.num_servos = app._HEXPANSION_TYPES[app.hexpansion_slot_type[valid_port - 1]].servos
                        app.num_steppers = app._HEXPANSION_TYPES[app.hexpansion_slot_type[valid_port - 1]].steppers
                    if (0 < app._HEXPANSION_TYPES[app.hexpansion_slot_type[valid_port - 1]].steppers) or app.hexdrive_app.get_status():
                        if app._settings['logging'].v:
                            print(f"H:HexDrive [{valid_port}] OK")
                        app.current_state = STATE_MENU
                        app._animation_counter = 0
                    else:
                        if app._settings['logging'].v:
                            print(f"H:HexDrive {valid_port}: Failed to initialise PWM resources")
                        app.error_message = [f"HexDrive {valid_port}", "PWM Init", "Failed", "Please", "Reboop"]
                        app.current_state = STATE_ERROR
                else:
                    if app._settings['logging'].v:
                        print(f"H:HexDrive {valid_port}: App not found, please reboop")
                    app.error_message = [f"HexDrive {valid_port}", "App not found.", "Please", "reboop"]
                    app.current_state = STATE_ERROR
            else:
                app.current_state = STATE_MENU
        elif app.hexdrive_port is not None:
            print(f"Check: {app.hexdrive_port} lost")
            app.hexdrive_port = None
            app.hexdrive_app = None
            app.current_state = STATE_REMOVED
        else:
            app._animation_counter = 0
            app.current_state = STATE_WARNING

    def _check_hexpansion_erase_port(self, delta):
        app = self.app
        from .linefollower import STATE_ERASE
        erase_port = app._settings['erase_slot'].v
        if erase_port != 0 and erase_port not in app.ports_with_blank_eeprom:
            try:
                hexpansion_header = read_header(erase_port, addr_len=_EEPROM_NUM_ADDRESS_BYTES)
            except OSError:
                return False
            except RuntimeError:
                if _IS_SIMULATOR:
                    return False
                pass
            if app._settings['logging'].v:
                print(f"H:Hexpansion on port {erase_port} Erase?")
            app.erase_port = erase_port
            app.notification = Notification("Erase?", port=app.erase_port)
            app.current_state = STATE_ERASE
            return True
        return False

    def _check_hexpansion_ports(self, delta):
        app = self.app
        from .linefollower import STATE_DETECTED
        if 0 < len(app.ports_with_blank_eeprom):
            app.detected_port = app.ports_with_blank_eeprom.pop()
            app.notification = Notification("Initialise?", port=app.detected_port)
            app.current_state = STATE_DETECTED
            return True
        return False

    def _check_hexdrive_ports(self, delta):
        app = self.app
        from .linefollower import STATE_CHECK, STATE_UPGRADE
        if app.waiting_app_port is not None or (0 < len(app.ports_with_hexdrive)):
            if app.waiting_app_port is None:
                app.waiting_app_port = app.ports_with_hexdrive.pop()
                app._animation_counter = 0
                hexpansion_app = self.find_hexpansion_app(app.waiting_app_port)
                if hexpansion_app is not None:
                    try:
                        hexpansion_app_version = hexpansion_app.get_version()
                    except Exception as e:
                        hexpansion_app_version = 0
                        print(f"H:Error getting Hexpansion app version - assume old: {e}")
                elif 5.0 < app._animation_counter:
                    if app._settings['logging'].v:
                        print("H:Timeout waiting for Hexpansion app to be started - assume it needs upgrading")
                    hexpansion_app_version = 0
                else:
                    if 0 == app._animation_counter:
                        if app._settings['logging'].v:
                            print(f"H:No app found on port {app.waiting_app_port} - WAITING for app to appear in Scheduler")
                    app.notification = Notification("Checking...", port=app.waiting_app_port)
                    app._animation_counter += delta / 1000
                    return True
                if hexpansion_app_version == app._HEXPANSION_TYPES[app.hexpansion_slot_type[app.waiting_app_port - 1]].app_mpy_version:
                    if app._settings['logging'].v:
                        print(f"H:Hexpansion on port {app.waiting_app_port} has latest App")
                    app.ports_with_latest_hexdrive.add(app.waiting_app_port)
                    app.current_state = STATE_CHECK
                else:
                    if app._settings['logging'].v:
                        print(f"H:Hexpansion app on port {app.waiting_app_port} needs upgrading from version {hexpansion_app_version} to {app._HEXPANSION_TYPES[app.hexpansion_slot_type[app.waiting_app_port - 1]].app_mpy_version}")
                    app.upgrade_port = app.waiting_app_port
                    app.notification = Notification("Upgrade?", port=app.upgrade_port)
                    app.current_state = STATE_UPGRADE
            app.waiting_app_port = None
            app._animation_counter = 0
            return True
        return False

    # ------------------------------------------------------------------
    # Draw hexpansion-related states
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render UI for hexpansion-related states.  Returns True if handled."""
        app = self.app
        from .linefollower import (STATE_DETECTED, STATE_ERASE, STATE_UPGRADE,
            STATE_PROGRAMMING)

        if app.current_state == STATE_DETECTED:
            hexpansion_type = app._HEXPANSION_TYPES[app.hexpansion_init_type].name
            hexpansion_sub_type = app._HEXPANSION_TYPES[app.hexpansion_init_type].sub_type
            app.draw_message(ctx, ["Hexpansion", f"in slot {app.detected_port}:", "Init EEPROM as", hexpansion_sub_type, f"{hexpansion_type}?"], [(1, 1, 1), (1, 1, 1), (1, 1, 1), (0, 0, 1), (1, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", up_label="^", down_label="\u25BC", left_label=app._HEXPANSION_TYPES[1].name, right_label=app._HEXPANSION_TYPES[6].name, cancel_label="No")
            return True
        elif app.current_state == STATE_ERASE:
            hexpansion_type = app._HEXPANSION_TYPES[app.hexpansion_init_type].name
            app.draw_message(ctx, [hexpansion_type, f"in slot {app.erase_port}:", "Erase EEPROM?"], [(1, 1, 0), (1, 1, 1), (1, 0, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif app.current_state == STATE_UPGRADE:
            hexpansion_type = app._HEXPANSION_TYPES[app.hexpansion_init_type].name
            app.draw_message(ctx, [hexpansion_type, f"in slot {app.upgrade_port}:", "Upgrade", f"{hexpansion_type} app?"], [(1, 1, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif app.current_state == STATE_PROGRAMMING:
            hexpansion_type = app._HEXPANSION_TYPES[app.hexpansion_init_type].name
            app.draw_message(ctx, [f"{hexpansion_type}:", "Programming", "EEPROM", "Please wait..."], [(1, 1, 0), (1, 1, 1), (1, 1, 1), (1, 1, 1)], label_font_size)
            return True
        return False

