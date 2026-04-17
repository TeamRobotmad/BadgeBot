# Hexpansion & EEPROM Management Module for BadgeBot
#
# Handles detection, initialisation, programming, upgrading and erasure of
# HexDrive / HexSense hexpansion EEPROMs.
#
# Public interface (called by the main app):
#   __init__(app)       – wire up to the main BadgeBotApp instance
#   register_events()   – register hexpansion insertion/removal handlers on eventbus
#   unregister_events() – unregister hexpansion event handlers
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
from system.hexpansion.header import HexpansionHeader, write_header
from system.hexpansion.util import get_hexpansion_block_devices, detect_eeprom_addr
from system.scheduler import scheduler

_NUM_HEXPANSION_SLOTS = 6

# HexDrive Hexpansion constants
# EEPROM Constants
_DEFAULT_EEPROM_PAGE_SIZE = 32
_DEFAULT_EEPROM_TOTAL_SIZE = 64 * 1024 // 8

_IS_SIMULATOR = sys.platform != "esp32"

# Local sub-states (internal to Hexpansion Mgr)
_SUB_INIT            = 0           # Initial state on app startup, before first port scan
_SUB_CHECK           = 1           # Checks for EEPROMs and HexDrives
_SUB_DETECTED        = 2           # Hexpansion detected & ready for EEPROM initialisation
_SUB_ERASE_CONFIRM   = 3           # Confirmation prompt for EEPROM erase
_SUB_ERASE           = 4           # Hexpansion EEPROM erasing in progress
_SUB_UPGRADE_CONFIRM = 5           # Hexpansion ready for App upgrade
_SUB_PROGRAMMING     = 6           # Hexpansion EEPROM programming (Initialsation or Upgrade) in progress
_SUB_PORT_SELECT     = 7           # User selecting which hexpansion to erase (if multiple) in order to free up a slot for initialisation or upgrade    
_SUB_DONE            = 8           # Final state after successful initialisation or upgrade, before returning to menu

# Hexpansion states
# unknown, blank, unrecognised, recognised but no app, recognised with app
_HEXPANSION_STATE_UNKNOWN = 0
_HEXPANSION_STATE_EMPTY = 1
_HEXPANSION_STATE_FAULTY = 2
_HEXPANSION_STATE_BLANK = 3
_HEXPANSION_STATE_UNRECOGNISED = 4
_HEXPANSION_STATE_RECOGNISED = 5
_HEXPANSION_STATE_RECOGNISED_NO_APP = 6
_HEXPANSION_STATE_RECOGNISED_OLD_APP = 7
_HEXPANSION_STATE_RECOGNISED_APP_OK = 8
_HEXPANSION_STATE_NAMES = [
    "Unknown",
    "Empty",
    "Faulty",
    "Blank",
    "Unrecognised",
    "Recognised",
    "No App",
    "Old App",
    "App OK"
]


# EEPROM app programming outcomes
_APP_EEPROM_RESULT_FAILURE = 0
_APP_EEPROM_RESULT_MISSING = -1
_APP_EEPROM_RESULT_SUCCESSFUL_UPGRADE = 1
_APP_EEPROM_RESULT_SUCCESSFUL_WRITE = 2


_HEXDRIVE_REQUIRED_MESSAGE = ["Requires:","RobotMad HexDrive","github.com","/TeamRobotmad","/BadgeBot"]
_HEXDRIVE_REQUIRED_MESSAGE_COLOURS = [(1,1,0),(1,1,0),(0,1,1),(0,1,1),(0,1,1)]

# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):        # pylint: disable=unused-argument
    """Register hexpansion-management-specific settings in the shared settings dict."""
    return


# ---- Hexpansion management -------------------------------------------------

class HexpansionMgr:
    """Manages hexpansion detection, EEPROM programming, and upgrades.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance so that shared state
        (settings, hexdrive_apps, current_state …) can be
        read and written.
    """

    # Sub-states are defined at module level (_SUB_*); app-level state
    # routing is handled by the dispatch tables in app.py.

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self._sub_state: int = _SUB_INIT
        self._prev_state: int = _SUB_INIT
        self._interactive_mode: bool = False
        self._port_selected: int = 0
        self._port_selected_header = None            # HexpansionHeader for selected port
        self._port_detail_page: int = 0              # 0=vid/pid, 1=eeprom, 2=details (conditional)
        self._port_detail_page_count: int = 2        # 2 or 3 depending on whether details page is available
        self._hexpansion_app_startup_timer: int = 0
        self._hexpansion_type_by_slot: list[HexpansionType | None] = [None]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_state_by_slot: list[int] = [_HEXPANSION_STATE_UNKNOWN]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_eeprom_addr_len: list[int] = [None]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_eeprom_addr: list[int] = [None]*_NUM_HEXPANSION_SLOTS 
        self._hexpansion_init_type: int = 0
        self._detected_port: int | None = None
        self._waiting_app_port: int | None = None
        self._erase_port: int | None = None
        self._upgrade_port: int | None = None
        self._ports_to_initialise: set[int] = set() # ports with blank EEPROM which could be initialised
        self._ports_to_check_app: set[int] = set()  # ports with recognised hexpansion type which should be checked for the correct app before being used
        self._reboop_required: bool = False
        self._hexpansion_serial_number: int | None = None
        if self._logging:
            print("HexpansionMgr initialised")


    # ------------------------------------------------------------------

    def register_events(self):
        """Register hexpansion insertion/removal event handlers directly."""
        from system.hexpansion.events import HexpansionRemovalEvent
        eventbus.on_async(HexpansionInsertionEvent, self._handle_insertion, self._app)
        eventbus.on_async(HexpansionRemovalEvent, self._handle_removal, self._app)


    def unregister_events(self):
        """Unregister hexpansion event handlers."""
        from system.hexpansion.events import HexpansionRemovalEvent
        eventbus.remove(HexpansionInsertionEvent, self._handle_insertion, self._app)
        eventbus.remove(HexpansionRemovalEvent, self._handle_removal, self._app)


    # ------------------------------------------------------------------


    @property
    def logging(self) -> bool:
        """Get the current logging state."""
        return self._logging
    
    @logging.setter
    def logging(self, value: bool):
        """Set the logging state."""    
        self._logging = value


    # ------------------------------------------------------------------
    # Async event handlers (registered directly on eventbus)
    # ------------------------------------------------------------------

    async def _handle_removal(self, event):
        app = self._app
        self._hexpansion_type_by_slot[event.port - 1] = None
        self._hexpansion_state_by_slot[event.port - 1] = _HEXPANSION_STATE_EMPTY
        if event.port in self._ports_to_initialise:
            self._ports_to_initialise.remove(event.port)
        if event.port in self._ports_to_check_app:
            self._ports_to_check_app.remove(event.app)    

        if self._detected_port is not None  and event.port == self._detected_port:
            # The hexpansion that is currently being initialised has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True
        elif self._upgrade_port is not None and event.port == self._upgrade_port:
            # The hexpansion that is currently being upgraded has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True
        elif self._waiting_app_port is not None and event.port == self._waiting_app_port:
            # The hexpansion that is currently waiting for its app to start has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True
        elif self._erase_port is not None and event.port == self._erase_port:
            # The hexpansion that is currently waiting to be erased has been removed, so reset the state machine to wait for a new hexpansion to be detected.
            app.hexpansion_update_required = True            
        elif event.port in app.hexdrive_ports:
            # The HexDrive that is currently in use has been removed, so update the app state accordingly (which may trigger a fallback to a different HexDrive if available, or a warning if no HexDrive is available).
            app.hexpansion_update_required = True
        elif app.hexsense_port is not None and event.port == app.hexsense_port:
            # The HexSense that is currently in use has been removed, so update the app state accordingly (which may trigger a warning if sensors are required for the current mode).
            app.hexpansion_update_required = True
        elif app.hextest_port is not None and event.port == app.hextest_port:
            # The HexTest that is currently in use has been removed, so update the app state accordingly (which may trigger a warning if the HexTest is required for the current mode).
            app.hexpansion_update_required = True
        elif app.hexdiag_port is not None and event.port == app.hexdiag_port:
            # The HexDiag that is currently in use has been removed, so update the app state accordingly (which may trigger a warning if the HexDiag is required for the current mode).
            app.hexpansion_update_required = True
        elif app.hexgps_port is not None and event.port == app.hexgps_port:
            # The HexGPS that is currently in use has been removed, so update the app state accordingly (which may trigger a warning if the HexGPS is required for the current mode).
            app.hexpansion_update_required = True
        elif self._port_selected != 0 and event.port == self._port_selected:
            # The port that the user is currently selecting a hexpansion for has been removed
            app.hexpansion_update_required = True

        if app.hexpansion_update_required and self._logging:
            print(f"H:Hexpansion removed from port {event.port}")


    async def _handle_insertion(self, event):
        if self._logging:
            print(f"H:Hexpansion inserted into port {event.port}")
        if self._check_port_for_known_hexpansions(event.port) or event.port == self._port_selected:
            # A known hexpansion type has been detected on the inserted port, so trigger an update of the hexpansion management state machine to handle it.
            # Or the inserted port is the one currently being selected by the user in interactive mode, so we should also trigger an update to check if it's a valid hexpansion and update the UI accordingly.
            self._app.hexpansion_update_required = True


   # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter hexpansion management from the main menu."""
        app = self._app
        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()
        self._interactive_mode = True
        if self._logging:
            print("Entered Hexpansion Management mode")
        self._enter_port_select()
        return True
    

    def _enter_port_select(self):
        if self._port_selected == 0:
            self._port_selected = 1
        if self._logging:
            print(f"H:Entering port select mode, starting with port {self._port_selected}")    
        self._read_port_header(self._port_selected)
        self._sub_state = _SUB_PORT_SELECT
        self._app.refresh = True


    def _read_port_header(self, port: int):
        """Read the EEPROM header for the given port and set the default detail page."""
        try:
            self._port_selected_header = self._read_header(port)
        except (OSError, RuntimeError):
            self._port_selected_header = None
        self._update_detail_page_count()
        # Default to details page if available, otherwise vid/pid
        self._port_detail_page = self._PAGE_DETAILS if self._port_detail_page_count > 2 else self._PAGE_VID_PID


    def _update_detail_page_count(self):
        """Set page count to 3 if the selected port has a recognised type with sub_type or app_name, else 2, or 1 if blank EEPROM."""
        app = self._app
        type_idx = self._hexpansion_type_by_slot[self._port_selected - 1] if 1 <= self._port_selected <= _NUM_HEXPANSION_SLOTS else None
        if type_idx is not None and 0 <= type_idx <= app.BLANK_HEXPANSION_INDEX:
            ht = app.HEXPANSION_TYPES[type_idx]
            if ht.sub_type or ht.app_name:
                # Recognised type with sub_type or app_name, so show details page
                self._port_detail_page_count = 3
                self._port_detail_page = self._PAGE_DETAILS
            elif type_idx == app.BLANK_HEXPANSION_INDEX:
                # Blank EEPROM - no details to show, so only show the EEPROM page
                self._port_detail_page_count = 0
            elif type_idx == app.UNRECOGNISED_HEXPANSION_INDEX:
                # Unrecognised type - show vid/pid page and EEPROM page
                self._port_detail_page_count = 2
                self._port_detail_page = self._PAGE_VID_PID
        else:
            # Empty
            self._port_detail_page_count = 0


    # ------------------------------------------------------------------
    # Per-tick update  (state machine for hexpansion management)
    # ------------------------------------------------------------------

    def update(self, delta) -> bool:
        """Per-tick update for hexpansion management state machine."""
        app = self._app

        if self._sub_state == _SUB_INIT:
            if self._logging and self._port_selected == 0:
                print("H:Initial scan for hexpansions")
            if self._scan_ports():
                self._port_selected = 0
                self._sub_state = _SUB_CHECK
                #if (len(app.hexdrive_ports) == 0) and (len(self._ports_to_initialise) == 0):
                #    # no HexDrive and no hexpansion with blank EEPROM which could be initialised as a HexDrive.
                #    self._report_hexpansion_states()
                #    app.show_message(_HEXDRIVE_REQUIRED_MESSAGE, _HEXDRIVE_REQUIRED_MESSAGE_COLOURS, "warning")
        elif app.hexpansion_update_required:
            # This flag is set when a hexpansion-related event occurs that should trigger an update of the hexpansion management state machine (e.g. insertion/removal of a hexpansion).
            app.hexpansion_update_required = False
            if self._sub_state != _SUB_CHECK:
                if self._logging:
                    print("H:Hexpansion Check")
                app.set_menu(None)
                self._sub_state = _SUB_CHECK
                # the check will show a message which forces the main App out of the current state.
        elif self._sub_state == _SUB_DETECTED:
            self._update_state_detected(delta)
        elif self._sub_state == _SUB_ERASE_CONFIRM:
            self._update_state_erase_confirm(delta)
        elif self._sub_state == _SUB_ERASE:
            self._update_state_erase(delta)
        elif self._sub_state == _SUB_UPGRADE_CONFIRM:
            self._update_state_upgrade(delta)
        elif self._sub_state == _SUB_PROGRAMMING:
            self._update_state_programming(delta)  
        elif self._check_ports_to_initialise(delta):
            pass
        elif self._check_ports_to_upgrade(delta):
            pass
        elif self._sub_state == _SUB_CHECK:
            self._update_state_check(delta)
        elif self._sub_state == _SUB_PORT_SELECT:
            self._update_state_port_select(delta)

        #self._report_hexpansion_states()

        if self._sub_state != self._prev_state:
            if self._logging:
                print(f"H:HexpansionMgr State: {self._prev_state} -> {self._sub_state}")
            self._prev_state = self._sub_state

        if self._sub_state == _SUB_DONE:
            if self._logging:
                print("H:HexDrive OK")
            app.initialise_settings()
            app.return_to_menu()
            self._sub_state = _SUB_CHECK
        return True


    # ------------------------------------------------------------------
    # Individual state handlers
    # ------------------------------------------------------------------

    def _update_state_programming(self, delta):     # pylint: disable=unused-argument
        app = self._app

        if self._upgrade_port is not None:
            # There is a hexpansion ready for App (upgrade) programming
            # remember if the EEPROM already contains an app, so we can show a more appropriate message after programming (e.g. "Upgraded" vs "Initialised")

            if app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_name is None:
                app.notification = Notification("No App", port=self._upgrade_port)
                self._sub_state = _SUB_CHECK
                app.show_message(["No App", "for this", "Hexpansion"], [(1,1,0),(1,1,1),(1,0,0)])
            else:
                result = self._update_app_in_eeprom(self._upgrade_port)
                if result == _APP_EEPROM_RESULT_FAILURE:
                    app.notification = Notification("Failed", port=self._upgrade_port)
                    app.show_message(["Hexpansion", "programming", "failed", "Protected?"], [(1,0,0),(1,0,0),(1,0,0),(1,0,0)], "warning")
                    self._sub_state = _SUB_CHECK
                elif result == _APP_EEPROM_RESULT_MISSING:
                    app.notification = Notification("App Missing", port=self._upgrade_port)
                    app.show_message(["App file", "missing for", "this Hexpansion"], [(1,0,0),(1,0,0),(1,0,0)], "warning")
                    self._sub_state = _SUB_CHECK
                else:
                    upgrade_text = "Upgraded" if result == _APP_EEPROM_RESULT_SUCCESSFUL_UPGRADE else "Programmed"
                    app.notification = Notification(upgrade_text, port=self._upgrade_port)
                    eventbus.emit(HexpansionInsertionEvent(self._upgrade_port))
                    app.show_message([f"{upgrade_text}:", "Please", "reboop"], [(0,1,0),(1,1,1),(1,1,1)], "reboop")
            self._upgrade_port = None
        elif self._detected_port is not None:
            # There is a hexpansion ready for EEPROM initialisation
            if self._prepare_eeprom(self._detected_port):
                app.notification = Notification("Initialised", port=self._detected_port)
                self._hexpansion_type_by_slot[self._detected_port - 1] = self._hexpansion_init_type
                if app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_name is not None:
                    self._upgrade_port = self._detected_port
                    self._sub_state = _SUB_PROGRAMMING
                    self._hexpansion_state_by_slot[self._detected_port - 1] = _HEXPANSION_STATE_RECOGNISED
                else:
                    self._sub_state = _SUB_CHECK
                    app.show_message(["No App", "for this", "Hexpansion"], [(1,1,0),(1,1,1),(1,1,1)])
                    self._hexpansion_state_by_slot[self._detected_port - 1] = _HEXPANSION_STATE_RECOGNISED_NO_APP
            else:
                app.notification = Notification("Failed", port=self._detected_port)
                self._hexpansion_type_by_slot[self._detected_port - 1] = None
                self._hexpansion_state_by_slot[self._detected_port - 1] = _HEXPANSION_STATE_FAULTY
                app.show_message(["EEPROM", "initialisation", "failed", "Protected?"], [(1,0,0),(1,0,0),(1,0,0),(1,0,0)], "warning")
                self._sub_state = _SUB_CHECK
            self._detected_port = None
        elif self._logging:
            print("H:Error - no port to program")
            self._sub_state = _SUB_CHECK


    def _update_state_detected(self, delta):        # pylint: disable=unused-argument
        """ Allow User to select which sub-type they want to initialise the hexpansion as (if there are multiple sub-types with the same PID), and confirm or cancel the initialisation."""
        app = self._app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            self._sub_state = _SUB_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if self._logging:
                print("H:Initialise Cancelled")
            self._detected_port = None
            self._sub_state = _SUB_CHECK
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            self._hexpansion_init_type = (self._hexpansion_init_type + 1) % app.UNRECOGNISED_HEXPANSION_INDEX
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            self._hexpansion_init_type = (self._hexpansion_init_type - 1) % app.UNRECOGNISED_HEXPANSION_INDEX
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            # "Left" is a shortcut button to HexDrive
            self._hexpansion_init_type = app.HEXDRIVE_HEXPANSION_INDEX
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            # "Right" is a shortcut button to HexSense
            self._hexpansion_init_type = app.HEXSENSE_HEXPANSION_INDEX
            app.refresh = True


    def _update_state_erase_confirm(self, delta):       # pylint: disable=unused-argument
        """ Allow User to confirm or cancel EEPROM erasure."""
        app = self._app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            if self._logging:
                print(f"H:Erase Confirmed on port {self._erase_port}")
            app.button_states.clear()
            self._sub_state = _SUB_ERASE
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if self._logging:
                print("H:Erase Cancelled")
            app.button_states.clear()
            self._erase_port = None
            self._sub_state = _SUB_CHECK


    def _update_state_erase(self, delta):       # pylint: disable=unused-argument
        """ Perform EEPROM erasure, and update app state accordingly (e.g. if the erased hexpansion is currently in use or being initialised/upgraded, reset those states).
            Unresponsive to buttons during the erasure process."""
        app = self._app
        if self._logging:
            print(f"H:Erasing EEPROM on port {self._erase_port}")        
        eeprom_page_size=app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_page_size if self._hexpansion_init_type > 0 else _DEFAULT_EEPROM_PAGE_SIZE
        eeprom_total_size=app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_total_size if self._hexpansion_init_type > 0 else _DEFAULT_EEPROM_TOTAL_SIZE
        if self._logging:
            print(f"H:Erase {self._hexpansion_init_type} page size: {eeprom_page_size} bytes, total size: {eeprom_total_size} bytes, addr_len: {self._hexpansion_eeprom_addr_len[self._erase_port-1]}, addr: {hex(self._hexpansion_eeprom_addr[self._erase_port-1])}")
        if self._erase_eeprom(self._erase_port,
                              self._hexpansion_eeprom_addr[self._erase_port-1],
                              self._hexpansion_eeprom_addr_len[self._erase_port-1],
                              eeprom_total_size,
                              eeprom_page_size):
            app.notification = Notification("Erased", port=self._erase_port)
            self._hexpansion_type_by_slot[self._erase_port - 1] = app.BLANK_HEXPANSION_INDEX
            self._hexpansion_state_by_slot[self._erase_port - 1] = _HEXPANSION_STATE_BLANK
            hexpansion_type = self._type_name_for_port(self._erase_port)
            app.show_message([hexpansion_type, f"in slot {self._erase_port}:", "Erased"], [(1,1,0), (1,1,1), (0,1,0)], "hexpansion")
            self._sub_state = _SUB_DETECTED
            self._detected_port = self._erase_port
        else:
            app.notification = Notification("Failed", port=self._erase_port)
            app.show_message(["EEPROM", "erasure", "failed", "Protected?"], [(1,0,0),(1,0,0),(1,0,0),(1,0,0)], "warning")
            self._sub_state = _SUB_CHECK        
        self._reboop_required = True
        
        if self._erase_port is not None and self._erase_port in app.hexdrive_ports:
            hexdrive_index = app.hexdrive_ports.index(self._erase_port)
            del app.hexdrive_ports[hexdrive_index]
            if hexdrive_index < len(app.hexdrive_apps):
                del app.hexdrive_apps[hexdrive_index]
            app.motor_controller = None
            if self._logging:
                print(f"H:HexDrive on port {self._erase_port} erased!")

        if app.hexsense_port is not None and app.hexsense_port == self._erase_port:
            app.hexsense_port = None
            if self._logging:
                print(f"H:HexSense on port {self._erase_port} erased!")

        if app.hextest_port is not None and app.hextest_port == self._erase_port:
            app.hextest_port = None
            if self._logging:
                print(f"H:HexTest on port {self._erase_port} erased!")

        if app.hexdiag_port is not None and app.hexdiag_port == self._erase_port:
            app.hexdiag_port = None
            if self._logging:
                print(f"H:HexDiag on port {self._erase_port} erased!")

        if app.hexgps_port is not None and app.hexgps_port == self._erase_port:
            app.hexgps_port = None
            if self._logging:
                print(f"H:HexGPS on port {self._erase_port} erased!")

        self._erase_port = None


    def _update_state_upgrade(self, delta):     # pylint: disable=unused-argument
        """ Allow User to confirm or cancel App upgrade."""
        app = self._app
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.notification = Notification("Upgrading", port=self._upgrade_port)
            self._sub_state = _SUB_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if self._logging:
                print("H:Upgrade Cancelled")
            app.button_states.clear()
            self._hexpansion_state_by_slot[self._upgrade_port - 1] = _HEXPANSION_STATE_RECOGNISED_OLD_APP
            self._upgrade_port = None
            self._sub_state = _SUB_CHECK
        """ Return the 1-based port number of a hexpansion of the given type, or None if no such hexpansion is currently detected."""
        for port in range(0, _NUM_HEXPANSION_SLOTS):
            if self._hexpansion_type_by_slot[port] == hexpansion_type:
                return port+1
        return None


    def _report_hexpansion_states(self):
        """ Utility function to print the current detected hexpansion types and states for each port, for debugging purposes."""
        app = self._app
        for port in range(0, _NUM_HEXPANSION_SLOTS):
            type_idx = self._hexpansion_type_by_slot[port]
            type_name = app.HEXPANSION_TYPES[type_idx].name if type_idx is not None else "None"
            state_name = _HEXPANSION_STATE_NAMES[self._hexpansion_state_by_slot[port]]
            print(f"Port {port+1}: Type={type_name}, State={state_name}")
        # _ports_to_initialise and _ports_to_check_app are also useful to report as they indicate hexpansions that have been detected but not yet fully processed, which can help with debugging issues around hexpansion detection and initialisation.
        print(f"Ports to initialise: {self._ports_to_initialise}")
        print(f"Ports to check app: {self._ports_to_check_app}")
        print(f"hexsense_port:{app.hexsense_port}")
        print(f"hextest_port:{app.hextest_port}")
        print(f"hexgps_port:{app.hexgps_port}")
        print(f"hexdiag_port:{app.hexdiag_port}")
        print(f"hexdrive_ports:{app.hexdrive_ports}")

    def _check_hexpansion(self, port: int | None, type_index: int) -> tuple[int | None, object | None]:
        """ Check if the currently configured hexpansion of the given type is still present, and
            if not, check if there is another hexpansion of the same type that can be switched to, or if the hexpansion has been removed entirely."""
        app = self._app
        hexpansion_app = None   
        hexpansion_was_present = port is not None
        if hexpansion_was_present:
            if self._hexpansion_type_by_slot[port - 1] != type_index:
                old_port = port
                port = None
        if port is None:
            name = app.HEXPANSION_TYPES[type_index].name
            new_port = self._get_hexpansion_by_type(type_index)
            if new_port is not None:
                if hexpansion_was_present:
                    if self._logging:
                        print(f"H:{name} moved from port {old_port} to port {new_port}")
                    app.show_message([f"{name} moved", f"to port {new_port}"], [(1,1,0),(1,1,1)])
                else:
                    if self._logging:
                        print(f"H:{name} found on port {new_port}")
                    #app.show_message([f"{name}", f"on port {new_port}"], [(1,1,0),(1,1,1)])
                port = new_port
            elif hexpansion_was_present:            
                if self._logging:
                    print(f"H:{name} on port {old_port} lost")
                if not self._interactive_mode:    
                    app.show_message([f"{name}","removed.","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], "error")
            # if the hexpansion type has an app then we need to find it
                        actual_version = hexpansion_app.get_version()
                        expected_version = app.HEXPANSION_TYPES[type_index].app_mpy_version
                        if actual_version != expected_version:
                            if self._logging:
                                print(f"Check: {app.HEXPANSION_TYPES[type_index].name} app on port {port} has version {actual_version}, expected {expected_version}")
                # shouldn't be needed self._hexpansion_type_by_slot[port - 1] = type_index
                self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED
                if app.HEXPANSION_TYPES[type_index].app_name is not None:
                    hexpansion_app = self._check_hexpansion_app_on_port(port, type_index)
                else:
                    self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED_NO_APP

        return port, hexpansion_app


    def _update_state_check(self, delta):       # pylint: disable=unused-argument
        """ Check for HexDrive & HexSense presence and update app state accordingly."""
        app = self._app

        self._report_hexpansion_states()

        hexdiag_port = app.hexdiag_port
        hextest_port = app.hextest_port

        app.hexsense_port, _                = self._check_hexpansion(app.hexsense_port, app.HEXSENSE_HEXPANSION_INDEX)
        app.hextest_port, _                 = self._check_hexpansion(app.hextest_port, app.HEXTEST_HEXPANSION_INDEX)
        app.hexgps_port, _                  = self._check_hexpansion(app.hexgps_port, app.HEXGPS_HEXPANSION_INDEX)
        app.hexdiag_port, _                 = self._check_hexpansion(app.hexdiag_port, app.HEXDIAG_HEXPANSION_INDEX)

        if hexdiag_port != app.hexdiag_port:
            # Update the diagnostics app reference if the HexDiag has moved or been removed/added
            app._sensor_test_mgr.hextest_setup(app.hexdiag_port) 

        if hextest_port != app.hextest_port and app._sensor_test_mgr is not None:
            # Update the test app reference if the HexTest has moved or been removed/added
            app._sensor_test_mgr.hextest_setup(app.hextest_port) 

        # HexDrive is more complex as we track multiple instances rather than just the first one.
        hexdrive_ports = app.hexdrive_ports.copy()
        hexdrive_apps = app.hexdrive_apps.copy()
        hexdrive_types = [self._hexpansion_type_by_slot[port - 1] for port in hexdrive_ports]
        # have any of the currently tracked HexDrives been removed or changed to a different type? (e.g. due to user reinserting them, or a new hexpansion being detected on the same port)
        for current_port, current_app, hexdrive_type in zip(hexdrive_ports, hexdrive_apps, hexdrive_types):
            hexdrive_port, hexdrive_app = self._check_hexpansion(current_port, hexdrive_type)
            if current_port != hexdrive_port:
                # The HexDrive that was previously on this port has been removed or is no longer recognised as a HexDrive, so remove it
                if self._logging:
                    print(f"H:HexDrive on port {current_port} removed from list")
                app.hexdrive_ports.remove(current_port)
                app.hexdrive_apps.remove(current_app)
                app.motor_controller = None

        # Now check if there are any new HexDrives that we haven't added to the list yet
        # check for each type of hexpansion that can be a HexDrive
        for type_index in app.hexdrive_hexpansion_types:
            print(f"H:Checking for HexDrive type index {type_index}")
            current_port, current_app = self._check_hexpansion(None, type_index)
            if current_port is not None and current_port not in app.hexdrive_ports:
                if self._logging:
                    print(f"H:HexDrive on port {current_port} added to list")
                app.hexdrive_ports.append(current_port)
            if current_app is not None and current_app not in app.hexdrive_apps:
                if self._logging:
                    print(f"H:HexDrive App on port {current_port} added to list")
                app.hexdrive_apps.append(current_app)

        if len(app.hexdrive_ports) > 0 and len(app.hexdrive_apps) > 0:
            hexdrive_type_idx = self._hexpansion_type_by_slot[app.hexdrive_ports[0] - 1]
            app.num_motors = app.HEXPANSION_TYPES[hexdrive_type_idx].motors
            app.num_servos = app.HEXPANSION_TYPES[hexdrive_type_idx].servos
            app.num_steppers = app.HEXPANSION_TYPES[hexdrive_type_idx].steppers

            # Create the high-level MotorController for IMU-aided driving
            # (only when the HexDrive has motors)
            if app.num_motors > 1 and app.motor_controller is None:
                try:
                    from .motor_controller import MotorController
                    app.motor_controller = MotorController(
                        app.hexdrive_apps[0], app.settings,
                        logging=self._logging,
                        front_face_setting=app.settings.get('front_face'),
                        apply_motor_directions_callback=app.apply_motor_directions,
                    )
                except Exception as e:      # pylint: disable=broad-except
                    print(f"H:MotorController init failed: {e}")
                    app.motor_controller = None
            elif app.motor_controller is not None and app.num_motors == 0:
                app.motor_controller = None

            if not self._interactive_mode and app.hexdrive_apps[0].get_status():
                # In non-interactive mode, if we have a HexDrive and its app is running, we can proceed straight to the main menu
                self._sub_state = _SUB_DONE
        elif self._interactive_mode:
            self._enter_port_select()
        elif self._reboop_required:
            app.show_message(["Please", "reboop"], [(1,1,1),(1,1,1)], "reboop")        
        else:
            app.show_message(_HEXDRIVE_REQUIRED_MESSAGE, _HEXDRIVE_REQUIRED_MESSAGE_COLOURS, "warning")

 
    def _update_state_port_select(self, delta: int):   # pylint: disable=unused-argument
        app = self._app
      
        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            app.button_states.clear()
            self._port_selected = (self._port_selected % _NUM_HEXPANSION_SLOTS) + 1
            self._read_port_header(self._port_selected)
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            app.button_states.clear()
            self._port_selected = ((self._port_selected - 2) % _NUM_HEXPANSION_SLOTS) + 1
            self._read_port_header(self._port_selected)
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.refresh = True
            if self._hexpansion_state_by_slot[self._port_selected - 1] == _HEXPANSION_STATE_BLANK:
                # The selected port has a blank EEPROM, so we can initialise it without erasing first.
                self._detected_port = self._port_selected
                app.notification = Notification("Init?", port=self._detected_port)
                self._sub_state = _SUB_DETECTED
            elif self._hexpansion_state_by_slot[self._port_selected - 1] >= _HEXPANSION_STATE_FAULTY:
                # The selected port has a non-blank EEPROM with a detected hexpansion type, so we need to erase it before we can initialise or upgrade it.
                self._erase_port = self._port_selected
                app.notification = Notification("Erase?", port=self._erase_port)
                self._sub_state = _SUB_ERASE_CONFIRM
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            if self._port_detail_page_count > 0:
                self._port_detail_page = (self._port_detail_page - 1) % self._port_detail_page_count
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            if self._port_detail_page_count > 0:
                self._port_detail_page = (self._port_detail_page + 1) % self._port_detail_page_count
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            self._interactive_mode = False
            self._port_selected = 0
            app.button_states.clear()
            app.return_to_menu()


    # ------------------------------------------------------------------
    # Draw hexpansion-related states
    # ------------------------------------------------------------------

    def _type_name_for_port(self, port: int, fallback_type_idx: int | None = None) -> str:
        """Return detected type name for a port, falling back to a selected type index."""
        ignore_blank_eeprom = 1 if fallback_type_idx is not None else 0
        if port is not None and 1 <= port <= _NUM_HEXPANSION_SLOTS:
            type_idx = self._hexpansion_type_by_slot[port - 1]
            if type_idx is not None and 0 <= type_idx < len(self._app.HEXPANSION_TYPES)-ignore_blank_eeprom:
                return self._app.HEXPANSION_TYPES[type_idx].name
        if fallback_type_idx is not None:
            # Blank EEPROM is reported as the fallback type - because that is what the user is selecting during initialisation
            return self._app.HEXPANSION_TYPES[fallback_type_idx].name
        return "Empty"


    def draw(self, ctx) -> bool:
        """Render UI for hexpansion-related states.  Returns True if handled."""
        app = self._app
        if self._sub_state == _SUB_DETECTED:
            hexpansion_type = app.HEXPANSION_TYPES[self._hexpansion_init_type].name
            hexpansion_sub_type = app.HEXPANSION_TYPES[self._hexpansion_init_type].sub_type
            app.draw_message(ctx, ["Hexpansion", f"in slot {self._detected_port}:", "Init EEPROM as", hexpansion_type, f"{hexpansion_sub_type if hexpansion_sub_type else ''}?"], [(1, 1, 0), (1, 1, 0), (1, 1, 0), (1, 0, 1), (1, 0, 1)], label_font_size)
            button_labels(ctx, confirm_label="Yes", up_label=app.special_chars['up'], down_label="\u25BC", left_label=app.HEXPANSION_TYPES[app.HEXDRIVE_HEXPANSION_INDEX].name, right_label=app.HEXPANSION_TYPES[app.HEXSENSE_HEXPANSION_INDEX].name, cancel_label="No")
            return True
        elif self._sub_state == _SUB_PORT_SELECT:
            self._draw_port_select(ctx)
            return True
        elif self._sub_state == _SUB_ERASE_CONFIRM:
            hexpansion_type_name = self._type_name_for_port(self._erase_port, self._hexpansion_init_type)
            # TODO IF WE DON'T KNOW THE EEPROM TYPE then show what we propose and allow user to select from common options...
            app.draw_message(ctx, [hexpansion_type_name, f"in slot {self._erase_port}:", "Erase EEPROM?"], [(1, 0, 1), (1, 1, 0), (1, 0, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif self._sub_state == _SUB_ERASE:
            hexpansion_type_name = self._type_name_for_port(self._erase_port, self._hexpansion_init_type)
            app.draw_message(ctx, [hexpansion_type_name, f"in slot {self._erase_port}:", "Erasing..."], [(1, 0, 1), (1, 1, 0), (1, 0, 0)], label_font_size)
            return True        
        elif self._sub_state == _SUB_UPGRADE_CONFIRM:
            hexpansion_type_name = self._type_name_for_port(self._upgrade_port, self._hexpansion_init_type)
            app.draw_message(ctx, [hexpansion_type_name, f"in slot {self._upgrade_port}:", "Upgrade", f"{hexpansion_type_name} app?"], [(1, 0, 1), (1, 1, 0), (1, 1, 0), (1, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif self._sub_state == _SUB_PROGRAMMING:
            # During upgrade, show the already-detected type for the selected port.
            hexpansion_type_name = self._type_name_for_port(self._upgrade_port, self._hexpansion_init_type)
            app.draw_message(ctx, [f"{hexpansion_type_name}:", f"in slot {self._upgrade_port}:", "Programming", "Please wait..."], [(1, 0, 1), (1, 1, 0), (1, 1, 0), (1, 1, 0)], label_font_size)
            return True
        return False
    
    


# Hexpansion/EEPROM information pages
    _PAGE_NAMES = ("VID/PID", "EEPROM", "Details")
    _PAGE_VID_PID = 0
    _PAGE_EEPROM = 1
    _PAGE_DETAILS = 2    

    def _draw_port_select(self, ctx):
        """Draw the port-select screen with paged details."""
        app = self._app
        hdr = self._port_selected_header
        hexpansion_state = _HEXPANSION_STATE_NAMES[self._hexpansion_state_by_slot[self._port_selected - 1]]
        if self._hexpansion_state_by_slot[self._port_selected - 1] > _HEXPANSION_STATE_BLANK:
            hexpansion_name = self._type_name_for_port(self._port_selected, None)
        else:
            hexpansion_name = ""
        up_label = down_label = ""
        if self._port_detail_page_count == 0:
            lines = [f"Slot {self._port_selected}", hexpansion_state, hexpansion_name]
            colours = [(1, 1, 0), (1, 0, 1), (0, 1, 1)]
        else:
            # Common header lines for all pages
            page = self._port_detail_page
            lines = [f"Slot {self._port_selected}-{self._PAGE_NAMES[page]}", hdr.friendly_name if hdr is not None else hexpansion_name]
            colours = [(1, 1, 0), (1, 0, 1)]
            if page == self._PAGE_VID_PID:
                # VID / PID page
                if hdr is not None:
                    lines += [f"VID: {hdr.vid:04X}", f"PID: {hdr.pid:04X}"]
                    colours += [(0, 1, 1), (0, 1, 1)]
                #else:
                #    lines = header_lines + ["No header"]
                #    colours = header_colours + [(1, 0, 0)]
            elif page == self._PAGE_EEPROM:
                # EEPROM parameters page
                if hdr is not None:
                    lines += [f"Size: {hdr.eeprom_total_size} Bytes", f"Page: {hdr.eeprom_page_size} Bytes"]
                    colours += [(0, 1, 1), (0, 1, 1)]
            else: # page == self._PAGE_DETAILS:
                # Details page (only when page_count == 3)
                type_idx = self._hexpansion_type_by_slot[self._port_selected - 1]
                ht = app.HEXPANSION_TYPES[type_idx] if type_idx is not None else None
                if ht is not None:
                    if ht.sub_type:
                        lines.append(ht.sub_type)
                        colours.append((0, 1, 1))
                    if ht.app_name:
                        lines.append(ht.app_name)
                        colours.append((0, 1, 1))
                        # Try to get running app version
                        running_app = self._find_hexpansion_app(self._port_selected)
                        if running_app is not None:
                            try:
                                ver = running_app.get_version()
                                lines.append(f"v{ver}")
                                colours.append((0, 1, 1))
                            except Exception:       # pylint: disable=broad-except
                                pass
                    else:
                        lines.append(hexpansion_state)
                        colours.append((0, 1, 1))        
            # Button labels: up/down show destination page names
            down_page = (page + 1) % self._port_detail_page_count
            up_page = (page - 1) % self._port_detail_page_count
            # Only show the DOWN label if there is more than 1 page, otherwise there is no other page to go to
            down_label=self._PAGE_NAMES[down_page] if self._port_detail_page_count > 1 else ""
            # only show the UP label if there are more than 2 pages, otherwise it would just show the same as the DOWN
            up_label=self._PAGE_NAMES[up_page] if self._port_detail_page_count > 2 else ""

        # Ensure there are always 5 lines to draw for consistent layout, even if some are blank
        while len(lines) < 5:
            lines.append("")
            colours.append((1, 1, 1))
        app.draw_message(ctx, lines, colours, label_font_size)

        confirm_label = "Init" if self._hexpansion_state_by_slot[self._port_selected - 1] == _HEXPANSION_STATE_BLANK else "Erase" if self._hexpansion_state_by_slot[self._port_selected - 1] >= _HEXPANSION_STATE_FAULTY else ""
        button_labels(ctx, confirm_label=confirm_label, left_label="<Slot", right_label="Slot>",
                    up_label=up_label, down_label=down_label, cancel_label="Back")



    # ------------------------------------------------------------------
    # Port scanning
    # ------------------------------------------------------------------

    def _scan_ports(self) -> bool:
        """Scan all ports one at a time for known hexpansions, and update app state accordingly.
           Returns True when all have been scanned (even if no hexpansions are detected), False if the scan is still in progress."""
        # use _port_selected as the iterator variable for which port we are currently scanning, starting at 1 and going up to _NUM_HEXPANSION_SLOTS
        if self._port_selected is None or self._port_selected > _NUM_HEXPANSION_SLOTS or self._port_selected < 1:
            self._port_selected = 1
        self._check_port_for_known_hexpansions(self._port_selected)
        self._port_selected += 1
        return self._port_selected > _NUM_HEXPANSION_SLOTS


    def _read_header(self, port: int, i2c: I2C | None=None) -> HexpansionHeader | None:
        if i2c is None:
            if port is None:
                return None
            i2c = I2C(port)
        addr_len = self._hexpansion_eeprom_addr_len[port-1]
        eeprom_addr = self._hexpansion_eeprom_addr[port-1]
        if addr_len is None:
            # Autodetect eeprom addr
            eeprom_addr, addr_len = detect_eeprom_addr(i2c)
            if eeprom_addr is None:
                print(f"H:Failed to detect EEPROM address on port {port}")
                return None
            self._hexpansion_eeprom_addr_len[port-1] = addr_len
            self._hexpansion_eeprom_addr[port-1] = eeprom_addr
        if addr_len is None or eeprom_addr is None:
            # Autodetect eeprom addr
            eeprom_addr, addr_len = detect_eeprom_addr(i2c)
        if eeprom_addr is None:
            print(f"H:Failed to detect EEPROM address on port {port}")
            print(f"H:Scan:{i2c.scan()}")  # debug - print all detected I2C addresses
            return None
        # Do we have a header?
        header_bytes = i2c.readfrom_mem(eeprom_addr, 0x00, 32, addrsize=8*addr_len)
        hexpansion_header = HexpansionHeader.from_bytes(header_bytes)
        print(f"H:Read header on port {port}: {hexpansion_header}")
        return hexpansion_header


    def _check_port_for_known_hexpansions(self, port) -> bool:
        """Check the given port for known hexpansion types by reading the EEPROM header, and update app state accordingly.
           Returns True if a known hexpansion type is detected (even if it was already known), False otherwise."""
        app = self._app
        if port not in range(1, _NUM_HEXPANSION_SLOTS + 1):
            return False
        try:
            if self.logging:
                print(f"H:Reading header on port {port}...")
            hexpansion_header = self._read_header(port)
        except OSError:
            # OSError just means there is no hexpansion EEPROM on this port
            self._hexpansion_type_by_slot[port - 1] = None
            self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_EMPTY
            return False
        except RuntimeError:
            # RuntimeError means there is a blank EEPROM
            #if _IS_SIMULATOR:
            #    return False
            if self._logging:
                print(f"H:Found EEPROM on port {port}")
            self._hexpansion_type_by_slot[port - 1] = app.BLANK_HEXPANSION_INDEX
            self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_BLANK
            self._ports_to_initialise.add(port)
            return True
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error reading header on port {port}: {e}")
            return False
        if hexpansion_header is None:
            print(f"H:Error reading header on port {port}")
            self._hexpansion_type_by_slot[port - 1] = None
            self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_EMPTY
            return False
        for index, hexpansion_type in enumerate(app.HEXPANSION_TYPES):
            if hexpansion_header.vid == hexpansion_type.vid and hexpansion_header.pid == hexpansion_type.pid:
                self._hexpansion_type_by_slot[port - 1] = index
                if self._hexpansion_state_by_slot[port - 1] < _HEXPANSION_STATE_RECOGNISED:
                    self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED
                if self._logging:
                    print(f"H:Found {hexpansion_type.name} {hexpansion_type.sub_type if hexpansion_type.sub_type else ''} on port {port}")
                if hexpansion_type.app_name is not None:
                    # This hexpansion should have an app, but do we have it and is it the right version?
                    self._check_hexpansion_app_on_port(port, index)
                    return True
                elif index == app.HEXSENSE_HEXPANSION_INDEX and app.hexsense_port is None:
                    return True
                elif index == app.HEXTEST_HEXPANSION_INDEX and app.hextest_port is None:
                    return True
                elif index == app.HEXGPS_HEXPANSION_INDEX and app.hexgps_port is None:
                    return True
                elif index == app.HEXDIAG_HEXPANSION_INDEX and app.hexdiag_port is None:
                    return True
                return False
        # Unrecognised Hexpansion
        if self._logging:
            # report VID/PID in hexadecimal
            print(f"H:Port {port} - VID/PID {hex(hexpansion_header.vid)}/{hex(hexpansion_header.pid)} not recognised") 
        self._hexpansion_type_by_slot[port - 1] = app.UNRECOGNISED_HEXPANSION_INDEX
        self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_UNRECOGNISED
        return False



    def _check_hexpansion_app_on_port(self, port: int, type_index: int) -> object | None:
        """Check if the app for the hexpansion on the given port is present and correct"""
        app = self._app
        hexpansion_app = self._find_hexpansion_app(port)
        if hexpansion_app is not None:
            # get version number from app and compare to expected version for this hexpansion type, and if it doesn't match, treat it as if the app is not present (i.e. show "App not found" message and prompt to reboop)
            try:
                version = hexpansion_app.get_version()
            except Exception as e:      # pylint: disable=broad-except
                print(f"H:Error getting app version for hexpansion on port {port}: {e}")
                version = None
            if version != app.HEXPANSION_TYPES[type_index].app_mpy_version:
                if self._logging:
                    print(f"H:{app.HEXPANSION_TYPES[type_index].name} app on port {port} has version {hexpansion_app.version}, expected {app.HEXPANSION_TYPES[type_index].app_mpy_version}")
                self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED_OLD_APP
                # add to upgrade list if not already there
                if port not in self._ports_to_check_app:
                    self._ports_to_check_app.add(port)
            else:
                self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED_APP_OK
            if self._logging:
                print(f"H:{app.HEXPANSION_TYPES[type_index].name} app found on port {port}")
        return hexpansion_app

    # ------------------------------------------------------------------
    # EEPROM operations
    # ------------------------------------------------------------------

    def _update_app_in_eeprom(self, port) -> int:
        """Copy the appropriate .mpy file to the hexpansion EEPROM on the given port.

        Returns one of:
          - _APP_EEPROM_RESULT_FAILURE
          - _APP_EEPROM_RESULT_MISSING
          - _APP_EEPROM_RESULT_SUCCESSFUL_UPGRADE
          - _APP_EEPROM_RESULT_SUCCESSFUL_WRITE
        """
        app = self._app
        if app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_name is None:
            if self._logging:
                print(f"H:Hexpansion type {app.HEXPANSION_TYPES[self._hexpansion_init_type].name} does not have an app to copy to EEPROM")
            return _APP_EEPROM_RESULT_FAILURE
        source_file = app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_name
        if self._logging:
            print(f"H:Writing app.mpy on port {port} with {source_file}")
        try:
            i2c = I2C(port)
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error opening I2C port {port}: {e}")
            return _APP_EEPROM_RESULT_FAILURE
        try:
            hexpansion_header = self._read_header(port, i2c=i2c)
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error reading header on port {port}: {e}")
            return _APP_EEPROM_RESULT_FAILURE
        if hexpansion_header is None:
            if self._logging:
                print(f"H:Error reading header on port {port}")
            return _APP_EEPROM_RESULT_FAILURE
        try:
            if self._logging:
                print(f"H:Getting block devices for port {port}, addr_len={self._hexpansion_eeprom_addr_len[port-1]}, addr={hex(self._hexpansion_eeprom_addr[port-1])}...")
            _, partition = get_hexpansion_block_devices(i2c, hexpansion_header, self._hexpansion_eeprom_addr[port-1], addr_len=self._hexpansion_eeprom_addr_len[port-1])
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return _APP_EEPROM_RESULT_FAILURE
        mountpoint = '/hexpansion_' + str(port)
        already_mounted = False
        if not already_mounted:
            if self._logging:
                print(f"H:Mounting {partition} at {mountpoint}")
            try:
                vfs.mount(partition, mountpoint, readonly=False)
            except OSError as e:
                if e.args[0] == 1:
                    already_mounted = True
                else:
                    print(f"H:Error mounting: {e}")
                    return _APP_EEPROM_RESULT_FAILURE
            except Exception as e:      # pylint: disable=broad-except
                print(f"H:Error mounting: {e}")
                return _APP_EEPROM_RESULT_FAILURE
        source_path = "/" + __file__.rsplit("/", 1)[0] + f"/{source_file}"
        dest_path = f"{mountpoint}/app.mpy"

        # Distinguish first write vs upgrade from whether delete succeeds.
        had_existing_app = True

        try:
            if self._logging:
                print(f"H:Deleting {dest_path}")
            os.remove(dest_path)
        except Exception as e:          # pylint: disable=broad-except
            if len(e.args) > 0 and e.args[0] == 2:
                # File not found, which is fine since we just want to ensure it's not there before copying the new file.
                had_existing_app = False
            else:    
                print(f"H:Error deleting {dest_path}: {e}")
                return _APP_EEPROM_RESULT_FAILURE

        if self._logging:
            print(f"H:Copying {source_path} to {dest_path}")

        try:
            template = open(source_path, "rb")
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error opening {source_path}: {e}")
            return _APP_EEPROM_RESULT_MISSING

        try:
            appfile = open(dest_path, "wb")
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error opening {dest_path}: {e}")
            return _APP_EEPROM_RESULT_FAILURE

        try:
            appfile.write(template.read())
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error writing Hexpansion: {e}")
            return _APP_EEPROM_RESULT_FAILURE

        try:
            appfile.close()
            template.close()
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Error closing files: {e}")
            return _APP_EEPROM_RESULT_FAILURE
        if not already_mounted:
            try:
                vfs.umount(mountpoint)
                if self._logging:
                    print(f"H:Unmounted {mountpoint}")
            except Exception as e:    # pylint: disable=broad-except
                print(f"H:Error unmounting {mountpoint}: {e}")
                return _APP_EEPROM_RESULT_FAILURE
        if self._logging:
            print(f"H:Hexpansion app.mpy written with version {self._app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_version}")
        if had_existing_app:
            return _APP_EEPROM_RESULT_SUCCESSFUL_UPGRADE
        return _APP_EEPROM_RESULT_SUCCESSFUL_WRITE


    # ------------------------------------------------------------------

    def _prepare_eeprom(self, port) -> bool:
        """Write the EEPROM header and format the filesystem on the hexpansion EEPROM on the given port, based on the selected hexpansion type.  Returns True if successful, False otherwise."""
        app = self._app
        if self._logging:
            print(f"H:Initialising EEPROM on port {port} as {self._hexpansion_init_type}:{app.HEXPANSION_TYPES[self._hexpansion_init_type].name}...")
        hexpansion_header_to_write = HexpansionHeader(
            manifest_version="2024",
            fs_offset=max(32, app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_page_size),
            eeprom_page_size=app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_page_size,
            eeprom_total_size=app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_total_size,
            vid=app.HEXPANSION_TYPES[self._hexpansion_init_type].vid,
            pid=app.HEXPANSION_TYPES[self._hexpansion_init_type].pid,
            unique_id=self._hexpansion_serial_number if self._hexpansion_serial_number is not None else 0,
            friendly_name=app.HEXPANSION_TYPES[self._hexpansion_init_type].name,
        )
        try:
            i2c = I2C(port)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error opening I2C port {port}: {e}")
            return False
        addr_len = self._hexpansion_eeprom_addr_len[port-1]
        addr = self._hexpansion_eeprom_addr[port-1]
        try:
            write_header(port, hexpansion_header_to_write, addr=addr, addr_len=addr_len, page_size=hexpansion_header_to_write.eeprom_page_size)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error writing header: port:{port}, addr={addr}, addr_len={addr_len}, page_size={hexpansion_header_to_write.eeprom_page_size}, error:{e} - device write protected?")
            return False
        try:
            hexpansion_header = self._read_header(port, i2c=i2c)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error reading header back: {e}")
            return False
        try:
            _, partition = get_hexpansion_block_devices(i2c, hexpansion_header, addr, addr_len=addr_len)
        except RuntimeError as e:
            print(f"H:Error getting block devices: {e}")
            return False
        try:
            vfs.VfsLfs2.mkfs(partition)
            if self._logging:
                print("H:EEPROM formatted")
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error formatting: {e}")
            return False
        try:
            mountpoint = '/hexpansion_' + str(port)
            vfs.mount(partition, mountpoint, readonly=False)
            if self._logging:
                print("H:EEPROM initialised")
        except OSError as e:
            if e.args[0] == 1:
                if self._logging:
                    print("H:EEPROM initialised")
            else:
                print(f"H:Error mounting: {e}")
                return False
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error mounting: {e}")
            return False
        return True


   # ------------------------------------------------------------------

    @staticmethod
    def _erase_eeprom(port: int, addr: int, addr_len: int, eeprom_total_size: int, eeprom_page_size: int) -> bool:
        """Erase the hexpansion EEPROM on the given port.  Returns True if successful, False otherwise."""
        try:
            i2c = I2C(port)
            for page in range(eeprom_total_size // eeprom_page_size):
                mem_addr = page * eeprom_page_size
                mem_addr_mask = (1 << (addr_len * 8)) - 1
                i2c.writeto_mem((addr | (mem_addr >> (8 * addr_len))), (mem_addr & mem_addr_mask), bytes([0xFF] * eeprom_page_size), addrsize=(8 * addr_len))
                while True:
                    try:
                        if i2c.writeto((addr | (mem_addr >> (8 * addr_len))), bytes([mem_addr & 0xFF]) if addr_len == 1 else bytes([mem_addr >> 8, mem_addr & 0xFF])):
                            break
                    except OSError:
                        pass
                    finally:
                        time.sleep_ms(1)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Error erasing EEPROM [port:{port}, addr:{addr}, addr_len:{addr_len}]: {e}")
            return False
        return True


    # ------------------------------------------------------------------
    # HEXPANSION operations
    # ------------------------------------------------------------------

    def _find_hexpansion_app(self, port: int) -> object | None:
        """Find the app instance running from the hexpansion on the given port, if any.  Returns the app instance if found, None otherwise."""

        print(f"H:Looking for app for hexpansion on port {port} in Scheduler...")
        self._report_hexpansion_states()

        app = self._app
        hexpansion_type = self._hexpansion_type_by_slot[port - 1]
        if hexpansion_type is None or hexpansion_type >= len(app.HEXPANSION_TYPES):
            return None
        expected_app_name = app.HEXPANSION_TYPES[hexpansion_type].app_name
        print(f"H:Expecting app {expected_app_name} for hexpansion type {app.HEXPANSION_TYPES[hexpansion_type].name} on port {port}")
        for an_app in scheduler.apps:
            if type(an_app).__name__ == expected_app_name:
                if hasattr(an_app, "config") and hasattr(an_app.config, "port") and an_app.config.port == port:
                    return an_app
        return None


    # ------------------------------------------------------------------

    def _check_ports_to_initialise(self, delta) -> bool:       # pylint: disable=unused-argument
        """Check for hexpansion presence in any ports, and if a new hexpansion with a blank EEPROM is detected, prompt the user to initialise it.  
           Returns True if we are now in the initialise confirmation state, False otherwise."""
        app = self._app
        if 0 < len(self._ports_to_initialise):
            self._detected_port = self._ports_to_initialise.pop()
            app.notification = Notification("Initialise?", port=self._detected_port)
            self._sub_state = _SUB_DETECTED
            return True
        return False


    # ------------------------------------------------------------------

    def _check_ports_to_upgrade(self, delta) -> bool:
        """Check any ports with Hexpansions which are expected to have apps have the latest app version and prompt the user to upgrade if not.
           Returns True if we are now in the upgrade confirmation state, False otherwise."""
        app = self._app
        if self._waiting_app_port is not None or (0 < len(self._ports_to_check_app)):
            if self._waiting_app_port is None:
                self._waiting_app_port = self._ports_to_check_app.pop()
                self._hexpansion_app_startup_timer = 0
                hexpansion_app = self._find_hexpansion_app(self._waiting_app_port)
                if hexpansion_app is not None:
                    try:
                        hexpansion_app_version = hexpansion_app.get_version()
                    except Exception as e:   # pylint: disable=broad-except
                        hexpansion_app_version = 0
                        print(f"H:Error getting Hexpansion app version - assume old: {e}")
                elif 5000 < self._hexpansion_app_startup_timer:
                    if self._logging:
                        print("H:Timeout waiting for Hexpansion app to be started - assume it needs upgrading")
                    hexpansion_app_version = 0
                else:
                    if 0 == self._hexpansion_app_startup_timer:
                        if self._logging:
                            print(f"H:No app found on port {self._waiting_app_port} - WAITING for app to appear in Scheduler")
                    app.notification = Notification("Checking...", port=self._waiting_app_port)
                    self._hexpansion_app_startup_timer += delta
                    return True
                if hexpansion_app_version == app.HEXPANSION_TYPES[self._hexpansion_type_by_slot[self._waiting_app_port - 1]].app_mpy_version:
                    if self._logging:
                        print(f"H:Hexpansion on port {self._waiting_app_port} has latest App")
                    self._hexpansion_state_by_slot[self._waiting_app_port - 1] = _HEXPANSION_STATE_RECOGNISED_APP_OK
                    self._sub_state = _SUB_CHECK
                else:
                    if self._logging:
                        print(f"H:Hexpansion app on port {self._waiting_app_port} needs upgrading from version {hexpansion_app_version} to {app.HEXPANSION_TYPES[self._hexpansion_type_by_slot[self._waiting_app_port - 1]].app_mpy_version}")
                    self._upgrade_port = self._waiting_app_port
                    app.notification = Notification("Upgrade?", port=self._upgrade_port)
                    self._sub_state = _SUB_UPGRADE_CONFIRM
            self._waiting_app_port = None
            self._hexpansion_app_startup_timer = 0  
            return True
        return False
    

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
    def __init__(self, pid: int, name: str, vid: int =0xCAFE,
                 eeprom_page_size: int =_DEFAULT_EEPROM_PAGE_SIZE, eeprom_total_size: int =_DEFAULT_EEPROM_TOTAL_SIZE,
                 motors: int =0, steppers: int =0, servos: int =0, sensors: int =0,
                 sub_type: str | None =None,
                 app_mpy_name: str | None =None, app_mpy_version: str | None =None, app_name: str | None =None):
        self.vid: int = vid
        self.pid: int = pid
        self.name: str = name
        self.eeprom_page_size: int = eeprom_page_size
        self.eeprom_total_size: int = eeprom_total_size
        self.sub_type: str | None = sub_type
        self.motors: int = motors
        self.servos: int = servos
        self.steppers: int = steppers
        self.sensors: int = sensors
        self.app_mpy_name: str | None = app_mpy_name
        self.app_mpy_version: str | None = app_mpy_version
        self.app_name: str | None = app_name

