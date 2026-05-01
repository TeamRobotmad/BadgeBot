""" Hexpansion & EEPROM Management Module for BadgeBot """
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
_SUB_EXIT            = 9           # State for exiting from interactive mode back to menu)

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

# Modes
_MODE_INIT = 0          # Initial mode on app startup to do first scan of all ports
_MODE_IDLE = 1          # Idle mode with no hexpansion-related activity, waiting for events
_MODE_UPDATE = 2        # Normal mode for responding to hexpansion-related events (insertion/removal)
_MODE_INTERACTIVE = 3   # Interactive mode for user interactions for initialisation/upgrade/erasure of hexpansions

_SINGLE_PORT_HEXPANSION_REFS = (
    ("hexsense_port", "HexSense", "HEXSENSE_HEXPANSION_INDEX"),
    ("hextest_port", "HexTest", "HEXTEST_HEXPANSION_INDEX"),
    ("hexdiag_port", "HexDiag", "HEXDIAG_HEXPANSION_INDEX"),
)

# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):        # pylint: disable=unused-argument, invalid-name
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
        self._mode: int = _MODE_INIT
        self._sub_state: int = _SUB_INIT
        self._prev_state: int = _SUB_INIT
        self._port_selected: int = 0
        self._port_selected_header: HexpansionHeader | None = None  # HexpansionHeader for selected port
        self._port_detail_page: int = 0              # 0=vid/pid, 1=eeprom, 2=details (conditional)
        self._port_detail_page_count: int = 2        # 2 or 3 depending on whether details page is available
        self._hexpansion_app_startup_timer: int = 0
        self._hexpansion_type_by_slot: list[int | None] = [None]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_state_by_slot: list[int] = [_HEXPANSION_STATE_UNKNOWN]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_eeprom_addr_len: list[int | None] = [None]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_eeprom_addr: list[int | None] = [None]*_NUM_HEXPANSION_SLOTS
        self._hexpansion_init_type: int = 0
        self._detected_port: int | None = None
        self._waiting_app_port: int | None = None
        self._erase_port: int | None = None
        self._upgrade_port: int | None = None
        self._ports_to_initialise: set[int] = set() # ports with blank EEPROM which could be initialised
        self._ports_to_check_app: set[int] = set()  # ports with recognised hexpansion type which should be checked for the correct app before being used
        self._reboop_required: bool = False
        self._hexpansion_serial_number: int | None = None
        self._message_being_shown: bool = False

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


    def _clear_single_port_hexpansion_refs(self, port: int | None):
        """Clear app references for single-port hexpansions assigned to *port*."""
        if port is None:
            return

        app = self._app
        for attr_name, display_name, _ in _SINGLE_PORT_HEXPANSION_REFS:
            if getattr(app, attr_name, None) == port:
                setattr(app, attr_name, None)
                if self._logging:
                    print(f"H:{display_name} on port {port} erased!")


    def _has_single_port_hexpansion_on_port(self, port: int) -> bool:
        """Return True when a tracked single-port hexpansion is assigned to *port*."""
        app = self._app
        for attr_name, _, _ in _SINGLE_PORT_HEXPANSION_REFS:
            if getattr(app, attr_name, None) == port:
                return True
        return False


    def _refresh_single_port_hexpansion_assignments(self):
        """Update tracked single-port hexpansion ports and refresh dependent state."""
        app = self._app
        previous_ports = {}

        for attr_name, _, type_index_attr in _SINGLE_PORT_HEXPANSION_REFS:
            previous_port = getattr(app, attr_name, None)
            previous_ports[attr_name] = previous_port
            type_index = getattr(app, type_index_attr)
            current_port, _ = self._check_hexpansion(previous_port, type_index)
            setattr(app, attr_name, current_port)

        if previous_ports["hexdiag_port"] != app.hexdiag_port:
            app.hexdiag_setup()

        if previous_ports["hextest_port"] != app.hextest_port and app.sensor_test_mgr is not None:
            app.sensor_test_mgr.hextest_setup(app.hextest_port)


    def _should_claim_single_port_hexpansion(self, type_index: int) -> bool:
        """Return True if a detected single-port hexpansion type is not yet assigned."""
        app = self._app
        for attr_name, _, type_index_attr in _SINGLE_PORT_HEXPANSION_REFS:
            if type_index == getattr(app, type_index_attr):
                return getattr(app, attr_name, None) is None
        return False


    # ------------------------------------------------------------------
    # Async event handlers (registered directly on eventbus)
    # ------------------------------------------------------------------

    async def _handle_removal(self, event):
        app = self._app
        self._hexpansion_type_by_slot[event.port - 1] = None
        self._hexpansion_state_by_slot[event.port - 1] = _HEXPANSION_STATE_EMPTY
        if event.port in self._ports_to_initialise:
            self._ports_to_initialise.remove(event.port)
        self._ports_to_check_app.discard(event.port)

        if (self._detected_port     is not None and event.port == self._detected_port) or \
            (self._upgrade_port     is not None and event.port == self._upgrade_port) or \
            (self._waiting_app_port is not None and event.port == self._waiting_app_port) or \
            (self._erase_port       is not None and event.port == self._erase_port) or \
            (self._port_selected != 0           and event.port == self._port_selected) or \
            (event.port in app.hexdrive_ports) or \
            self._has_single_port_hexpansion_on_port(event.port):
            # The port from which a hexpansion has been removed is significant
            app.hexpansion_update_required = True
            if self._logging:
                print(f"H:Hexpansion removed from port {event.port}")


    async def _handle_insertion(self, event):
        if self._check_port_for_known_hexpansions(event.port) or event.port == self._port_selected:
            # A known hexpansion type has been detected on the inserted port, so trigger an update of
            # the hexpansion management state machine to handle it. Or the inserted port is the one
            # currently being selected by the user in interactive mode, so we should also trigger an
            # update to check if it's a valid hexpansion and update the UI accordingly.
            self._app.hexpansion_update_required = True
            if self._logging:
                print(f"H:Hexpansion inserted into port {event.port}")

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
        self._mode = _MODE_INTERACTIVE
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
        # remember the unique ID so that we can program it back if the EEPROM is re-initialised.
        self._hexpansion_serial_number = self._port_selected_header.unique_id if self._port_selected_header is not None else None
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
        elif app.hexpansion_update_required:
            # This flag is set when a hexpansion-related event occurs that should trigger an update of the hexpansion management state machine (e.g. insertion/removal of a hexpansion).
            if self._mode == _MODE_IDLE:
                self._mode = _MODE_UPDATE
                if self._logging:
                    print("H:Hexpansion update triggered by event")
                self._sub_state = _SUB_CHECK
                app.set_menu(None)
            elif self._mode == _MODE_INTERACTIVE:
                app.hexpansion_update_required = False
                self._sub_state = _SUB_CHECK

        if self._check_ports_to_upgrade(delta):
            # moves to _SUB_UPGRADE_CONFIRM if there are any ports with recognised hexpansion which need upgrading
            # then to _SUB_PROGRAMMING if the user confirms the upgrade
            # finally gets to _SUB_CHECK
            pass
        elif self._check_ports_to_initialise(delta):
            # moves to _SUB_DETECTED if there are any ports with blank EEPROM which could be initialised
            # then to _SUB_PROGRAMMING if the user confirms the initialisation
            # finally gets to _SUB_CHECK
            pass
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
        elif self._sub_state == _SUB_PORT_SELECT:
            self._update_state_port_select(delta)
        elif self._sub_state == _SUB_CHECK:
            self._update_state_check(delta)

        if self._sub_state != self._prev_state:
            if self._logging:
                print(f"H:HexpansionMgr State: {self._prev_state} -> {self._sub_state}")
            self._prev_state = self._sub_state

        if self._sub_state == _SUB_DONE:
            print("H:DONE")
            if self._mode == _MODE_INTERACTIVE:
                self._enter_port_select()
                # Exit from _SUB_DONE to allow user to select another port for management if they wish
            else:
                if self._mode == _MODE_UPDATE:
                    app.hexpansion_update_required = False #to avoid beign called immediately
                self._sub_state = _SUB_EXIT # exit to menu on next call (when user accepts warning)
        elif self._sub_state == _SUB_EXIT:
            print("H:EXIT")
            app.hexpansion_update_required = False
            self._message_being_shown = False
            app.initialise_settings()
            app.return_to_menu()
            self._mode = _MODE_IDLE

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
                #app.notification = Notification("No App", port=self._upgrade_port)
                #app.show_message(["No App", "for this", "Hexpansion"], [(1,0,0),(1,0,0),(1,0,0)], "hexpansion")
                #self._message_being_shown = True
                self._sub_state = _SUB_CHECK
            else:
                result = self._update_app_in_eeprom(self._upgrade_port)
                if result == _APP_EEPROM_RESULT_FAILURE:
                    app.notification = Notification("Failed", port=self._upgrade_port)
                    app.show_message(["Hexpansion", "programming", "failed", "Protected?"], [(1,0,0),(1,0,0),(1,0,0),(1,0,0)], "warning")
                    self._message_being_shown = True
                    self._sub_state = _SUB_CHECK
                elif result == _APP_EEPROM_RESULT_MISSING:
                    app.notification = Notification("App Missing", port=self._upgrade_port)
                    app.show_message(["App file", "missing for", "this Hexpansion"], [(1,0,0),(1,0,0),(1,0,0)], "warning")
                    self._message_being_shown = True
                    self._sub_state = _SUB_CHECK
                else:
                    #upgrade_text = "Upgraded" if result == _APP_EEPROM_RESULT_SUCCESSFUL_UPGRADE else "Programmed"
                    #app.notification = Notification(upgrade_text, port=self._upgrade_port)
                    # No point showing "Programmed" vs "Upgraded" as the Hexpansion Insertion Notification will cover it up
                    eventbus.emit(HexpansionInsertionEvent(self._upgrade_port))
                    #app.show_message([f"{upgrade_text}:", "Please", "reboop"], [(0,1,0),(1,1,1),(1,1,1)], "reboop")
                    #self._reboop_required = True
                    self._hexpansion_state_by_slot[self._upgrade_port - 1] = _HEXPANSION_STATE_RECOGNISED_APP_OK
                    self._sub_state = _SUB_CHECK
            self._upgrade_port = None
        elif self._detected_port is not None:
            # There is a hexpansion ready for EEPROM initialisation
            if self._prepare_eeprom(self._detected_port):
                self._hexpansion_type_by_slot[self._detected_port - 1] = self._hexpansion_init_type
                if app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_name is not None:
                    app.notification = Notification("Initialised", port=self._detected_port)
                    # Only worth showing "Initialised" Notification is we are NOT going to trigger Hexpansion Insertion Notification
                    self._upgrade_port = self._detected_port
                    self._sub_state = _SUB_PROGRAMMING
                    self._hexpansion_state_by_slot[self._detected_port - 1] = _HEXPANSION_STATE_RECOGNISED
                else:
                    eventbus.emit(HexpansionInsertionEvent(self._detected_port))
                    #app.show_message(["No App", "for this", "Hexpansion"], [(1,1,0),(1,1,1),(1,1,1)], "hexpansion")
                    #self._message_being_shown = True
                    self._sub_state = _SUB_CHECK
                    self._hexpansion_state_by_slot[self._detected_port - 1] = _HEXPANSION_STATE_RECOGNISED_NO_APP
            else:
                app.notification = Notification("Failed", port=self._detected_port)
                self._hexpansion_type_by_slot[self._detected_port - 1] = None
                self._hexpansion_state_by_slot[self._detected_port - 1] = _HEXPANSION_STATE_FAULTY
                app.show_message(["EEPROM", "initialisation", "failed", "Protected?"], [(1,0,0),(1,0,0),(1,0,0),(1,0,0)], "warning")
                self._message_being_shown = True
                self._sub_state = _SUB_CHECK
            self._detected_port = None
        else:
            print("H:Error - no port to program")
            self._sub_state = _SUB_INIT if self._mode == _MODE_INIT else _SUB_CHECK


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
            self._sub_state = _SUB_INIT if self._mode == _MODE_INIT else _SUB_CHECK
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
        # not used in _MODE_INIT
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
        # not used in _MODE_INIT
        app = self._app
        erase_port = self._erase_port
        if erase_port is None:
            self._sub_state = _SUB_CHECK
            return
        if self._logging:
            print(f"H:Erasing EEPROM on port {erase_port}")
        eeprom_page_size=app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_page_size if self._hexpansion_init_type > 0 else _DEFAULT_EEPROM_PAGE_SIZE
        eeprom_total_size=app.HEXPANSION_TYPES[self._hexpansion_init_type].eeprom_total_size if self._hexpansion_init_type > 0 else _DEFAULT_EEPROM_TOTAL_SIZE
        erase_addr_len = self._hexpansion_eeprom_addr_len[erase_port - 1]
        erase_addr = self._hexpansion_eeprom_addr[erase_port - 1]
        if erase_addr_len is None or erase_addr is None:
            app.notification = Notification("Failed", port=erase_port)
            self._sub_state = _SUB_CHECK
            return
        if self._logging:
            print(f"H:Erase {self._hexpansion_init_type} page size: {eeprom_page_size} bytes, total size: {eeprom_total_size} bytes, addr_len: {erase_addr_len}, addr: {hex(erase_addr)}")

        if self._erase_eeprom(erase_port,
                              erase_addr,
                              erase_addr_len,
                              eeprom_total_size,
                              eeprom_page_size):
            app.notification = Notification("Erased", port=erase_port)
            self._hexpansion_type_by_slot[erase_port - 1] = app.BLANK_HEXPANSION_INDEX
            self._hexpansion_state_by_slot[erase_port - 1] = _HEXPANSION_STATE_BLANK
            hexpansion_type = self._type_name_for_port(erase_port)
            app.show_message([hexpansion_type, f"in slot {erase_port}:", "Erased"], [(1,1,0), (1,1,1), (0,1,0)], "hexpansion")
            self._sub_state = _SUB_DETECTED
            self._detected_port = erase_port
        else:
            app.notification = Notification("Failed", port=erase_port)
            app.show_message(["EEPROM", "erasure", "failed", "Protected?"], [(1,0,0),(1,0,0),(1,0,0),(1,0,0)], "warning")
            self._message_being_shown = True
            self._sub_state = _SUB_CHECK

        #self._reboop_required = True

        if erase_port in app.hexdrive_ports:
            hexdrive_index = app.hexdrive_ports.index(erase_port)
            del app.hexdrive_ports[hexdrive_index]
            if hexdrive_index < len(app.hexdrive_apps):
                del app.hexdrive_apps[hexdrive_index]
            app.motor_controller = None
            if self._logging:
                print(f"H:HexDrive on port {erase_port} erased!")

        self._clear_single_port_hexpansion_refs(erase_port)

        self._erase_port = None


    def _update_state_upgrade(self, delta):     # pylint: disable=unused-argument
        """ Allow User to confirm or cancel App upgrade."""
        app = self._app
        upgrade_port = self._upgrade_port
        if upgrade_port is None:
            if self.logging:
                print("H:Error - no port to upgrade")
            self._sub_state = _SUB_INIT if self._mode == _MODE_INIT else _SUB_CHECK
            return
        if app.button_states.get(BUTTON_TYPES["CONFIRM"]):
            app.button_states.clear()
            app.notification = Notification("Upgrading", port=upgrade_port)
            self._sub_state = _SUB_PROGRAMMING
        elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
            if self._logging:
                print("H:Upgrade Cancelled")
            app.button_states.clear()
            #self._hexpansion_state_by_slot[upgrade_port - 1] = _HEXPANSION_STATE_RECOGNISED_OLD_APP
            self._upgrade_port = None
            self._sub_state = _SUB_INIT if self._mode == _MODE_INIT else _SUB_CHECK


    def _get_hexpansion_by_type(self, hexpansion_type) -> int | None:
        """ Return the port number of a hexpansion of the given type, or None if no such hexpansion is currently detected."""
        for port in range(0, _NUM_HEXPANSION_SLOTS):
            if self._hexpansion_type_by_slot[port] == hexpansion_type:
                return port+1
        return None


    def _report_hexpansion_states(self):
        """ Utility function to print the current detected hexpansion types and states for each port, for debugging purposes."""
        if not self._logging:
            return
        app = self._app
        for port in range(0, _NUM_HEXPANSION_SLOTS):
            type_idx = self._hexpansion_type_by_slot[port]
            type_name = app.HEXPANSION_TYPES[type_idx].name if type_idx is not None else "None"
            state_name = _HEXPANSION_STATE_NAMES[self._hexpansion_state_by_slot[port]]
            print(f"Port {port+1}: Type={type_name}, State={state_name}")
        print(f"Ports to initialise: {self._ports_to_initialise}")
        print(f"Ports to check app: {self._ports_to_check_app}")
        print(f"hexsense_port:{app.hexsense_port}")
        print(f"hextest_port:{app.hextest_port}")
        print(f"hexdiag_port:{app.hexdiag_port}")
        print(f"hexdrive_ports:{app.hexdrive_ports}")
        print(f"hexpansion_update_required = {app.hexpansion_update_required}")
        print(f"mode = {self._mode}")



    def _check_hexpansion(self, port: int | None, type_index: int) -> tuple[int | None, object | None]:
        """ Check if the currently configured hexpansion of the given type is still present, and
            if not, check if there is another hexpansion of the same type that can be switched to, or if the hexpansion has been removed entirely."""
        app = self._app
        hexpansion_app = None
        hexpansion_was_present = port is not None
        old_port = port
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
                    #app.show_message([f"{name} moved", f"to port {new_port}"], [(1,1,0),(1,1,1)], "hexpansion")
                    #self._message_being_shown = True
                else:
                    if self._logging:
                        print(f"H:{name} found on port {new_port}")
                port = new_port
                if app.HEXPANSION_TYPES[type_index].app_name is not None:
                    self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED # may be updated to _RECOGNISED_APP_OK or _RECOGNISED_OLD_APP after checking for the correct app
                    hexpansion_app = self._check_hexpansion_app_on_port(port, type_index)
                else:
                    self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED_NO_APP
            elif hexpansion_was_present:
                if self._logging:
                    print(f"H:{name} on port {old_port} lost")
                if self._mode == _MODE_UPDATE:
                    app.show_message([f"{name}","removed.","Please reinsert"], [(1,1,0),(1,1,1),(1,1,1)], "error")
                    self._message_being_shown = True
                app.notification = Notification(f"{name} removed", port=old_port)

        return port, hexpansion_app


    def _update_state_check(self, delta):       # pylint: disable=unused-argument
        """ Check for HexDrive & HexSense presence and update app state accordingly."""
        app = self._app

        self._report_hexpansion_states()

        # For hexpansiosn of which we only need to know where one is we track movements between ports and update the assigned port accordingly
        self._refresh_single_port_hexpansion_assignments()

        # Build a new list of ports with HexDrives - to allow for more than one being present and used:
        new_hexdrive_ports = []
        for port in range(1, _NUM_HEXPANSION_SLOTS + 1):
            # check if there is a hexpansion of a type that can be a HexDrive on this port
            type_idx = self._hexpansion_type_by_slot[port-1]
            if type_idx is not None and type_idx in app.hexdrive_hexpansion_types:
                if self._logging:
                    print(f"H:HexDrive on port {port} added to list")
                new_hexdrive_ports.append(port)

        if set(new_hexdrive_ports) != set(app.hexdrive_ports):
            if self._logging:
                print(f"H:HexDrive ports changed from {app.hexdrive_ports} to {new_hexdrive_ports}")
            app.hexdrive_ports = new_hexdrive_ports
            app.hexdrive_apps = []
            app.num_motors = 0
            app.num_servos = 0
            for port in app.hexdrive_ports:
                hexdrive_type_idx = self._hexpansion_type_by_slot[port - 1]
                if hexdrive_type_idx is not None and 0 <= hexdrive_type_idx < len(app.HEXPANSION_TYPES):
                    app.num_motors   += app.HEXPANSION_TYPES[hexdrive_type_idx].motors
                    app.num_servos   += app.HEXPANSION_TYPES[hexdrive_type_idx].servos

        if len(app.hexdrive_ports) != len (app.hexdrive_apps):
            hexdrive_apps = []
            for port in app.hexdrive_ports:
                if self._logging:
                    print(f"H:Checking HexDrive app on port {port}, current state: {_HEXPANSION_STATE_NAMES[self._hexpansion_state_by_slot[port - 1]]}")
                if self._hexpansion_state_by_slot[port - 1] == _HEXPANSION_STATE_RECOGNISED_APP_OK:
                    # already checked and app is OK, so just add it to the list
                    hexdrive_app = self._find_hexpansion_app(port)
                    if hexdrive_app is not None and hexdrive_app not in app.hexdrive_apps:
                        hexdrive_apps.append(hexdrive_app)
                elif self._hexpansion_state_by_slot[port - 1] == _HEXPANSION_STATE_RECOGNISED:
                    # should this port have an app
                    type_idx = self._hexpansion_type_by_slot[port - 1]
                    if app.HEXPANSION_TYPES[type_idx].app_name is not None:
                        # Yes this port should have an app, but we haven't checked it yet, so check if the correct app is running on this port
                        if port not in self._ports_to_check_app:
                            if self._logging:
                                print(f"H:Request Check for {app.HEXPANSION_TYPES[type_idx].app_name} app on port {port}")
                            self._ports_to_check_app.add(port)

            if len(hexdrive_apps) > 0:
                app.hexdrive_apps = hexdrive_apps
                if self._logging:
                    print(f"H:Latest HexDrive apps: {app.hexdrive_apps}")

        # Create the high-level MotorController for IMU-aided driving
        # (only when the HexDrive has motors)
        if app.num_motors > 1 and len(app.hexdrive_apps) > 0 and app.motor_controller is None:
            # App may still be loading - we can't initialise the MotorController until the HexDrive app is loaded
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

        if len(self._ports_to_check_app) > 0:
            # there are outstandind apps to check
            if self._logging:
                print(f"H:Waiting for app version check on port(s): {self._ports_to_check_app}")
        else:
            # Check Complete - decide next state
            if self._reboop_required:
                app.show_message(["Please", "reboop"], [(1,1,1),(1,1,1)], "reboop")
                return # so that you can't get out of this without a reboop
            elif len(app.hexdrive_apps) == 0 and self._mode == _MODE_INIT:
                app.show_message(_HEXDRIVE_REQUIRED_MESSAGE, _HEXDRIVE_REQUIRED_MESSAGE_COLOURS, "warning")
                self._message_being_shown = True

            if self._message_being_shown or self._mode == _MODE_INTERACTIVE:
                # we will be called again when the message is dismissed
                self._sub_state = _SUB_DONE
            else:
                self._sub_state = _SUB_EXIT


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
            app.button_states.clear()
            self._sub_state = _SUB_EXIT


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
            app.draw_message(ctx, ["Hexpansion", f"in slot {self._detected_port}:", "Init EEPROM as", hexpansion_type, f"{hexpansion_sub_type if hexpansion_sub_type else ''}?"], \
                                  [(1, 1, 0), (1, 1, 0), (1, 1, 0), (1, 0, 1), (1, 0, 1)], label_font_size)
            button_labels(ctx, confirm_label="Yes", up_label=app.special_chars['up'], down_label="\u25BC", \
                          left_label=app.HEXPANSION_TYPES[app.HEXDRIVE_HEXPANSION_INDEX].name, \
                         right_label=app.HEXPANSION_TYPES[app.HEXSENSE_HEXPANSION_INDEX].name, cancel_label="No")
            return True
        elif self._sub_state == _SUB_PORT_SELECT:
            self._draw_port_select(ctx)
            return True
        elif self._sub_state == _SUB_ERASE_CONFIRM:
            if self._erase_port is None:
                return False
            hexpansion_type_name = self._type_name_for_port(self._erase_port, self._hexpansion_init_type)
            # If the EEPROM type is unknown, show the proposed type and later allow selecting from common options.
            app.draw_message(ctx, [hexpansion_type_name, f"in slot {self._erase_port}:", "Erase EEPROM?"], [(1, 0, 1), (1, 1, 0), (1, 0, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif self._sub_state == _SUB_ERASE:
            if self._erase_port is None:
                return False
            hexpansion_type_name = self._type_name_for_port(self._erase_port, self._hexpansion_init_type)
            app.draw_message(ctx, [hexpansion_type_name, f"in slot {self._erase_port}:", "Erasing..."], [(1, 0, 1), (1, 1, 0), (1, 0, 0)], label_font_size)
            return True
        elif self._sub_state == _SUB_UPGRADE_CONFIRM:
            if self._upgrade_port is None:
                return False
            hexpansion_type_name = self._type_name_for_port(self._upgrade_port, self._hexpansion_init_type)
            upgrade_or_install = "Upgrade" if self._hexpansion_state_by_slot[self._upgrade_port-1] == _HEXPANSION_STATE_RECOGNISED_OLD_APP  else "Install"
            app.draw_message(ctx, [hexpansion_type_name, f"in slot {self._upgrade_port}:", upgrade_or_install, f"{hexpansion_type_name} app?"], [(1, 0, 1), (1, 1, 0), (1, 1, 0), (1, 1, 0)], label_font_size)
            button_labels(ctx, confirm_label="Yes", cancel_label="No")
            return True
        elif self._sub_state == _SUB_PROGRAMMING:
            # During upgrade, show the already-detected type for the selected port.
            if self._upgrade_port is None:
                return False
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
                    lines += [f"VID: {hdr.vid:04X}", f"PID: {hdr.pid:04X}", f"UID: {hdr.unique_id}"]
                    colours += [(0, 1, 1), (0, 1, 1), (0, 1, 1)]
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
                            ver = getattr(running_app, "VERSION",
                                          getattr(running_app, "version", None))
                            if ver is not None:
                                lines.append(f"v{ver}")
                                colours.append((0, 1, 1))
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

        confirm_label = "Init" if self._hexpansion_state_by_slot[self._port_selected - 1] == _HEXPANSION_STATE_BLANK else \
                        "Erase" if self._hexpansion_state_by_slot[self._port_selected - 1] >= _HEXPANSION_STATE_FAULTY else ""
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
        if eeprom_addr is None or addr_len is None:
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
                elif self._should_claim_single_port_hexpansion(index):
                    return True
                return False
        # Unrecognised Hexpansion
        if self._logging:
            # report VID/PID in hexadecimal
            print(f"H:Port {port} - VID/PID {hex(hexpansion_header.vid)}/{hex(hexpansion_header.pid)} not recognised")
        self._hexpansion_type_by_slot[port - 1] = app.UNRECOGNISED_HEXPANSION_INDEX
        self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_UNRECOGNISED
        return False



    def _check_hexpansion_app_on_port(self, port: int, type_index: int, ) -> object | None:
        """Check if the app for the hexpansion on the given port is present and correct"""
        app = self._app
        hexpansion_app = self._find_hexpansion_app(port)
        if hexpansion_app is not None:
            # Read version from the running app object's VERSION attribute.
            # EEPROM apps expose this on the class so per-port app instances
            # can report their loaded code version reliably.
            version = getattr(hexpansion_app, "VERSION",
                              getattr(hexpansion_app, "version", None))
            expected = app.HEXPANSION_TYPES[type_index].app_mpy_version
            if expected is None:
                # No expected version recorded for this type – treat any running app as current.
                self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED_APP_OK
            elif not _versions_match(version, expected):
                if self._hexpansion_state_by_slot[port - 1] != _HEXPANSION_STATE_RECOGNISED_OLD_APP:
                    if self._logging:
                        print(f"H:{app.HEXPANSION_TYPES[type_index].name} app on port {port} has version {version}, expected {expected}")
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
        source_file = f"EEPROM/{app.HEXPANSION_TYPES[self._hexpansion_init_type].app_mpy_name}"
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
                i2c.writeto_mem((addr | (mem_addr >> (8 * addr_len))), (mem_addr & mem_addr_mask), bytes([0xFF] * eeprom_page_size), addrsize=8 * addr_len)
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
        app = self._app
        hexpansion_type = self._hexpansion_type_by_slot[port - 1]
        if hexpansion_type is None or hexpansion_type >= len(app.HEXPANSION_TYPES):
            return None
        expected_app_name = app.HEXPANSION_TYPES[hexpansion_type].app_name
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
        if 0 < len(self._ports_to_initialise) and self._detected_port is None:
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
        port = self._waiting_app_port
        if port is not None or (0 < len(self._ports_to_check_app)):
            if port is None:
                port = self._ports_to_check_app.pop()
                self._waiting_app_port = port
                self._hexpansion_app_startup_timer = 0
            type_index = self._hexpansion_type_by_slot[port - 1]
            if type_index is None:
                if self._logging:
                    print(f"H:Unexpectedly no hexpansion type for port {port} when checking app - skipping")
                self._waiting_app_port = None
                return False
            hexpansion_app = self._check_hexpansion_app_on_port(port, type_index)
            if hexpansion_app is not None:
                if self._hexpansion_state_by_slot[port - 1] == _HEXPANSION_STATE_RECOGNISED_APP_OK:
                    self._sub_state = _SUB_CHECK
                elif self._hexpansion_state_by_slot[port - 1] == _HEXPANSION_STATE_RECOGNISED_OLD_APP:
                    if self._logging:
                        print(f"H:Hexpansion [{port}] upgrade to {app.HEXPANSION_TYPES[type_index].app_mpy_version}?")
                    self._upgrade_port = port
                    self._sub_state = _SUB_UPGRADE_CONFIRM
            elif 5000 < self._hexpansion_app_startup_timer:
                if self._logging:
                    print("H:Timeout waiting for Hexpansion app to be started - assume it needs installing")
                    print(f"H:Hexpansion [{port}] install {app.HEXPANSION_TYPES[type_index].app_mpy_name} app?")
                self._hexpansion_state_by_slot[port - 1] = _HEXPANSION_STATE_RECOGNISED_NO_APP
                self._upgrade_port = port
                self._sub_state = _SUB_UPGRADE_CONFIRM
            else:
                if 0 == self._hexpansion_app_startup_timer:
                    if self._logging:
                        print(f"H:No app found on port {port} - WAITING for app to appear in Scheduler")
                self._hexpansion_app_startup_timer += delta
                # Keep calling this function to keep checking for the app and updating the timer until we find the app or hit the timeout, at which point we will prompt to install.
                return True
            # Clear waiting app state so that we will check the next port on the next call.
            self._waiting_app_port = None
            self._hexpansion_app_startup_timer = 0
            return True
        return False


# ---- Hexpansion type descriptor -------------------------------------------

def _versions_match(running, expected) -> bool:
    """Return True when *running* (read from the hexpansion app's ``VERSION``
    attribute) equals *expected* (from ``HexpansionType.app_mpy_version``).

    * Integer versions are compared directly.
    * String versions are tokenised the same way as ``parse_version()`` in
      ``app.py`` so that ``"1.10"`` compares greater than ``"1.2"``.
    * If *running* is ``None`` (attribute missing) the versions do not match.
    """
    if running is None:
        return False
    if isinstance(expected, str) and isinstance(running, str):
        def _tok(v):
            v = v.strip("v")
            if "+" in v:
                v = v.split("+", 1)[0]
            if "-" in v:
                v = v.split("-", 1)[0]
            parts = v.split(".")
            return [int(p) if p.isdigit() else p for p in parts]
        return _tok(running) == _tok(expected)
    return running == expected


class HexpansionType:
    """Descriptor for known hexpansion types, used for detection and EEPROM programming.

    Parameters
    ----------
        pid: the PID value to identify the hexpansion type from its EEPROM header
        name: human-friendly name of the hexpansion type (e.g. "HexDrive")
        vid: the VID value to identify the hexpansion type from its EEPROM header (default 0xCAFE)
        motors, servos, sensors: the capabilities of this hexpansion type, used to configure the app when detected
        sub_type: a human-friendly string describing the specific variant of this hexpansion type
        app_mpy_name: the filename of the .mpy to copy to the hexpansion EEPROM for this type (if any)
        app_mpy_version: the version string to report for the .mpy copied to the hexpansion EEPROM for this type (if any)
        app_name: the name of the App class to look for when checking if a detected hexpansion's app is running (if any)
    """
    def __init__(self, pid: int, name: str, vid: int =0xCAFE,
                 eeprom_page_size: int =_DEFAULT_EEPROM_PAGE_SIZE, eeprom_total_size: int =_DEFAULT_EEPROM_TOTAL_SIZE,
                 motors: int =0, servos: int =0, sensors: int =0,
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
        self.sensors: int = sensors
        self.app_mpy_name: str | None = app_mpy_name
        self.app_mpy_version: str | None = app_mpy_version
        self.app_name: str | None = app_name
