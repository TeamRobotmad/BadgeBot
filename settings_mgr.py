# Settings Module for BadgeBot
#
# Contains the MySetting class for managing individual settings with
# min/max bounds, persistence, and increment/decrement by level.
# Also contains the settings editing UI state handler
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to BadgeBotApp
#   update(delta)   – per-tick state machine update
#   draw(ctx)       – render settings editing UI

import settings as platform_settings
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification
from .app import SETTINGS_NAME_PREFIX

MENU_ENTRY_NAME = "Settings"

class MySetting:
    def __init__(self, container, default, minimum, maximum, labels=None):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum
        self._labels = labels

    def __str__(self):
        return str(self.v)

    def _index(self):
        for k, v in self._container.items():
            if v == self:
                return k
        return None

    def label(self, index: int = None):
        if index is not None:
            if self._labels is not None and index < len(self._labels):
                return self._labels[int(index)]
            return str(index)
        if self._labels is not None and self.v is not None and self.v < len(self._labels):
            return self._labels[int(self.v)]
        return str(self.v)

    @staticmethod
    def _quantize_tenths(value: float) -> float:
        """Round to 0.1 steps deterministically to avoid float drift artifacts."""
        scaled = int((value * 10) + (0.5 if value >= 0 else -0.5))
        return scaled / 10.0

    def inc(self, v, l=0):
        """ Increment the setting value.  If l > 0, increment by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.)"""
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l == 0:
                v += 1
            else:
                d = 10 ** l
                v = ((v // d) + 1) * d
            if v > self._max:
                if self._labels is not None:
                    # settings that are purely label-based wrap around
                    v = 0
                else:
                    v = self._max
        elif isinstance(self.v, float):
            v = self._quantize_tenths(v) + 0.1
            if v > self._max:
                v = self._max
            v = self._quantize_tenths(v)
        elif self._container['logging'].v:
            print(f"H:inc type: {type(self.v)}")
        return v

    def dec(self, v, l=0):
        """Decrement the setting value.  If l > 0, decrement by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.)"""
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l == 0:
                v -= 1
            else:
                d = 10 ** l
                v = (((v + (9 * (10 ** (l - 1)))) // d) - 1) * d
            if v < self._min:
                if self._labels is not None:
                    # settings that are purely label-based wrap around
                    v = len(self._labels) - 1
                else:
                    v = self._min
        elif isinstance(self.v, float):
            v = self._quantize_tenths(v) - 0.1
            if v < self._min:
                v = self._min
            v = self._quantize_tenths(v)
        elif self._container['logging'].v:
            print(f"H: dec type: {type(self.v)}")
        return v

    def persist(self):
        """Persist the setting value to platform storage.  If the value is equal to the default, the setting will be removed from storage to save space."""
        try:
            if self.v != self.d:
                platform_settings.set(f"{SETTINGS_NAME_PREFIX}.{self._index()}", self.v)
            else:
                platform_settings.set(f"{SETTINGS_NAME_PREFIX}.{self._index()}", None)
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Failed to persist setting {self._index()}: {e}")


class SettingsMgr:
    """Manages the Settings editing UI.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self.edit_setting: int  = None
        self.edit_setting_value = None
        if self._logging:
            print("SettingsMgr initialised")

    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        """Whether to print debug logs to the console."""
        return self._logging

    @logging.setter
    def logging(self, value: bool):
        self._logging = value


    def  start(self, item: str) -> bool:
        """Enter Settings editing mode from the main menu."""
        app = self._app
        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()
        if self._logging:
            print("Entered Settings editing mode")
        self.edit_setting = item
        self.edit_setting_value = app.settings[item].v
        return True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle Settings editing UI.  Returns True if this module handled the state."""
        app = self._app

        if app.button_states.get(BUTTON_TYPES["UP"]):
            if app.auto_repeat_check(delta, False):
                self.edit_setting_value = app.settings[self.edit_setting].inc(self.edit_setting_value, app.auto_repeat_level)
                if self._logging:
                    print(f"Setting: {self.edit_setting} (+) Value: {self.edit_setting_value}")
                app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            if app.auto_repeat_check(delta, False):
                self.edit_setting_value = app.settings[self.edit_setting].dec(self.edit_setting_value, app.auto_repeat_level)
                if self._logging:
                    print(f"Setting: {self.edit_setting} (-) Value: {self.edit_setting_value}")
                app.refresh = True
        else:
            app.auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["RIGHT"]) or app.button_states.get(BUTTON_TYPES["LEFT"]):
                app.button_states.clear()
                self.edit_setting_value = app.settings[self.edit_setting].d
                if self._logging:
                    print(f"Setting: {self.edit_setting} Default: {self.edit_setting_value}")
                app.refresh = True
                app.notification = Notification("Default")
            elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if self._logging:
                    print(f"Setting: {self.edit_setting} Cancelled")
                app.fast_settings_update()  # Update fast access settings which might have been changed
                app.return_to_menu(MENU_ENTRY_NAME)
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                if self._logging:
                    print(f"Setting: {self.edit_setting} = {self.edit_setting_value}")
                app.settings[self.edit_setting].v = self.edit_setting_value
                app.settings[self.edit_setting].persist()
                app.notification = Notification(f"  Setting:   {self.edit_setting}={self.edit_setting_value}")
                app.return_to_menu(MENU_ENTRY_NAME)
        return True


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Settings editing UI.  Returns True if handled."""
        app = self._app
        disp_val = app.settings[self.edit_setting].label(self.edit_setting_value)
        app.draw_message(ctx, ["Edit Setting", f"{self.edit_setting}:", f"{disp_val}"], [(1, 1, 0), (0, 0, 1), (0, 1, 0)], label_font_size)
        button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
        return True


    # ------------------------------------------------------------------
