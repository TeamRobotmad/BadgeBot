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
from .app import (MAIN_MENU_ITEMS, MENU_ITEM_SETTINGS)

# Front face direction labels (0=BtnA corner between slots 6 & 1, each step = 30° CW)
_FRONT_FACE_LABELS = (
    "BtnA", "Slot 1", "BtnB", "Slot 2", "BtnC", "Slot 3",
    "BtnD", "Slot 4", "BtnE", "Slot 5", "BtnF", "Slot 6",
)
_FWD_DIR_LABELS = ("Normal", "Reverse")

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
        for k, v in self._container.items():
            if v == self:
                return k
        return None

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
                v = self._max
        elif isinstance(self.v, float):
            v += 0.1
            if v > self._max:
                v = self._max
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
                v = self._min
        elif isinstance(self.v, float):
            v -= 0.1
            if v < self._min:
                v = self._min
        elif self._container['logging'].v:
            print(f"H: dec type: {type(self.v)}")
        return v

    def persist(self):
        """Persist the setting value to platform storage.  If the value is equal to the default, the setting will be removed from storage to save space."""
        try:
            if self.v != self.d:
                platform_settings.set(f"badgebot.{self._index()}", self.v)
            else:
                platform_settings.set(f"badgebot.{self._index()}", None)
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:Failed to persist setting {self._index()}: {e}")


class SettingsMgr:
    """Manages the Settings editing UI.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle Settings editing UI.  Returns True if this module handled the state."""
        app = self.app

        if app.button_states.get(BUTTON_TYPES["UP"]):
            if app.auto_repeat_check(delta, False):
                app.edit_setting_value = app.settings[app.edit_setting].inc(app.edit_setting_value, app.auto_repeat_level)
                if app.settings['logging'].v:
                    print(f"Setting: {app.edit_setting} (+) Value: {app.edit_setting_value}")
                app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            if app.auto_repeat_check(delta, False):
                app.edit_setting_value = app.settings[app.edit_setting].dec(app.edit_setting_value, app.auto_repeat_level)
                if app.settings['logging'].v:
                    print(f"Setting: {app.edit_setting} (-) Value: {app.edit_setting_value}")
                app.refresh = True
        else:
            app.auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["RIGHT"]) or app.button_states.get(BUTTON_TYPES["LEFT"]):
                app.button_states.clear()
                app.edit_setting_value = app.settings[app.edit_setting].d
                if app.settings['logging'].v:
                    print(f"Setting: {app.edit_setting} Default: {app.edit_setting_value}")
                app.refresh = True
                app.notification = Notification("Default")
            elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if app.settings['logging'].v:
                    print(f"Setting: {app.edit_setting} Cancelled")
                app.set_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
                app.return_to_menu()
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                if app.settings['logging'].v:
                    print(f"Setting: {app.edit_setting} = {app.edit_setting_value}")
                app.settings[app.edit_setting].v = app.edit_setting_value
                app.settings[app.edit_setting].persist()
                app.notification = Notification(f"  Setting:   {app.edit_setting}={app.edit_setting_value}")
                app.set_menu(MAIN_MENU_ITEMS[MENU_ITEM_SETTINGS])
                app.return_to_menu()
        return True

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Settings editing UI.  Returns True if handled."""
        app = self.app
        disp_val = self._format_setting_value(app.edit_setting, app.edit_setting_value)
        app.draw_message(ctx, ["Edit Setting", f"{app.edit_setting}:", f"{disp_val}"], [(1, 1, 1), (0, 0, 1), (0, 1, 0)], label_font_size)
        button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
        return True

    @staticmethod
    def _format_setting_value(key, value):
        """Return a display-friendly string for the given setting key/value."""
        if key == 'fwd_dir':
            try:
                return _FWD_DIR_LABELS[int(value)]
            except (IndexError, ValueError, TypeError):
                pass
        elif key == 'front_face':
            try:
                return _FRONT_FACE_LABELS[int(value)]
            except (IndexError, ValueError, TypeError):
                pass
        return str(value)
