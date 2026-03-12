# Settings Module for BadgeBot
#
# Contains the MySetting class for managing individual settings with
# min/max bounds, persistence, and increment/decrement by level.
# Also contains the settings editing UI state handler (STATE_SETTINGS).
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to LineFollowerApp
#   update(delta)   – per-tick state machine update for STATE_SETTINGS
#   draw(ctx)       – render settings editing UI

import settings as platform_settings
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification


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
        try:
            if self.v != self.d:
                platform_settings.set(f"badgebot.{self._index()}", self.v)
            else:
                platform_settings.set(f"badgebot.{self._index()}", None)
        except Exception as e:
            print(f"H:Failed to persist setting {self._index()}: {e}")


class SettingsMgr:
    """Manages the Settings editing UI (STATE_SETTINGS).

    Parameters
    ----------
    app : LineFollowerApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle STATE_SETTINGS.  Returns True if this module handled the state."""
        app = self.app
        from .linefollower import STATE_SETTINGS, STATE_MENU, _main_menu_items, MENU_ITEM_SETTINGS
        if app.current_state != STATE_SETTINGS:
            return False

        if app.button_states.get(BUTTON_TYPES["UP"]):
            if app._auto_repeat_check(delta, False):
                app._edit_setting_value = app._settings[app._edit_setting].inc(app._edit_setting_value, app._auto_repeat_level)
                if app._settings['logging'].v:
                    print(f"Setting: {app._edit_setting} (+) Value: {app._edit_setting_value}")
                app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            if app._auto_repeat_check(delta, False):
                app._edit_setting_value = app._settings[app._edit_setting].dec(app._edit_setting_value, app._auto_repeat_level)
                if app._settings['logging'].v:
                    print(f"Setting: {app._edit_setting} (-) Value: {app._edit_setting_value}")
                app._refresh = True
        else:
            app._auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["RIGHT"]) or app.button_states.get(BUTTON_TYPES["LEFT"]):
                app.button_states.clear()
                app._edit_setting_value = app._settings[app._edit_setting].d
                if app._settings['logging'].v:
                    print(f"Setting: {app._edit_setting} Default: {app._edit_setting_value}")
                app._refresh = True
                app.notification = Notification("Default")
            elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if app._settings['logging'].v:
                    print(f"Setting: {app._edit_setting} Cancelled")
                app.set_menu(_main_menu_items[MENU_ITEM_SETTINGS])
                app.current_state = STATE_MENU
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                if app._settings['logging'].v:
                    print(f"Setting: {app._edit_setting} = {app._edit_setting_value}")
                app._settings[app._edit_setting].v = app._edit_setting_value
                app._settings[app._edit_setting].persist()
                app.notification = Notification(f"  Setting:   {app._edit_setting}={app._edit_setting_value}")
                app.set_menu(_main_menu_items[MENU_ITEM_SETTINGS])
                app.current_state = STATE_MENU
        return True

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Settings editing UI.  Returns True if handled."""
        app = self.app
        from .linefollower import STATE_SETTINGS
        if app.current_state != STATE_SETTINGS:
            return False
        app.draw_message(ctx, ["Edit Setting", f"{app._edit_setting}:", f"{app._edit_setting_value}"], [(1, 1, 1), (0, 0, 1), (0, 1, 0)], label_font_size)
        button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", right_label="Default")
        return True
