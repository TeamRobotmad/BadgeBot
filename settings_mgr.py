"""Settings Manager for BadgeBot."""
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
from app_components import Menu
from app_components.utils import wrap_text
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, small_font_size, button_labels
from app_components.notification import Notification
from .app import SETTINGS_NAME_PREFIX, STATE_SETTINGS

MENU_ENTRY_NAME = "Settings"
_GROUP_MENU_PREFIX = "Settings/"
_RESET_MENU_NAME = "Settings/reset"
_RESET_ITEM = "Reset all..."
_HELP_WIDTH = 190
_HELP_VISIBLE_LINES = 6

class MySetting:
    """A single setting with min/max bounds, persistence, and increment/decrement by level."""
    GROUP_GENERAL = 0
    GROUP_MOTORS = 1
    GROUP_DRIVING = 2
    GROUP_LINE_FOLLOWER = 3
    GROUP_AUTO_DRIVE = 4
    GROUP_SERVOS = 5
    GROUP_NAMES = ("General", "Motors", "Driving", "Line follower", "Auto drive", "Servos")

    __slots__ = ("_container", "d", "v", "_min", "_max", "_wrap", "_labels", "group", "order", "title", "description")

    def __init__(self, container, default, minimum, maximum, wrap=False, labels=None,
                 group=GROUP_GENERAL, order=1000, title="", description=""):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum
        self._wrap = wrap
        self._labels = labels
        self.group = group
        self.order = order
        self.title = title
        self.description = description

    def __str__(self):
        return str(self.v)


    def clamp(self):
        """Clamp the setting value to its min/max bounds."""
        if self.v < self._min:
            if self._container['logging'].v:
                print(f"Warning: Setting {self._index()} value {self.v} is below minimum {self._min}, adjusting to minimum")
            self.v = self._min
        elif self.v > self._max:
            if self._container['logging'].v:
                print(f"Warning: Setting {self._index()} value {self.v} is above maximum {self._max}, adjusting to maximum")
            self.v = self._max


    def _index(self):
        for k, v in self._container.items():
            if v == self:
                return k
        return None


    def label(self, index: int | None = None):
        """Return a string label for the setting value.  If index is provided, return the label for that index instead of the current value."""
        if index is not None:
            if self._labels is not None and index < len(self._labels):
                return self._labels[int(index)]
            return str(index)
        if self._labels is not None and self.v is not None and self.v < len(self._labels):
            return self._labels[int(self.v)]
        return str(self.v)


    def info(self, value=None):
        """Build concise help text for this setting and value."""
        if value is None:
            value = self.v
        text = self.description
        text += f"\n\nCurrent: {self.label(value)}"
        text += f"\nDefault: {self.label(self.d)}"
        if self._labels is not None:
            text += "\nOptions: " + ", ".join(self._labels)
        else:
            text += f"\nRange: {self._min} to {self._max}"
        return text


    @staticmethod
    def _quantize_tenths(value: float) -> float:
        """Round to 0.1 steps deterministically to avoid float drift artifacts."""
        scaled = int((value * 10) + (0.5 if value >= 0 else -0.5))
        return scaled / 10.0


    def inc(self, v, l=0):
        """ Increment the setting value.  If l > 0, increment by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.).  If wrap, roll over to the minimum when the maximum is exceeded."""
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l == 0:
                v += 1
            else:
                d = 10 ** l
                v = ((v // d) + 1) * d
            if v > self._max:
                if self._labels is not None or self._wrap:
                    # label-based (or explicitly wrapping) settings wrap around
                    v = self._min
                else:
                    v = self._max
        elif isinstance(self.v, float):
            v = self._quantize_tenths(v) + 0.1
            if v > self._max:
                v = self._max
            v = self._quantize_tenths(v)
        elif self._container['logging'].v:
            print(f"B:inc type: {type(self.v)}")
        return v


    def dec(self, v, l=0):
        """Decrement the setting value.  If l > 0, decrement by the next highest order of magnitude (e.g. 10s place for l=1, 100s place for l=2, etc.).  If wrap, roll over to the maximum when the minimum is exceeded."""
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l == 0:
                v -= 1
            else:
                d = 10 ** l
                v = (((v + (9 * (10 ** (l - 1)))) // d) - 1) * d
            if v < self._min:
                if self._labels is not None or self._wrap:
                    # settings that are purely label-based or explicitly wrapping wrap around
                    v = len(self._labels) - 1 if self._labels is not None else self._max
                else:
                    v = self._min
        elif isinstance(self.v, float):
            v = self._quantize_tenths(v) - 0.1
            if v < self._min:
                v = self._min
            v = self._quantize_tenths(v)
        elif self._container['logging'].v:
            print(f"B: dec type: {type(self.v)}")
        return v


    def persist(self):
        """Persist the setting value to platform storage."""
        index = self._index()
        if index is None:
            return
        key = f"{SETTINGS_NAME_PREFIX}.{index}"
        try:
            platform_settings.set(key, self.v)
        except Exception as e:          # pylint: disable=broad-except
            print(f"B:Failed to persist setting {key}: {e}")


class SettingsMgr:
    """Manages the Settings editing UI.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """
    __slots__ = (
        "_app", "_logging", "edit_setting", "edit_setting_value",
        "_group_ids", "_setting_keys", "_setting_positions", "_active_group",
        "_show_help", "_help_lines", "_help_scroll",
    )

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self.edit_setting: str | None = None
        self.edit_setting_value = None
        self._group_ids = []
        self._setting_keys = []
        self._setting_positions = [0] * len(MySetting.GROUP_NAMES)
        self._active_group: int | None = None
        self._show_help = False
        self._help_lines = None
        self._help_scroll = 0
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


    @staticmethod
    def handles_menu(menu_name: str | None) -> bool:
        """Return whether menu_name belongs to the settings hierarchy."""
        return menu_name == MENU_ENTRY_NAME or (
            menu_name is not None and menu_name.startswith(_GROUP_MENU_PREFIX)
        )


    @staticmethod
    def _group_menu_name(group: int) -> str:
        return f"{_GROUP_MENU_PREFIX}{group}"


    def build_menu(self, menu_name: str):
        """Build one level of the settings menu hierarchy."""
        app = self._app
        if menu_name == MENU_ENTRY_NAME:
            self._group_ids = []
            for group in range(len(MySetting.GROUP_NAMES)):
                for setting in app.settings.values():
                    if setting.group == group:
                        self._group_ids.append(group)
                        break
            menu_items = [MySetting.GROUP_NAMES[group] for group in self._group_ids]
            menu_items.append(_RESET_ITEM)
            position = min(app.settings_menu_position, len(menu_items) - 1)
            return Menu(
                app,
                menu_items,
                select_handler=self._group_select_handler,
                back_handler=self.menu_back,
                position=position,
            )

        if menu_name == _RESET_MENU_NAME:
            return Menu(
                app,
                ["Cancel", "Reset all"],
                select_handler=self._reset_select_handler,
                back_handler=self.menu_back,
            )

        try:
            group = int(menu_name[len(_GROUP_MENU_PREFIX):])
        except ValueError:
            return None
        if group < 0 or group >= len(MySetting.GROUP_NAMES):
            return None

        self._active_group = group
        self._setting_keys = []
        menu_items = []
        info_items = []
        group_settings = [
            (key, setting) for key, setting in app.settings.items()
            if setting.group == group
        ]
        group_settings.sort(key=lambda item: item[1].order)
        for key, setting in group_settings:
            self._setting_keys.append(key)
            menu_items.append(setting.title or key)
            info_items.append(setting.info())
        position = min(self._setting_positions[group], len(menu_items) - 1)
        return Menu(
            app,
            menu_items,
            info_items=info_items,
            select_handler=self._setting_select_handler,
            back_handler=self.menu_back,
            position=position,
        )


    def _group_select_handler(self, _item: str, idx: int):
        app = self._app
        app.settings_menu_position = app.menu.position if app.menu else 0
        if idx >= len(self._group_ids):
            app.set_menu(_RESET_MENU_NAME)
            return
        group = self._group_ids[idx]
        app.set_menu(self._group_menu_name(group))


    def _setting_select_handler(self, _item: str, idx: int):
        if idx < len(self._setting_keys):
            self.start(self._setting_keys[idx])


    def _reset_select_handler(self, _item: str, idx: int):
        app = self._app
        if idx == 0:
            app.set_menu(MENU_ENTRY_NAME)
            return
        for setting in app.settings.values():
            setting.v = setting.d
            setting.persist()
        app.fast_settings_update()
        app.notification = Notification("Settings reset")
        app.set_menu(MENU_ENTRY_NAME)


    def menu_back(self):
        """Return to the parent of the current settings menu."""
        app = self._app
        if app.current_menu == MENU_ENTRY_NAME:
            app.settings_menu_position = app.menu.position if app.menu else 0
            app.set_menu()
        elif app.current_menu == _RESET_MENU_NAME:
            app.set_menu(MENU_ENTRY_NAME)
        else:
            if self._active_group is not None and app.menu is not None:
                self._setting_positions[self._active_group] = app.menu.position
            app.set_menu(MENU_ENTRY_NAME)


    def start(self, item: str) -> bool:
        """Enter setting editing mode from a settings group menu."""
        app = self._app
        if self._active_group is not None and app.menu is not None:
            self._setting_positions[self._active_group] = app.menu.position
        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True
        app.auto_repeat_clear()
        if self._logging:
            print("Entered Settings editing mode")
        self.edit_setting = item
        self.edit_setting_value = app.settings[item].v
        self._show_help = False
        self._help_lines = None
        self._help_scroll = 0
        app.current_state = STATE_SETTINGS
        return True


    def _return_to_group(self):
        group = self._active_group
        self._app.return_to_menu(
            MENU_ENTRY_NAME if group is None else self._group_menu_name(group)
        )


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle Settings editing UI.  Returns True if this module handled the state."""
        app = self._app

        if self._show_help:
            if app.button_states.get(BUTTON_TYPES["UP"]):
                if app.auto_repeat_check(delta):
                    self._help_scroll = max(0, self._help_scroll - 1)
                    app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["DOWN"]):
                if app.auto_repeat_check(delta):
                    line_count = len(self._help_lines) if self._help_lines is not None else 0
                    max_scroll = max(0, line_count - _HELP_VISIBLE_LINES)
                    self._help_scroll = min(max_scroll, self._help_scroll + 1)
                    app.refresh = True
            else:
                app.auto_repeat_clear()
                if (app.button_states.get(BUTTON_TYPES["LEFT"])
                        or app.button_states.get(BUTTON_TYPES["CANCEL"])):
                    app.button_states.clear()
                    self._show_help = False
                    app.refresh = True
            return True

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
            if app.button_states.get(BUTTON_TYPES["LEFT"]):
                app.button_states.clear()
                self.edit_setting_value = app.settings[self.edit_setting].d
                if self._logging:
                    print(f"Setting: {self.edit_setting} Default: {self.edit_setting_value}")
                app.refresh = True
                app.notification = Notification("Default")
            elif app.button_states.get(BUTTON_TYPES["RIGHT"]):
                app.button_states.clear()
                self._show_help = True
                self._help_lines = None
                self._help_scroll = 0
                app.refresh = True
            elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if self._logging:
                    print(f"Setting: {self.edit_setting} Cancelled")
                # now done in return_to_menu ... app.fast_settings_update()  # Update fast access settings which might have been changed
                self._return_to_group()
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                if self._logging:
                    print(f"Setting: {self.edit_setting} = {self.edit_setting_value}")
                app.settings[self.edit_setting].v = self.edit_setting_value
                app.settings[self.edit_setting].persist()
                title = app.settings[self.edit_setting].title or self.edit_setting
                app.notification = Notification(f"{title}={self.edit_setting_value}")
                self._return_to_group()
        return True


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Settings editing UI.  Returns True if handled."""
        app = self._app
        setting = app.settings[self.edit_setting]
        if self._show_help:
            if self._help_lines is None:
                self._help_lines = wrap_text(
                    ctx, setting.info(self.edit_setting_value), small_font_size, _HELP_WIDTH
                )
            max_scroll = max(0, len(self._help_lines) - _HELP_VISIBLE_LINES)
            self._help_scroll = min(self._help_scroll, max_scroll)

            ctx.save()
            ctx.text_align = ctx.CENTER
            ctx.text_baseline = ctx.MIDDLE
            ctx.font_size = label_font_size
            ctx.rgb(1, 1, 0).move_to(0, -62).text(setting.title or self.edit_setting)
            ctx.font_size = small_font_size
            ctx.text_align = ctx.LEFT
            ctx.rgb(1, 1, 1)
            for line_num, line in enumerate(
                    self._help_lines[self._help_scroll:self._help_scroll + _HELP_VISIBLE_LINES]):
                ctx.move_to(-95, -35 + line_num * small_font_size).text(line)
            ctx.restore()
            button_labels(ctx, up_label="Up", down_label="Down", cancel_label="Back", left_label="Close")
            return True

        disp_val = setting.label(self.edit_setting_value)
        title = setting.title or self.edit_setting
        app.draw_message(ctx, ["Edit Setting", f"{title}:", f"{disp_val}"], [(1, 1, 0), (0, 0, 1), (0, 1, 0)], label_font_size)
        button_labels(ctx, up_label="+", down_label="-", confirm_label="Set", cancel_label="Cancel", left_label="Default", right_label="Help")
        return True


    # ------------------------------------------------------------------
