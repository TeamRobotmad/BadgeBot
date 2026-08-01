def _press(app, button):
    from events.input import BUTTON_TYPES
    app.button_states.buttons[BUTTON_TYPES[button]] = True


def test_all_setting_registrations_have_ui_metadata(badgebot_app):
    from sim.apps.BadgeBot.line_follow import init_settings as init_line_settings
    from sim.apps.BadgeBot.motor_moves import init_settings as init_drive_settings
    from sim.apps.BadgeBot.servo_test import init_settings as init_servo_settings
    from sim.apps.BadgeBot.settings_mgr import MySetting

    settings = badgebot_app.settings
    init_drive_settings(settings, MySetting)
    init_line_settings(settings, MySetting)
    init_servo_settings(settings, MySetting)

    for key, setting in settings.items():
        assert setting.title, f"Missing title for {key}"
        assert setting.description, f"Missing description for {key}"
        assert 0 <= setting.group < len(MySetting.GROUP_NAMES)
        assert isinstance(setting.order, int)
        assert "Current:" in setting.info()
        assert "Default:" in setting.info()


def test_settings_menu_is_grouped(badgebot_app):
    app = badgebot_app
    app.settings = dict(reversed(list(app.settings.items())))
    app.set_menu("Settings")

    assert app.menu is not None
    assert app.menu.menu_items == ["General", "Motors", "Driving", "Reset all..."]

    app.menu.select_handler("Motors", 1)

    assert app.current_menu == "Settings/1"
    assert app.menu is not None
    assert app.menu.menu_items == [
        "Acceleration", "Deadband", "Max power", "M1 direction",
        "M2 direction", "M1 minimum", "M2 minimum",
    ]
    assert len(app.menu.info_items) == len(app.menu.menu_items)
    assert all(app.menu.info_items)


def test_duplicate_and_gapped_order_hints_are_valid(badgebot_app):
    app = badgebot_app
    app.settings["acceleration"].order = 10
    app.settings["mtr_deadband"].order = 100
    app.settings["max_power"].order = 100
    app.settings["mtr1_dir"].order = 1000
    app.settings["mtr2_dir"].order = 1000
    app.settings["mtr1_min"].order = 5000
    app.settings["mtr2_min"].order = 9000

    app.set_menu("Settings/1")

    assert app.menu is not None
    assert app.menu.menu_items[0] == "Acceleration"
    assert set(app.menu.menu_items[1:3]) == {"Deadband", "Max power"}
    assert set(app.menu.menu_items[3:5]) == {"M1 direction", "M2 direction"}
    assert app.menu.menu_items[5:] == ["M1 minimum", "M2 minimum"]


def test_reset_all_requires_confirmation(badgebot_app):
    app = badgebot_app
    app.set_menu("Settings")
    assert app.menu is not None

    app.menu.select_handler("Reset all...", len(app.menu.menu_items) - 1)

    assert app.current_menu == "Settings/reset"
    assert app.menu is not None
    assert app.menu.menu_items == ["Cancel", "Reset all"]


def test_edit_help_and_default_buttons_return_to_group(badgebot_app):
    from sim.apps.BadgeBot.app import STATE_MENU, STATE_SETTINGS

    app = badgebot_app
    setting = app.settings["max_power"]
    app.set_menu("Settings/1")
    assert app.menu is not None
    setting_index = app.menu.menu_items.index("Max power")
    app.menu.select_handler("Max power", setting_index)

    assert app.current_state == STATE_SETTINGS

    _press(app, "DOWN")
    app.update(0)
    app.button_states.clear()

    _press(app, "RIGHT")
    app.update(0)

    _press(app, "LEFT")
    app.update(0)

    _press(app, "CONFIRM")
    app.update(0)
    assert setting.v == setting.d - 1
    assert app.current_state == STATE_MENU
    assert app.current_menu == "Settings/1"

    assert app.menu is not None
    app.menu.select_handler("Max power", setting_index)
    _press(app, "LEFT")
    app.update(0)

    _press(app, "CONFIRM")
    app.update(0)
    assert setting.v == setting.d
    assert app.current_state == STATE_MENU
    assert app.current_menu == "Settings/1"
