# Servo Tester Module for BadgeBot
#
# Handles the servo tester functionality (STATE_SERVO).
# Allows the user to control up to 4 servos with position, trim,
# and scanning modes.
#
# Public interface (called by the main app):
#   __init__(app)   – wire up to LineFollowerApp
#   start()         – enter servo test from menu
#   update(delta)   – per-tick state machine update
#   draw(ctx)       – render servo tester UI

from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification


# Servo Tester - Defaults
_SERVO_DEFAULT_STEP    = 10
_SERVO_DEFAULT_CENTRE  = 1500
_SERVO_DEFAULT_RANGE   = 1000
_SERVO_DEFAULT_RATE    = 25
_SERVO_DEFAULT_MODE    = 0
_SERVO_DEFAULT_PERIOD  = 20
_SERVO_MAX_RATE        = 1000
_SERVO_MIN_RATE        = 1
_SERVO_MAX_TRIM        = 1000
_MAX_SERVO_RANGE       = 1400


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):
    """Register servo-test-specific settings in the shared settings dict."""
    s['servo_step']    = MySetting(s, _SERVO_DEFAULT_STEP, 1, 100)
    s['servo_range']   = MySetting(s, _SERVO_DEFAULT_RANGE, 100, _MAX_SERVO_RANGE)
    s['servo_period']  = MySetting(s, _SERVO_DEFAULT_PERIOD, 5, 50)


class ServoMode:
    OFF = 0
    TRIM = 1
    POSITION = 2
    SCANNING = 3
    servo_modes = ["OFF", "TRIM", "POSITION", "SCANNING"]

    def __init__(self, mode=OFF):
        self.mode = mode

    def set(self, mode):
        self.mode = mode

    def inc(self):
        self.mode = (self.mode + 1) % 4

    def __eq__(self, other):
        return self.mode == other

    def __str__(self):
        return self.servo_modes[self.mode]


class ServoTestMgr:
    """Manages the Servo Tester functionality.

    Parameters
    ----------
    app : LineFollowerApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app

    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self):
        """Enter servo test from the main menu."""
        app = self.app
        app.set_menu(None)
        app.button_states.clear()
        self.reset_servo()
        from .linefollower import STATE_SERVO
        app.current_state = STATE_SERVO
        app._refresh = True
        app._auto_repeat_clear()

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle STATE_SERVO.  Returns True if this module handled the state."""
        app = self.app
        from .linefollower import STATE_SERVO, STATE_MENU
        if app.current_state != STATE_SERVO:
            return False

        if app.button_states.get(BUTTON_TYPES["RIGHT"]):
            if app._auto_repeat_check(delta, (app.servo_mode[app.servo_selected] != ServoMode.SCANNING)):
                if app.servo_mode[app.servo_selected] == ServoMode.TRIM:
                    app.servo_centre[app.servo_selected] += app._settings['servo_step'].v
                    if app.servo_centre[app.servo_selected] > (_SERVO_DEFAULT_CENTRE + _SERVO_MAX_TRIM):
                        app.servo_centre[app.servo_selected] = _SERVO_DEFAULT_CENTRE + _SERVO_MAX_TRIM
                    if app.hexdrive_app is not None:
                        if not app.hexdrive_app.set_servocentre(app.servo_centre[app.servo_selected], app.servo_selected):
                            print("H:Failed to set servo centre")
                elif app.servo_mode[app.servo_selected] == ServoMode.SCANNING:
                    if app.servo_rate[app.servo_selected] < 0:
                        negative = True
                        rate = -app.servo_rate[app.servo_selected]
                    else:
                        negative = False
                        rate = app.servo_rate[app.servo_selected]
                    rate = app._inc(rate, app._auto_repeat_level)
                    if _SERVO_MAX_RATE < rate:
                        rate = _SERVO_MAX_RATE
                    if negative:
                        app.servo_rate[app.servo_selected] = -rate
                    else:
                        app.servo_rate[app.servo_selected] = rate
                else:
                    if app.servo[app.servo_selected] is None:
                        app.servo[app.servo_selected] = 0
                    app.servo_mode[app.servo_selected].set(ServoMode.POSITION)
                    app.servo[app.servo_selected] += app._settings['servo_step'].v
                if app.servo[app.servo_selected] is not None:
                    if app.servo_range[app.servo_selected] < (app.servo[app.servo_selected] + (app.servo_centre[app.servo_selected] - _SERVO_DEFAULT_CENTRE)):
                        app.servo[app.servo_selected] = app.servo_range[app.servo_selected] - (app.servo_centre[app.servo_selected] - _SERVO_DEFAULT_CENTRE)
                app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["LEFT"]):
            if app._auto_repeat_check(delta, (app.servo_mode[app.servo_selected] != ServoMode.SCANNING)):
                if app.servo_mode[app.servo_selected] == ServoMode.TRIM:
                    app.servo_centre[app.servo_selected] -= app._settings['servo_step'].v
                    if app.servo_centre[app.servo_selected] < (_SERVO_DEFAULT_CENTRE - _SERVO_MAX_TRIM):
                        app.servo_centre[app.servo_selected] = _SERVO_DEFAULT_CENTRE - _SERVO_MAX_TRIM
                    if app.hexdrive_app is not None:
                        if not app.hexdrive_app.set_servocentre(app.servo_centre[app.servo_selected], app.servo_selected):
                            print("H:Failed to set servo centre")
                elif app.servo_mode[app.servo_selected] == ServoMode.SCANNING:
                    if app.servo_rate[app.servo_selected] < 0:
                        negative = True
                        rate = -app.servo_rate[app.servo_selected]
                    else:
                        negative = False
                        rate = app.servo_rate[app.servo_selected]
                    rate = app._dec(rate, app._auto_repeat_level)
                    if _SERVO_MIN_RATE > rate:
                        rate = _SERVO_MIN_RATE
                    if negative:
                        app.servo_rate[app.servo_selected] = -rate
                    else:
                        app.servo_rate[app.servo_selected] = rate
                else:
                    if app.servo[app.servo_selected] is None:
                        app.servo[app.servo_selected] = 0
                    app.servo_mode[app.servo_selected].set(ServoMode.POSITION)
                    app.servo[app.servo_selected] -= app._settings['servo_step'].v
                if app.servo[app.servo_selected] is not None:
                    if -app.servo_range[app.servo_selected] > (app.servo[app.servo_selected] + (app.servo_centre[app.servo_selected] - _SERVO_DEFAULT_CENTRE)):
                        app.servo[app.servo_selected] = -app.servo_range[app.servo_selected] - (app.servo_centre[app.servo_selected] - _SERVO_DEFAULT_CENTRE)
                app._refresh = True
        else:
            app._auto_repeat_clear()
            if app.button_states.get(BUTTON_TYPES["UP"]):
                app.button_states.clear()
                app.servo_selected = (app.servo_selected - 1) % app.num_servos
                app._refresh = True
            elif app.button_states.get(BUTTON_TYPES["DOWN"]):
                app.button_states.clear()
                app.servo_selected = (app.servo_selected + 1) % app.num_servos
                app._refresh = True
            elif app.button_states.get(BUTTON_TYPES["CANCEL"]):
                app.button_states.clear()
                if app.hexdrive_app is not None:
                    app.hexdrive_app.set_power(False)
                    app.hexdrive_app.set_servoposition()
                app.current_state = STATE_MENU
                return True
            elif app.button_states.get(BUTTON_TYPES["CONFIRM"]):
                app.button_states.clear()
                app.servo_mode[app.servo_selected].inc()
                if app.servo_mode[app.servo_selected] == ServoMode.OFF:
                    if app.hexdrive_app is not None:
                        app.hexdrive_app.set_servoposition(app.servo_selected, None)
                else:
                    app._refresh = True
                app.notification = Notification(f"  Servo {app.servo_selected}:\n {app.servo_mode[app.servo_selected]}")

        if app._refresh:
            app._time_since_last_input = 0
        else:
            app._time_since_last_input += delta
            if app._time_since_last_input > app._timeout_period:
                if app.hexdrive_app is not None:
                    app.hexdrive_app.set_power(False)
                    app.hexdrive_app.set_servoposition()
                app.current_state = STATE_MENU
                app.notification = Notification("  Servo:\n Timeout")

        app._time_since_last_update += delta
        if app._time_since_last_update > app._keep_alive_period:
            app._time_since_last_update = 0
            app._refresh = True

        for i in range(app.num_servos):
            _refresh = app._refresh
            if app.servo_mode[i] == ServoMode.SCANNING:
                if app.servo[app.servo_selected] is None:
                    app.servo[app.servo_selected] = 0
                app.servo[i] = app.servo[i] + (10 * app.servo_rate[i] * delta / 1000)
                if app.servo_range[i] < (app.servo[i] + (app.servo_centre[i] - _SERVO_DEFAULT_CENTRE)):
                    app.servo_rate[i] = -app.servo_rate[i]
                    app.servo[i] = app.servo_range[i] - (app.servo_centre[i] - _SERVO_DEFAULT_CENTRE)
                elif -app.servo_range[i] > (app.servo[i] + (app.servo_centre[i] - _SERVO_DEFAULT_CENTRE)):
                    app.servo_rate[i] = -app.servo_rate[i]
                    app.servo[i] = -app.servo_range[i] - (app.servo_centre[i] - _SERVO_DEFAULT_CENTRE)
                _refresh = True
            if _refresh and app.hexdrive_app is not None and app.servo_mode[i] != ServoMode.OFF and app.servo[i] is not None:
                app.hexdrive_app.set_servoposition(i, int(app.servo[i]))

        return True

    # ------------------------------------------------------------------
    # Servo reset
    # ------------------------------------------------------------------

    def reset_servo(self):
        app = self.app
        if app.hexdrive_app is not None:
            app.hexdrive_app.set_power(True)
            app.hexdrive_app.set_freq(1000 // app._settings['servo_period'].v)
        for i in range(4):
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_servocentre(app.servo_centre[app.servo_selected], app.servo_selected)
            app.servo_range[i] = app._settings['servo_range'].v
            if app.servo[i] is not None:
                if app.servo[i] > app.servo_range[i]:
                    app.servo[i] = app.servo_range[i]
                elif app.servo[i] < -app.servo_range[i]:
                    app.servo[i] = -app.servo_range[i]
                if app.hexdrive_app is not None:
                    app.hexdrive_app.set_servoposition(i, int(app.servo[i]))
        app.servo_selected = 0
        app._time_since_last_update = 0
        app._time_since_last_input = 0

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Servo Tester UI.  Returns True if handled."""
        app = self.app
        from .linefollower import STATE_SERVO
        if app.current_state != STATE_SERVO:
            return False

        servo_text = ["S"] * (1 + app.num_servos)
        servo_text_colours = [(0.4, 0.0, 0.0)] * (1 + app.num_servos)
        servo_text[0] = "Servo Test"
        servo_text_colours[0] = (1, 1, 1)
        for i in range(app.num_servos):
            if app.servo[i] is None or app.servo_mode[i] == ServoMode.OFF:
                body_colour = (0.2, 0.2, 0.2)
                bar_colour = (0.4, 0.4, 0.4)
            elif app.servo_mode[i] == ServoMode.SCANNING:
                body_colour = (0.1, 0.5, 0.1)
                bar_colour = (0.1, 1.0, 0.1)
                servo_text_colours[1 + i] = (0.4, 0.0, 0.4)
            else:
                body_colour = (0.1, 0.1, 0.5)
                bar_colour = (0.1, 0.1, 1.0)
                servo_text_colours[1 + i] = (0.4, 0.4, 0.0)

            ctx.save()
            ctx.translate(0, (i - (app.num_servos / 2) + 0.5) * label_font_size)
            background_colour = (0.1, 0.1, 0.1) if i != app.servo_selected else (0.15, 0.15, 0.15)
            ctx.rgb(*background_colour).rectangle(-100, 1, 200, label_font_size - 2).fill()
            c = 100 * (app.servo_centre[i] - _SERVO_DEFAULT_CENTRE) / app.servo_range[i]
            if app.servo[i] is not None:
                x = 100 * (app.servo[i] + app.servo_centre[i] - _SERVO_DEFAULT_CENTRE) / app.servo_range[i]
                ctx.rgb(*bar_colour).rectangle(x - 2, 1, 5, label_font_size - 2).fill()
                ctx.rgb(*body_colour)
                if x > (c + 4):
                    ctx.rectangle(c + 1, 3, x - c - 4, label_font_size - 6).fill()
                elif x < (c - 4):
                    ctx.rectangle(x + 4, 3, c - x - 4, label_font_size - 6).fill()
            ctx.rgb(0, 0, 0).move_to(c, 0).line_to(c, label_font_size).stroke()
            ctx.restore()
            if app.servo_mode[i] == ServoMode.SCANNING:
                servo_text[i + 1] = f"{int(abs(app.servo_rate[i])):4}/s"
            else:
                servo_text[i + 1] = "Off" if (app.servo[i] is None or app.servo_mode[i] == ServoMode.OFF) else f"{int(app.servo[i]):+5} "
        servo_text_colours[1 + app.servo_selected] = tuple(int(j * 2.5) for j in servo_text_colours[1 + app.servo_selected])
        app.draw_message(ctx, servo_text, servo_text_colours, label_font_size)
        if app.servo_mode[app.servo_selected] == ServoMode.SCANNING:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="Slower", right_label="Faster")
        elif app.servo_mode[app.servo_selected] == ServoMode.TRIM:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="Trim-", right_label="+Trim")
        else:
            button_labels(ctx, up_label="^", down_label="\u25BC", confirm_label="Mode", cancel_label="Exit", left_label="<--", right_label="-->")
        return True
