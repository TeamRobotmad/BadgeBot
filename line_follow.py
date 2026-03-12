# Line Follower Module for BadgeBot
#
# Handles the line-following functionality (STATE_FOLLOWER).
# Contains the LineSensors and LineSensor hardware driver classes
# for QTRX reflectance sensors.
#
# Public interface (called by the main app):
#   __init__(app)           – wire up to LineFollowerApp
#   start()                 – enter line follower from menu
#   update(delta)           – per-tick state machine update
#   draw(ctx)               – render line follower UI
#   background_update(delta)– called from the fast background loop

import time
from math import pi
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from machine import Pin, disable_irq, enable_irq


# Line Follower constants
_NUM_LINE_SENSORS = 2
_LINE_SENSOR_DEFAULT_THRESHOLD = 500
_LINE_SENSOR_TIMEOUT = 2000
_LINE_SENSOR_READ_TIMEOUT_US = 10000

FOLLOWER_SENSOR_SCAN_PERIOD = 10
FOLLOWER_SENSOR_TRIGGER_DURATION_US = 10
DEFAULT_UPDATE_PERIOD = 50
SAMPLE_RATE_UPDATE_INTERVAL = 1000

# PID Gains
FOLLOWER_PID_KP_DEFAULT = 10
FOLLOWER_PID_KI_DEFAULT = 0
FOLLOWER_PID_KD_DEFAULT = 0
FOLLOWER_FORWARD_POWER = 25000

# Line Follower Modes
FOLLOWER_MODE_DIFFERENTIAL = 0
FOLLOWER_MODE_BINARY = 1

# Dedicated Pins
SENSOR_1_CTRL   = 3
SENSOR_1_SIGNAL = 2
SENSOR_2_CTRL   = 1
SENSOR_2_SIGNAL = 0
SENSOR_CTRL_PINS   = [SENSOR_1_CTRL,   SENSOR_2_CTRL]
SENSOR_SIGNAL_PINS = [SENSOR_1_SIGNAL, SENSOR_2_SIGNAL]
SENSOR_NAMES       = ["Left", "Right"]


# ---- LineSensors (plural) class -------------------------------------------

class LineSensors:
    """Manages multiple QTRX line sensors efficiently.

    All ctrl/sig pins are pulsed simultaneously (single 10µs charge for all)
    and timing starts at the same moment, so all sensor responses are measured
    in parallel rather than sequentially.
    """

    def __init__(self, container, sensor_configs, threshold=_LINE_SENSOR_DEFAULT_THRESHOLD):
        self._container = container
        self._threshold = threshold
        self._sensors = [
            LineSensor(container, cfg["pins"], name=cfg["name"], threshold=threshold)
            for cfg in sensor_configs
        ]

    @property
    def num_sensors(self):
        return len(self._sensors)

    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, value):
        self._threshold = value
        for sensor in self._sensors:
            sensor.threshold = value

    def enable(self):
        for sensor in self._sensors:
            sensor.enable()

    def disable(self):
        for sensor in self._sensors:
            sensor.disable()

    def value(self, index):
        return self._sensors[index]._state

    def values(self):
        return [sensor._state for sensor in self._sensors]

    def raw_value(self, index):
        return self._sensors[index]._diff

    def raw_values(self):
        return [sensor._diff for sensor in self._sensors]

    @property
    def updated(self):
        return any(sensor.updated for sensor in self._sensors)

    def clear_updated(self):
        for sensor in self._sensors:
            sensor.updated = False

    def read(self):
        for sensor in self._sensors:
            if sensor._start_time != 0:
                if time.ticks_diff(time.ticks_us(), sensor._start_time) <= _LINE_SENSOR_READ_TIMEOUT_US:
                    print(f"Sensor {sensor._name} already in progress")
                    return False

        for sensor in self._sensors:
            sensor.updated = False

        for sensor in self._sensors:
            sensor._pins["ctrl"].on()
            sensor._pins["sig"].init(mode=Pin.OUT)
            sensor._pins["sig"].on()

        time.sleep_us(FOLLOWER_SENSOR_TRIGGER_DURATION_US)

        for sensor in self._sensors:
            sensor._pins["sig"].irq(trigger=Pin.IRQ_FALLING, handler=sensor._handler)

        irq_state = disable_irq()
        start_time = time.ticks_us()
        for sensor in self._sensors:
            sensor._start_time = start_time
            sensor._pins["sig"].init(mode=Pin.IN, pull=None)
        enable_irq(irq_state)

        return True


# ---- LineSensor class ------------------------------------------------------

class LineSensor:
    def __init__(self, container, pins, name="LineSensor",
                 threshold=_LINE_SENSOR_DEFAULT_THRESHOLD):
        try:
            self._container = container
            self._threshold = threshold
            self._state = False
            self._prev_state = False
            self._name = name
            self._start_time = 0
            self._diff = 0
            self._irq_state = None
            self.updated = False
            self._phase = 0

            self._pins = pins
            self._pins["ctrl"].init(mode=Pin.OUT)
            self._pins["ctrl"].off()
            self._pins["sig"].init(mode=Pin.IN, pull=Pin.PULL_UP)
        except Exception as e:
            print(f"{self._name} Init failed:{e}")

    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, value):
        self._threshold = value

    def disable(self):
        self._pins["ctrl"].off()
        self._pins["sig"].irq(handler=None)

    def enable(self):
        self._start_time = 0

    def _handler(self, _):
        self._irq_state = disable_irq()
        self._diff = time.ticks_diff(time.ticks_us(), self._start_time)
        self._pins["sig"].irq(handler=None)
        enable_irq(self._irq_state)
        self._pins["sig"].off()
        self._pins["sig"].init(mode=Pin.OUT)
        self._pins["ctrl"].off()

        if self._start_time == 0:
            return
        self._prev_state = self._state
        self._state = True if self._diff < self._threshold else False
        if (self._prev_state != self._state):
            self.updated = True
        self._start_time = 0
        self._container._sample_count += 1


# ---- Line Follower Manager -------------------------------------------------

class LineFollowMgr:
    """Manages the Line Follower functionality.

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
        """Enter line follower from the main menu."""
        app = self.app
        app.set_menu(None)
        app.button_states.clear()
        app._animation_counter = 0
        if app._line_sensors is None and app._line_sensors_hexpansion_config is not None:
            sensor_configs = [
                {"pins": {"ctrl": app._line_sensors_hexpansion_config.pin[SENSOR_CTRL_PINS[i]],
                          "sig":  app._line_sensors_hexpansion_config.pin[SENSOR_SIGNAL_PINS[i]]},
                 "name": SENSOR_NAMES[i]}
                for i in range(app.num_line_sensors)
            ]
            app._line_sensors = LineSensors(app, sensor_configs, threshold=app._settings['line_threshold'].v)
        if app._line_sensors is None:
            from app_components.notification import Notification
            app.notification = Notification("No Line Sensors")
        else:
            app._line_sensors.enable()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_logging(False)
                if not app.hexdrive_app.set_power(True):
                    print("Failed to enable HexDrive power")
            else:
                print("No HexDrive App")
            from .linefollower import STATE_FOLLOWER
            app.current_state = STATE_FOLLOWER
            app._update_period = FOLLOWER_SENSOR_SCAN_PERIOD
        app._refresh = True

    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta):
        """Handle STATE_FOLLOWER.  Returns True if handled."""
        app = self.app
        from .linefollower import STATE_FOLLOWER, STATE_MENU
        if app.current_state != STATE_FOLLOWER:
            return False

        app._sample_time += delta
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_power(False)
            app._line_sensors.disable()
            app._update_period = DEFAULT_UPDATE_PERIOD
            app._pid_integral = 0
            app._pid_previous_error = 0
            app.current_state = STATE_MENU
            return True
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            app._settings['line_threshold'].v = app._settings['line_threshold'].inc(app._settings['line_threshold'].v)
            app._line_sensors.threshold = app._settings['line_threshold'].v
            app._refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            app._settings['line_threshold'].v = app._settings['line_threshold'].dec(app._settings['line_threshold'].v)
            app._line_sensors.threshold = app._settings['line_threshold'].v
            app._refresh = True
        if app._line_sensors.updated:
            app._refresh = True
            app._line_sensors.clear_updated()
        if (app._sample_time > SAMPLE_RATE_UPDATE_INTERVAL):
            state = disable_irq()
            app._rate = int(((SAMPLE_RATE_UPDATE_INTERVAL / _NUM_LINE_SENSORS) * app._sample_count) // app._sample_time)
            app._sample_count = 0
            enable_irq(state)
            app._sample_time = 0
            app._refresh = True
        return True

    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta):
        """Line follower motor control during STATE_FOLLOWER."""
        app = self.app
        from .linefollower import STATE_FOLLOWER
        if app.current_state != STATE_FOLLOWER:
            return
        output = (0, 0)
        if type(app._override) is bool:
            if not app._override:
                s = app._line_sensors.values()
                app._line_sensors.read()
                if app._follower_mode == FOLLOWER_MODE_DIFFERENTIAL:
                    output = app._compute_differential_output()
                else:
                    if s != app._s:
                        app._s = s
                        if s[0] and not s[1]:
                            output = (-app._settings['max_power'].v, app._settings['max_power'].v)
                        elif not s[0] and s[1]:
                            output = (app._settings['max_power'].v, -app._settings['max_power'].v)
                        else:
                            output = (app._settings['max_power'].v, app._settings['max_power'].v)
                        app._output = output
                    else:
                        output = app._output
        else:
            if app._override == BUTTON_TYPES["UP"]:
                output = (app._settings['max_power'].v, app._settings['max_power'].v)
            elif app._override == BUTTON_TYPES["DOWN"]:
                output = (-app._settings['max_power'].v, -app._settings['max_power'].v)
            elif app._override == BUTTON_TYPES["LEFT"]:
                output = (-app._settings['max_power'].v, app._settings['max_power'].v)
            elif app._override == BUTTON_TYPES["RIGHT"]:
                output = (app._settings['max_power'].v, -app._settings['max_power'].v)
        app.hexdrive_app.set_motors(output)

    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx):
        """Render Line Follower UI.  Returns True if handled."""
        app = self.app
        from .linefollower import STATE_FOLLOWER
        if app.current_state != STATE_FOLLOWER:
            return False

        ctx.save()
        ctx.rgb(1, 1, 0).move_to(0, -1 * label_font_size).text(f"TH:{app._settings['line_threshold'].v}")
        ctx.rgb(1, 1, 1).move_to(-70, -1 * label_font_size).text(f"{app._rate} Hz")
        for i in range(app.num_line_sensors):
            x = 40 - i * 80
            colour = (0, 1, 0) if app._line_sensors.value(i) else (0, 0, 0)
            ctx.rgb(*colour).arc(x, 0, 24, 0, 2 * pi, True).fill()
            ctx.rgb(1, 1, 1).arc(x, 0, 25, 0, 2 * pi, True).stroke()
            ctx.rgb(1, 1, 0).move_to(x - 20, 2 * label_font_size).text(f"{app._line_sensors.raw_value(i):4}")
            if app._settings['logging'].v:
                print(f"Sensor {i}: {app._line_sensors.value(i)} (raw: {app._line_sensors.raw_value(i)})")
        ctx.restore()
        button_labels(ctx, up_label="+", down_label="-", cancel_label="Cancel")
        return True
