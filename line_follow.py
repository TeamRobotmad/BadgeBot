# Line Follower Module for BadgeBot
#
# Handles the line-following functionality.
# Contains the LineSensors and LineSensor hardware driver classes
# for QTRX reflectance sensors.
#
# Public interface (called by the main app):
#   __init__(app)           – wire up to BadgeBotApp
#   start()                 – enter line follower from menu
#   update(delta)           – per-tick state machine update
#   draw(ctx)               – render line follower UI
#   background_update(delta)– called from the fast background loop;
#                             returns motor output tuple or None
#   init_settings(settings) – register line-follower specific settings
#   create_line_sensors()   – create LineSensors from hexpansion config


import time
import gc
from math import pi
from events.input import BUTTON_TYPES
from app_components.notification import Notification
from app_components.tokens import label_font_size, button_labels
from machine import Pin
try:
    from machine import disable_irq, enable_irq
except ImportError:
    # Simulator doesn't provide interrupt control; use no-op stubs
    def disable_irq():
        return 0
    def enable_irq(_state):
        pass
from system.hexpansion.config import HexpansionConfig

# Line Follower constants
_NUM_LINE_SENSORS = 2
_LINE_SENSOR_DEFAULT_THRESHOLD = 500
_LINE_SENSOR_READ_TIMEOUT_US = 5000            # Maximum expected discharge time for the line sensors; readings above this are ignored as timeouts
_LINE_SENSOR_TRIGGER_DURATION_US = 10
_LINE_SENSOR_UPDATE_PERIOD_MS = 10
_LINE_SENSOR_SAMPLE_RATE_UPDATE_PERIOD_MS = 1000

# PID Gains
FOLLOWER_PID_KP_DEFAULT = 20000
FOLLOWER_PID_KI_DEFAULT = 0
FOLLOWER_PID_KD_DEFAULT = 0

FOLLOWER_FORWARD_POWER = 20000

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

    def __init__(self, sensor_configs):
        self._sensors = [
            LineSensor(cfg["pins"], name=cfg["name"])
            for cfg in sensor_configs
        ]
        self._threshold = 0
        
    @property
    def num_sensors(self):
        return len(self._sensors)

    @property
    def threshold(self):
        return self._threshold

    @threshold.setter
    def threshold(self, value):
        self._threshold = value

    def enable(self):
        for sensor in self._sensors:
            sensor.enable()

    def disable(self):
        for sensor in self._sensors:
            sensor.disable()

    def raw_value(self, index):
        return self._sensors[index].value

    def raw_values(self):
        return [sensor.value for sensor in self._sensors]

    def sample_count(self):
        """Get the total sample count across all sensors."""
        return sum(sensor.sample_count for sensor in self._sensors)
    
    def sample_count_and_reset(self):
        """Atomically get the total sample count across all sensors and reset to zero."""
        count = self.sample_count()
        for sensor in self._sensors:
            sensor.sample_count = 0
        return count

    def read(self):
        for sensor in self._sensors:
            if sensor.start_time != 0:
                if time.ticks_diff(time.ticks_us(), sensor.start_time) <= _LINE_SENSOR_READ_TIMEOUT_US:
                    print(f"Sensor {sensor.name} already in progress")
                    return False

        for sensor in self._sensors:
            sensor.pins["ctrl"].on()
            sensor.pins["sig"].init(mode=Pin.OUT)
            sensor.pins["sig"].on()

        time.sleep_us(_LINE_SENSOR_TRIGGER_DURATION_US)

        irq_state = disable_irq()
        start_time = time.ticks_us()
        for sensor in self._sensors:
            sensor.start_time = start_time
            sensor.pins["sig"].init(mode=Pin.IN, pull=None)
        enable_irq(irq_state)

        return True
    
    #@micropython.native
    def read_blocking(self):
        """Charge, release, and poll for all sensors — no IRQ needed."""

        gc.disable()
        irq_state = disable_irq()
        
        # Charge phase
        for sensor in self._sensors:
            sensor.pins["ctrl"].on()
            sensor.pins["sig"].init(mode=Pin.OUT)
            sensor.pins["sig"].on()
        
        time.sleep_us(_LINE_SENSOR_TRIGGER_DURATION_US)
        
        # Release all sig pins simultaneously
        start = time.ticks_us()
        for sensor in self._sensors:
            sensor.pins["sig"].init(mode=Pin.IN, pull=None)
        
        # Poll until both sensors have fallen or timeout
        done = [False] * len(self._sensors)
        while not all(done):
            now = time.ticks_us()
            elapsed = time.ticks_diff(now, start)
            if elapsed > _LINE_SENSOR_READ_TIMEOUT_US:
                break
            for i, sensor in enumerate(self._sensors):
                if not done[i] and sensor.pins["sig"].value() == 0:
                    sensor.value = elapsed
                    sensor.pins["ctrl"].off()
                    sensor.sample_count += 1
                    done[i] = True
        
        enable_irq(irq_state)
        gc.enable()
        
        # Mark timed-out sensors
        for i, sensor in enumerate(self._sensors):
            if not done[i]:
                sensor.pins["ctrl"].off()

# ---- LineSensor class ------------------------------------------------------

class LineSensor:
    def __init__(self, pins, name="LineSensor"):
        try:
            self._name = name
            self.start_time = 0
            self.value = 0
            self._irq_state = None
            self.sample_count = 0
            self.pins = pins
            self.pins["ctrl"].init(mode=Pin.OUT)
            self.pins["ctrl"].off()
            self.pins["sig"].init(mode=Pin.IN, pull=Pin.PULL_UP)
        except Exception as e:          # pylint: disable=broad-exception-caught
            print(f"{self._name} Init failed:{e}")

    @property
    def name(self) -> str:
        return self._name

    def disable(self):
        """Disable the sensor by turning off control pin and disabling interrupts on signal pin."""
        print(f"Line Sensor {self._name} disabled")
        self.pins["sig"].irq(handler=None)
        self.pins["ctrl"].off()
        self.pins["sig"].off()


    def enable(self):
        """Enable the sensor by enabling interrupts on signal pin."""
        print(f"Line Sensor {self._name} enabled")
        self.pins["sig"].irq(trigger=Pin.IRQ_FALLING, handler=self.handler) # unfortunately hard and priority are not recognised keywords)
        self.start_time = 0


    def handler(self, _):
        """Interrupt handler for the sensor signal pin.
        Measures the time since the sensor was triggered and updates the state."""
        # Currently as the "irq" is not a hardware interrupt we are not guaranteed that the handler will be called immediately
        #  when the signal pin goes low, so instead of relying on the handler to capture the timing 
        #  it is recommended to do a blocking read in the background update loop.
        # However the code here is the leanest way to capture the timing if the IRQ handler is called immediately when the signal pin goes low, so leaving it here for now in case that becomes possible in the future.
        
        #if self.start_time == 0:
        #    # spurious interrupt or handler called before read() set up the start_time; ignore
        #    return
        #self._irq_state = disable_irq()
        value = time.ticks_diff(time.ticks_us(), self.start_time)
        # Handle the hardware
        #self.pins["sig"].irq(handler=None)
        #enable_irq(self._irq_state)
        #self.pins["sig"].off()
        #self.pins["sig"].init(mode=Pin.OUT)
        self.pins["ctrl"].off()
        self.start_time = 0

        if value > _LINE_SENSOR_READ_TIMEOUT_US:
            # timeout expired; ignore this reading
            #print(f"{self._name} reading timed out (value={value}µs)")
            return
        self.value = value
        self.sample_count += 1


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting):
    """Register line-follower-specific settings in the shared settings dict."""
    s['line_threshold'] = MySetting(s, _LINE_SENSOR_DEFAULT_THRESHOLD, 0, 65535)
    s['pid_kp']         = MySetting(s, FOLLOWER_PID_KP_DEFAULT, 0, 65536)
    s['pid_ki']         = MySetting(s, FOLLOWER_PID_KI_DEFAULT, 0, 65535)
    s['pid_kd']         = MySetting(s, FOLLOWER_PID_KD_DEFAULT, 0, 65535)


# ---- Shared helper: create LineSensors from hexpansion config --------------

def create_line_sensors(config: HexpansionConfig, number_of_sensors: int = _NUM_LINE_SENSORS):
    """Create a LineSensors instance from the app's hexpansion config.

    Returns a new LineSensors or None if no config is available.
    Used by both LineFollowMgr and AutotuneMgr to avoid duplicating
    sensor initialisation code.
    """
    if config is None:
        return None
    sensor_configs = [
        {
            "pins": {
                "ctrl": config.pin[SENSOR_CTRL_PINS[i]],
                "sig":  config.pin[SENSOR_SIGNAL_PINS[i]]
            },
            "name": SENSOR_NAMES[i],
        }
        for i in range(number_of_sensors)
    ]
    return LineSensors(sensor_configs)


# ---- Line Follower Manager -------------------------------------------------

class LineFollowMgr:
    """Manages the Line Follower functionality.

    Parameters
    ----------
    app : BadgeBotApp
        Reference to the main application instance.
    """

    def __init__(self, app):
        self.app = app
        # Line Follower
        self._sensor_state = [False, False]
        self.line_sensors = None                               # Will be a LineSensors instance when active
        self.sample_time: int   = 0
        self.sensor_rate: int = 0     # sample rate
        self.follower_mode: int = FOLLOWER_MODE_DIFFERENTIAL   # Default follower mode
        self.forward_power: int = -FOLLOWER_FORWARD_POWER      # Default forward power for line follower (sign sets direction)
        self.pid_integral: int = 0                             # Accumulated integral term for PID controller
        self.pid_previous_error: int = 0                       # Previous error for derivative term of PID controller
        self.kp: int = FOLLOWER_PID_KP_DEFAULT
        self.ki: int = FOLLOWER_PID_KI_DEFAULT
        self.kd: int = FOLLOWER_PID_KD_DEFAULT
        self.max_pwr: int = 0
        self.integral_limit: int = 0
        self.motor_output = (0,0)


    # ------------------------------------------------------------------
    # Entry point from menu
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Enter line follower from the main menu."""
        app = self.app

        if self.line_sensors is None:
            self.line_sensors = create_line_sensors(app.line_sensors_hexpansion_config, app.num_line_sensors)

        if self.line_sensors is None:
            # Line sensors are not available; inform the user and abort line follower.
            Notification(app, "Line sensors not available")
            return False
        else:
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_logging(False)
                if app.hexdrive_app.initialise() and app.hexdrive_app.set_power(True):
                    #self.line_sensors.enable()
                    #self.line_sensors.read()    # initiate first sensor reading
                    self.line_sensors.read_blocking()    # initiate first sensor reading
                    app.update_period = _LINE_SENSOR_UPDATE_PERIOD_MS
                    app.set_menu(None)
                    app.button_states.clear()            
                    app.refresh = True
                    app.auto_repeat_clear()
                    self.motor_output = (0,0)
                    self.kp = app.settings['pid_kp'].v
                    self.ki = app.settings['pid_ki'].v
                    self.kd = app.settings['pid_kd'].v
                    self.max_pwr = app.settings['max_power'].v
                    if self.ki > 0:
                        self.integral_limit = self.max_pwr // self.ki
                    else:
                        self.integral_limit = 0            
                    if app.settings['logging'].v:
                        print("Entered Line Follower mode")
                    return True                    
            if app.settings['logging'].v:
                print("HexDrive not available; Line Follower requires HexDrive to run")
            Notification(app, "HexDrive Init Failed")
            return False                


    # ------------------------------------------------------------------
    # Per-tick update
    # ------------------------------------------------------------------

    def update(self, delta) -> bool:
        """Handle Line Follower UI.  Returns True if handled."""
        app = self.app
    
        self.sample_time += delta
        if app.button_states.get(BUTTON_TYPES["CANCEL"]):
            app.button_states.clear()
            if app.hexdrive_app is not None:
                app.hexdrive_app.set_power(False)
            self.line_sensors.disable()
            app.pid_integral = 0
            app.pid_previous_error = 0
            app.return_to_menu()
            return True
        elif app.button_states.get(BUTTON_TYPES["UP"]):
            app.button_states.clear()
            app.settings['line_threshold'].v = app.settings['line_threshold'].inc(app.settings['line_threshold'].v)
            self.line_sensors.threshold = app.settings['line_threshold'].v
            app.refresh = True
        elif app.button_states.get(BUTTON_TYPES["DOWN"]):
            app.button_states.clear()
            app.settings['line_threshold'].v = app.settings['line_threshold'].dec(app.settings['line_threshold'].v)
            self.line_sensors.threshold = app.settings['line_threshold'].v
            app.refresh = True
        #if self.line_sensors.updated:
        #    app.refresh = True
        #    self.line_sensors.clear_updated()
        if (self.sample_time > _LINE_SENSOR_SAMPLE_RATE_UPDATE_PERIOD_MS):
            sample_count = self.line_sensors.sample_count_and_reset()
            self.sensor_rate = int(((self.sample_time / self.line_sensors.num_sensors) * sample_count) // self.sample_time)
            self.sample_time = 0
            app.refresh = True
        return True


    # ------------------------------------------------------------------
    # Background update (called from the fast loop)
    # ------------------------------------------------------------------

    def background_update(self, delta) -> tuple[int, int] | None:  # pylint: disable=unused-argument
        """Line follower motor control.
        Returns motor output tuple, or None if not active."""
        #app = self.app
        #output = (0, 0)
        #s = self.line_sensors.values()
        #if self.follower_mode == FOLLOWER_MODE_DIFFERENTIAL:
            # PID control
            # Calculate the error as the normalised difference between the two sensor readings
        self.line_sensors.read_blocking()    # wait for sensor reading        
        error = self.compute_error(self.line_sensors.raw_value(0), self.line_sensors.raw_value(1))            
        # self.line_sensors.read()           # initiate next sensor reading (non-blocking, using IRQ handler to capture values when ready)
        output = self.compute_differential_output(error)
        #else:
        #    # Bang Bang control
        #    if s != self._sensor_state:
        #        self._sensor_state = s
        #        if s[0] and not s[1]:
        #            output = (-app.settings['max_power'].v, app.settings['max_power'].v)
        #        elif not s[0] and s[1]:
        #            output = (app.settings['max_power'].v, -app.settings['max_power'].v)
        #        else:
        #            output = (app.settings['max_power'].v, app.settings['max_power'].v)
        #        self.motor_output = output
        #    else:
        #        output = self.motor_output
        return output


    def compute_differential_output(self, error: int) -> tuple[int, int]:
        """Compute motor output using a full PID controller for differential line following.
        
        Uses the difference between left and right sensor readings as the error signal,
        and applies proportional, integral, and derivative terms to compute a steering correction.
        Returns a tuple of (left_motor, right_motor) power values, clamped to max_power.
        """

        # Proportional term
        p_term = (self.kp * error) // 1000  # Scale down the error for the proportional term

        # Integral term - accumulate error over time with anti-windup clamping
        #self.pid_integral += error
        #self.pid_integral = max(min(self.pid_integral, self.integral_limit), -self.integral_limit)
        #i_term = (self.ki * self.pid_integral) // 1000

        # Derivative term - rate of change of error
        #d_term = (self.kd * (error - self.pid_previous_error)) // 1000
        #self.pid_previous_error = error

        # Combined PID output
        # make correction value as integer to avoid issues with motor control expecting int values
        correction = p_term # + int(i_term) + int(d_term)
    
        # Combine correction with base forward power to get output for each motor
        output = (self.forward_power + correction, self.forward_power - correction)

        # Limit output to max power
        output = (max(min(output[0], self.max_pwr), -self.max_pwr), max(min(output[1], self.max_pwr), -self.max_pwr))

        #if self.app.settings['logging'].v:
        #    print(f"PID: err={error} P={p_term} I={i_term} D={d_term} corr={correction} out={output}")

        return output
    
    
    def compute_error(self, left_raw: int, right_raw: int) -> int:
        """Compute a normalised error from raw sensor discharge times.

        Parameters
        ----------
        left_raw : int
            Raw discharge time (µs) from the left sensor.
        right_raw : int
            Raw discharge time (µs) from the right sensor.

        Returns
        -------
        int
            Error in range [-1000, +1000].
            Negative = line is to the left (steer left).
            Positive = line is to the right (steer right).

        The sensor with the *shorter* discharge time is further from the dark line
        (higher reflectance).  So if left_raw > right_raw the line is to the
        left and the error is negative.
        """
        total = left_raw + right_raw
        if total == 0:
            return 0
        return (1000 * (right_raw - left_raw)) // total


    # ------------------------------------------------------------------
    # Draw
    # ------------------------------------------------------------------

    def draw(self, ctx) -> bool:
        """Render Line Follower UI.  Returns True if handled."""
        app = self.app
 
        ctx.save()
        ctx.rgb(1, 1, 0).move_to(0, -1 * label_font_size).text(f"TH:{app.settings['line_threshold'].v}")
        ctx.rgb(1, 1, 1).move_to(-70, -1 * label_font_size).text(f"{self.sensor_rate} Hz")
        spacing = 80
        offset = (spacing // 2) * (app.num_line_sensors // 2)
        for i in range(app.num_line_sensors):
            x = offset - i * spacing
            # make a simple visualization of the sensor reading as a filled circle, with colour indicating whether it's above or below the threshold
            colour = (0, 1, 0) if self.line_sensors.raw_value(i) < app.settings['line_threshold'].v else (0, 0, 0)
            ctx.rgb(*colour).arc(x, 0, 24, 0, 2 * pi, True).fill()
            ctx.rgb(1, 1, 1).arc(x, 0, 25, 0, 2 * pi, True).stroke()
            ctx.rgb(1, 1, 0).move_to(x - 20, 2 * label_font_size).text(f"{self.line_sensors.raw_value(i):4}")
        #    if app.settings['logging'].v:
        #        print(f"Sensor {i}: {self.line_sensors.value(i)} (raw: {self.line_sensors.raw_value(i)})")
        ctx.restore()
        button_labels(ctx, up_label="+", down_label="-", cancel_label="Cancel")
        return True
