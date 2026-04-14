"""High-level motor controller for BadgeBot.

Provides simple awaitable commands such as:

    mc = MotorController(hexdrive_app, settings)
    await mc.forward(500)        # drive forward for 500 ms
    await mc.turn(90)            # turn 90° clockwise using the gyro
    await mc.turn(-45)           # turn 45° anti-clockwise
    await mc.backward(300)       # reverse for 300 ms
    mc.stop()                    # immediate stop

The controller handles acceleration ramps internally and uses the
on-board IMU gyroscope for accurate heading changes.
"""

import asyncio
import time
from math import cos, sin, radians
from typing import TYPE_CHECKING, Callable

if TYPE_CHECKING:
    from .settings_mgr import MySetting

try:
    import imu as _imu
except ImportError:
    _imu = None

from .app import MOTOR_PWM_FREQ

# Constants inlined from Sensor_Testing constants.py to avoid splitting
# application constants into a separate module.
_TICK_MS                = 10        # Smallest unit of change for power, in ms
_AUTO_GYRO_AXIS         = 2         # index into gyro_read() tuple for yaw (0=X,1=Y,2=Z)
_AUTO_GYRO_DEADBAND_DPS = 3.0       # ignore gyro readings below this magnitude (noise floor)
_AUTO_ACCEL_AXIS        = 0         # index into acc_read() tuple for fwd/back (0=X,1=Y,2=Z)
_AUTO_ACCEL_DEADBAND    = 0.05      # m/s² - ignore accel readings below this (vibration/noise)
_AUTO_ACCEL_LPF_ALPHA   = 0.6       # low-pass filter coefficient (0-1, higher = faster response)
_AUTO_ACCEL_SCALE       = 100       # distance calibration % (100 = 1:1, raise if undershooting)
_AUTO_DRIVE_TIMEOUT_MS  = 30000     # safety timeout for distance-based drives (ms)

# ---------------------------------------------------------------------------
# Defaults (can be overridden via constructor kwargs)
# ---------------------------------------------------------------------------
_DEFAULT_TURN_SPEED_FRAC = 0.85     # fraction of max_power used when turning
_DEFAULT_DRIVE_SPEED_FRAC = 1.0     # fraction of max_power used when driving
_DEFAULT_TURN_TIMEOUT_MS = 10000    # safety cap on any single turn
_DEFAULT_DRIVE_TIMEOUT_MS = _AUTO_DRIVE_TIMEOUT_MS  # safety cap on distance drives
_DEFAULT_UPDATE_MS = 10             # how often the control loop ticks
_DEFAULT_SETTLE_MS = 50             # brief pause after stopping motors

MotorOutputTuple = tuple[int, ...]


class MotorController:
    """High-level, async-friendly motor controller.

    Parameters
    ----------
    hexdrive_app : HexDriveApp
        The low-level HexDrive interface (``set_motors``, ``set_power``).
    settings : dict[str, MySetting]
        The shared BadgeBot settings dict (needs ``max_power``,
        ``acceleration``).
    front_face_setting : MySetting | None
        The ``front_face`` setting (0-11).  Each step is 30° CW around
        the badge.  The accelerometer X/Y axes are rotated by this
        amount so that the forward acceleration is measured correctly
        regardless of how the motors are mounted.
    apply_motor_directions_callback : Callable[[MotorOutputTuple], MotorOutputTuple] | None
        Callback that applies per-motor direction settings to an output
        tuple before values are sent to HexDrive. If ``None``, outputs
        are sent unchanged.
    gyro_axis : int
        Index into ``imu.gyro_read()`` for the yaw axis (default 2).
    gyro_deadband : float
        Minimum gyro reading (°/s) before it counts (default 3.0).
    accel_axis : int
        Index into ``imu.acc_read()`` for the forward/back axis (default 0).
    accel_deadband : float
        Minimum accel reading (m/s²) before it counts (default 0.15).
    accel_lpf_alpha : float
        Low-pass filter coefficient for accel smoothing (0–1, default 0.3).
    turn_speed_frac : float
        Fraction of ``max_power`` used for turning (default 0.85).
    drive_speed_frac : float
        Fraction of ``max_power`` used for driving (default 1.0).
    turn_timeout_ms : int
        Safety timeout for a single turn command (default 10 000 ms).
    drive_timeout_ms : int
        Safety timeout for a distance-based drive (default 30 000 ms).
    update_ms : int
        Control-loop tick period in ms (default 10).
    """

    def __init__(
        self,
        hexdrive_app,
        settings,
        *,
        logging=False,
        front_face_setting=None,
        apply_motor_directions_callback=None,
        gyro_axis=_AUTO_GYRO_AXIS,
        gyro_deadband=_AUTO_GYRO_DEADBAND_DPS,
        accel_axis=_AUTO_ACCEL_AXIS,
        accel_deadband=_AUTO_ACCEL_DEADBAND,
        accel_lpf_alpha=_AUTO_ACCEL_LPF_ALPHA,
        turn_speed_frac=_DEFAULT_TURN_SPEED_FRAC,
        drive_speed_frac=_DEFAULT_DRIVE_SPEED_FRAC,
        turn_timeout_ms=_DEFAULT_TURN_TIMEOUT_MS,
        drive_timeout_ms=_DEFAULT_DRIVE_TIMEOUT_MS,
        update_ms=_DEFAULT_UPDATE_MS,
    ):
        self._hexdrive = hexdrive_app
        self._settings = settings
        self._logging: bool = logging
        self._front_face_setting: "MySetting | None" = front_face_setting
        self._apply_motor_directions_callback: "Callable[[MotorOutputTuple], MotorOutputTuple] | None" = apply_motor_directions_callback
        self._gyro_axis: int = gyro_axis
        self._gyro_deadband: float = gyro_deadband
        self._accel_axis: int = accel_axis
        self._accel_deadband: float = accel_deadband
        self._accel_lpf_alpha: float = accel_lpf_alpha
        self._turn_speed_frac: float = turn_speed_frac
        self._drive_speed_frac: float = drive_speed_frac
        self._turn_timeout_ms: int = turn_timeout_ms
        self._drive_timeout_ms: int = drive_timeout_ms
        self._update_ms: int = update_ms

        # Live state (read-only for callers)
        self.motor_output = (0, 0)           # current PWM sent to motors
        self.gyro_dps = 0.0                  # latest gyro reading (deg/s)
        self.integrated_deg = 0.0            # degrees turned since reset
        self.accel_mps2 = 0.0                # latest filtered accel (m/s2)
        self.velocity_mps = 0.0              # integrated velocity (m/s)
        self.distance_m = 0.0                # integrated distance (m)
        self._accel_filtered = 0.0           # low-pass filtered accel
        self._accel_bias_x = 0.0             # calibrated X-axis bias
        self._accel_bias_y = 0.0             # calibrated Y-axis bias
        self._accel_calibrated = False
        self._ramp_overshoot_m = 0.0         # estimated extra distance during ramp-down
        self._avg_loop_ms = _TICK_MS           # measured average loop period
        self._busy = False    
        if self._logging:
            print("MotorController initialised")

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------


    # ------------------------------------------------------------------

    @property
    def logging(self) -> bool:
        """Whether to print diagnostic messages about motor controller activity."""
        return self._logging
    
    @logging.setter
    def logging(self, value: bool):
        self._logging = value

    @property
    def max_power(self) -> int:
        """Maximum motor power (PWM value) from settings."""
        return int(self._settings['max_power'].v)


    @property
    def acceleration(self) -> int:
        """Acceleration for ramps, in motor PWM per second."""
        return max(1, int(self._settings['acceleration'].v))


    @property
    def drive_step_ms(self) -> int:
        """Estimated time in ms to drive one step (for time-based driving)."""
        return int(self._settings['drive_step_ms'].v) if 'drive_step_ms' in self._settings else 0
    

    @property
    def turn_step_ms(self) -> int:
        """Estimated time in ms to turn one step (for time-based turning)."""
        return int(self._settings['turn_step_ms'].v) if 'turn_step_ms' in self._settings else 0


    @property
    def is_busy(self):
        """True while a command is executing."""
        return self._busy
    

    # ------------------------------------------------------------------
    # Public high-level commands (all awaitable)
    # ------------------------------------------------------------------

    async def forward(self, duration_ms, *, speed_frac=None):
        """Drive both motors forward for *duration_ms* milliseconds.

        Parameters
        ----------
        duration_ms : int
            How long to drive (ms).  Includes ramp-up and ramp-down.
        speed_frac : float | None
            Override the default drive speed as a fraction of max_power.
        """
        frac = speed_frac if speed_frac is not None else self._drive_speed_frac
        target = int(self.max_power * frac)
        await self._timed_drive((target, target), duration_ms)


    async def forward_mm(self, distance_mm, *, speed_frac=None,
                         timeout_ms=None):
        """Drive forward until the accelerometer estimates *distance_mm* travelled.

        Uses double-integration of the on-board accelerometer.  Accuracy is
        approximate (~10-20 % on smooth surfaces) because accelerometer
        dead-reckoning accumulates drift.  Best for short distances
        (< 1 m) at moderate speed.

        Parameters
        ----------
        distance_mm : float
            Target distance in millimetres (positive).
        speed_frac : float | None
            Override the default drive speed as a fraction of max_power.
        timeout_ms : int | None
            Override the safety timeout for this drive.
        """
        frac = speed_frac if speed_frac is not None else self._drive_speed_frac
        target = int(self.max_power * frac)
        await self._distance_drive((target, target), abs(distance_mm), timeout_ms)


    async def backward(self, duration_ms, *, speed_frac=None):
        """Drive both motors backward for *duration_ms* milliseconds."""
        frac = speed_frac if speed_frac is not None else self._drive_speed_frac
        target = int(self.max_power * frac)
        await self._timed_drive((-target, -target), duration_ms)


    async def backward_mm(self, distance_mm, *, speed_frac=None,
                          timeout_ms=None):
        """Drive backward until the accelerometer estimates *distance_mm* travelled.

        See ``forward_mm`` for accuracy notes.
        """
        frac = speed_frac if speed_frac is not None else self._drive_speed_frac
        target = int(self.max_power * frac)
        await self._distance_drive((-target, -target), abs(distance_mm), timeout_ms)


    async def turn(self, degrees, *, speed_frac=None,
                   timeout_ms=None):
        """Turn *degrees* using the gyro for feedback.

        Positive = clockwise, negative = anti-clockwise.
        Falls back to a time estimate if the IMU is unavailable.

        Parameters
        ----------
        degrees : float
            Angle to turn (positive = CW, negative = CCW).
        speed_frac : float | None
            Override the default turn speed as a fraction of max_power.
        timeout_ms : int | None
            Override the safety timeout for this turn.
        """
        if degrees == 0:
            return
        frac = speed_frac if speed_frac is not None else self._turn_speed_frac
        timeout = timeout_ms if timeout_ms is not None else self._turn_timeout_ms
        target_deg = abs(degrees)
        direction = 1 if degrees > 0 else -1  # +1 = CW, -1 = CCW
        speed = int(self.max_power * frac)

        # CW  = left motor fwd, right motor rev
        # CCW = left motor rev, right motor fwd
        target_output = (speed * direction, -speed * direction)

        dir_label = "CW" if direction > 0 else "CCW"
        print("[MC-DIAG] turn start: %.1f deg %s, speed=%d, timeout=%d ms"
              % (target_deg, dir_label, speed, timeout))

        self._busy = True
        self.integrated_deg = 0.0
        elapsed_ms = 0
        loop_count = 0
        peak_dps = 0.0
        last_time = time.ticks_ms()

        try:
            self._power_on()

            while elapsed_ms < timeout:
                cur_time = time.ticks_ms()
                delta = time.ticks_diff(cur_time, last_time)
                last_time = cur_time

                # Ramp toward target
                self._ramp_toward(target_output, delta)
                self._send_output()

                # Integrate gyro
                self._read_gyro(delta)
                elapsed_ms += delta
                loop_count += 1
                if abs(self.gyro_dps) > peak_dps:
                    peak_dps = abs(self.gyro_dps)

                # Periodic progress (every ~250 ms)
                if loop_count % 25 == 0 and self._logging:
                    remaining = target_deg - self.integrated_deg
                    print("[MC-DIAG]   turn progress: %.1f / %.1f deg  "
                          "gyro=%.1f dps  elapsed=%d ms  remaining=%.1f deg"
                          % (self.integrated_deg, target_deg,
                             self.gyro_dps, elapsed_ms, remaining))

                if self.integrated_deg >= target_deg:
                    break

                await asyncio.sleep_ms(self._update_ms)

            timed_out = elapsed_ms >= timeout
            overshoot = self.integrated_deg - target_deg
            avg_loop = elapsed_ms / loop_count if loop_count else 0

            # Ramp down to stop
            await self._ramp_stop()

            # Read a few more gyro samples during coast-down
            post_stop_deg = self.integrated_deg
            for _ in range(5):
                t = time.ticks_ms()
                d = time.ticks_diff(t, last_time)
                last_time = t
                self._read_gyro(d)
                await asyncio.sleep_ms(self._update_ms)
            coast_deg = self.integrated_deg - post_stop_deg
            if self._logging:
                print("[MC-DIAG] turn done: integrated=%.2f deg  target=%.1f deg  "
                    "overshoot=%.2f deg" % (self.integrated_deg, target_deg, overshoot))
                print("[MC-DIAG]   elapsed=%d ms  loops=%d  avg_loop=%.1f ms"
                    % (elapsed_ms, loop_count, avg_loop))
                print("[MC-DIAG]   peak_dps=%.1f  coast_after_stop=%.2f deg"
                    % (peak_dps, coast_deg))
                if timed_out:
                    print("[MC-DIAG]   WARNING: turn timed out before reaching target")
        finally:
            self._busy = False


    async def turn_left(self, degrees, **kwargs):
        """Convenience: turn anti-clockwise by *degrees*."""
        await self.turn(-abs(degrees), **kwargs)


    async def turn_right(self, degrees, **kwargs):
        """Convenience: turn clockwise by *degrees*."""
        await self.turn(abs(degrees), **kwargs)


    async def timed_turn(self, duration_ms, direction=1, *, speed_frac=None):
        """Turn for *duration_ms* milliseconds without gyro feedback.

        Parameters
        ----------
        duration_ms : int
            How long to turn (ms).  Includes ramp-up and ramp-down.
        direction : int
            +1 = clockwise, -1 = anti-clockwise.
        speed_frac : float | None
            Override the default turn speed as a fraction of max_power.
        """
        frac = speed_frac if speed_frac is not None else self._turn_speed_frac
        speed = int(self.max_power * frac)
        # CW  = left motor fwd, right motor rev
        # CCW = left motor rev, right motor fwd
        target = (speed * direction, -speed * direction)
        await self._timed_drive(target, abs(duration_ms))


    async def timed_turn_left(self, duration_ms, **kwargs):
        """Time-based anti-clockwise turn."""
        await self.timed_turn(abs(duration_ms), direction=-1, **kwargs)


    async def timed_turn_right(self, duration_ms, **kwargs):
        """Time-based clockwise turn."""
        await self.timed_turn(abs(duration_ms), direction=1, **kwargs)


    def stop(self):
        """Immediately zero the motors (non-async, no ramp)."""
        self.motor_output = (0, 0)
        self._send_output()
        if self._hexdrive is not None:
            self._hexdrive.set_power(False)
        self._busy = False


    async def brake(self):
        """Ramp the motors to zero then cut power."""
        await self._ramp_stop()
        if self._hexdrive is not None:
            self._hexdrive.set_power(False)


    # ------------------------------------------------------------------
    # Instruction-list helpers  (drop-in for the old power-plan system)
    # ------------------------------------------------------------------

    async def run_instructions(self, instructions):
        """Execute a list of ``Instruction`` objects sequentially.

        Each instruction is translated into the appropriate ``forward``,
        ``backward``, or ``turn`` call.  When the ``drive_mode`` setting
        is 1 (Distance), UP/DOWN use the accelerometer-based
        ``forward_mm`` / ``backward_mm`` with ``drive_step_mm`` as the
        distance per step.  Otherwise time-based driving is used with
        ``drive_step_ms``.

        Turns always use the gyro regardless of drive mode.

        Parameters
        ----------
        instructions : list[Instruction]
            The recorded instruction sequence (UP/DOWN/LEFT/RIGHT with
            duration multipliers).
        """
        from events.input import BUTTON_TYPES

        use_distance = True # (self._settings.get('drive_mode') is not None and int(self._settings['drive_mode'].v) == 1)

        self._power_on()
        try:
            for instr in instructions:
                btn = instr.press_type
                count = instr.duration

                if btn == BUTTON_TYPES["UP"]:
                    if use_distance:
                        # temprarily use drive_step_ms as the time per step to estimate distance, until we have a real distance-per-step calibration value
                        mm = self.drive_step_ms * count
                        await self.forward_mm(mm)
                    else:
                        dur = self.drive_step_ms * count
                        await self.forward(dur)
                elif btn == BUTTON_TYPES["DOWN"]:
                    if use_distance:
                        # temprarily use drive_step_ms as the time per step to estimate distance, until we have a real distance-per-step calibration value
                        mm = self.drive_step_ms * count
                        await self.backward_mm(mm)
                    else:
                        dur = self.drive_step_ms * count
                        await self.backward(dur)
                elif btn == BUTTON_TYPES["LEFT"]:
                    if use_distance:
                        # temprarily use turn_step_ms as the time per step to estimate distance, until we have a real distance-per-step calibration value
                        deg = self.turn_step_ms * count
                        await self.turn_left(deg)
                    else:
                        dur = self.turn_step_ms * count
                        await self.timed_turn_left(dur)
                elif btn == BUTTON_TYPES["RIGHT"]:
                    if use_distance:
                        # temprarily use turn_step_ms as the time per step to estimate distance, until we have a real distance-per-step calibration value
                        deg = self.turn_step_ms * count
                        await self.turn_right(deg)
                    else:
                        dur = self.turn_step_ms * count
                        await self.timed_turn_right(dur)
        finally:
            await self.brake()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _power_on(self):
        if self._hexdrive is not None:
            self._hexdrive.set_power(True)
            self._hexdrive.set_freq(MOTOR_PWM_FREQ)

    def _get_front_angle_rad(self):
        """Return the front-face rotation in radians.

        ``front_face`` 0–11 maps to 0°–330° in 30° steps CW.  We negate
        the angle because we need to *undo* the rotation to project
        the accelerometer reading back into the robot’s forward axis.
        """
        if self._front_face_setting is None:
            return 0.0
        return radians(-(int(self._front_face_setting.v) * 30))

    async def _calibrate_accel(self):
        """Sample the accelerometer while stationary to estimate bias.

        Both X and Y axes are sampled so the forward projection
        (rotated by ``front_face``) can be bias-corrected properly.
        """
        if _imu is None:
            self._accel_calibrated = False
            return
        total_x = 0.0
        total_y = 0.0
        samples_x = []
        samples_y = []
        n = 10
        for _ in range(n):
            try:
                raw = _imu.acc_read()
                sx = float(raw[0])
                sy = float(raw[1])
                total_x += sx
                total_y += sy
                samples_x.append(sx)
                samples_y.append(sy)
            except Exception:           # pylint: disable=broad-exception-caught
                # Ignore individual read failures; we'll check how many succeeded below.
                pass
            # we need to wait a bit between samples to let the IMU update, otherwise we just get the same reading repeatedly
            await asyncio.sleep_ms(10)
        success_count = len(samples_x)
        if success_count == 0:
            # No valid samples: leave calibration disabled and report failure.
            self._accel_calibrated = False
            if self._logging:
                print("[MC-DIAG] calibrate: failed, no valid accelerometer samples")
            return
        self._accel_bias_x = total_x / success_count
        self._accel_bias_y = total_y / success_count
        self._accel_filtered = 0.0
        self._accel_calibrated = True
        if self._logging:
            print("[MC-DIAG] calibrate: bias_x=%.4f  bias_y=%.4f" % (self._accel_bias_x, self._accel_bias_y))
            if samples_x:
                sx_str = ', '.join(['%.3f' % s for s in samples_x])
                sy_str = ', '.join(['%.3f' % s for s in samples_y])
                print("[MC-DIAG]   raw_x samples: [%s]" % sx_str)
                print("[MC-DIAG]   raw_y samples: [%s]" % sy_str)


    def _reset_distance(self):
        """Zero the accelerometer integrator state."""
        self.accel_mps2 = 0.0
        self.velocity_mps = 0.0
        self.distance_m = 0.0
        self._accel_filtered = 0.0


    def _send_output(self):
        """Push ``self.motor_output`` to the HexDrive."""
        if self._hexdrive is not None:
            output = self.motor_output
            if self._apply_motor_directions_callback is not None:
                mapped = self._apply_motor_directions_callback(output)
                if isinstance(mapped, tuple) and len(mapped) == len(output):
                    output = mapped
                elif self._logging:
                    print("[MC] apply_motor_directions_callback ignored invalid output")
            self._hexdrive.set_motors(output)

    
    @staticmethod
    def _slew(current, target, step):
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current


    def _ramp_toward(self, target, delta_ms):
        """Slew ``self.motor_output`` toward *target* respecting acceleration."""
        ticks = max(1, delta_ms // _TICK_MS)
        step = self.acceleration * ticks
        cur_l, cur_r = int(self.motor_output[0]), int(self.motor_output[1])
        tgt_l = max(-self.max_power, min(self.max_power, int(target[0])))
        tgt_r = max(-self.max_power, min(self.max_power, int(target[1])))
        self.motor_output = (
            self._slew(cur_l, tgt_l, step),
            self._slew(cur_r, tgt_r, step),
        )


    async def _ramp_stop(self):
        """Ramp motors to zero and hold briefly to let the robot settle."""
        last_time = time.ticks_ms()
        while self.motor_output != (0, 0):
            cur_time = time.ticks_ms()
            delta = time.ticks_diff(cur_time, last_time)
            last_time = cur_time
            self._ramp_toward((0, 0), max(delta, 1))
            self._send_output()
            await asyncio.sleep_ms(self._update_ms)
        self._send_output()
        # Zero velocity when we know the robot has stopped (drift reset)
        self.velocity_mps = 0.0
        await asyncio.sleep_ms(_DEFAULT_SETTLE_MS)

    async def _ramp_stop_with_accel(self):
        """Ramp motors to zero while continuing to read the accelerometer.

        This is used by ``_distance_drive`` so that the final distance
        includes displacement accumulated during the ramp-down phase.
        """
        last_time = time.ticks_ms()
        while self.motor_output != (0, 0):
            cur_time = time.ticks_ms()
            delta = time.ticks_diff(cur_time, last_time)
            last_time = cur_time
            self._ramp_toward((0, 0), max(delta, 1))
            self._send_output()
            self._read_accel(max(delta, 1))
            await asyncio.sleep_ms(self._update_ms)
        self._send_output()
        self.velocity_mps = 0.0
        await asyncio.sleep_ms(_DEFAULT_SETTLE_MS)


    async def _timed_drive(self, target, duration_ms):
        """Run motors at *target* for *duration_ms* with ramp-up and ramp-down."""
        self._busy = True
        elapsed = 0
        last_time = time.ticks_ms()

        try:
            self._power_on()

            # Phase 1 — ramp up and cruise
            while elapsed < duration_ms:
                cur_time = time.ticks_ms()
                delta = time.ticks_diff(cur_time, last_time)
                last_time = cur_time
                self._ramp_toward(target, delta)
                self._send_output()
                elapsed += delta
                await asyncio.sleep_ms(self._update_ms)

            # Phase 2 — ramp down
            await self._ramp_stop()
        finally:
            self._busy = False


    async def _distance_drive(self, target, distance_mm,
                              timeout_ms=None):
        """Drive until the accelerometer estimates *distance_mm* covered.

        Uses proportional deceleration: the motor speed scales linearly
        from full to zero over the last 30% of the target distance.
        This avoids the need to predict coast/overshoot and naturally
        adapts to different speeds, surfaces and battery levels.
        """
        timeout = timeout_ms if timeout_ms is not None else self._drive_timeout_ms

        # Apply calibration scale (percentage).
        scale_pct = 100
        if self._settings.get('accel_scale') is not None:
            scale_pct = max(1, int(self._settings['accel_scale'].v))
        target_m = abs(distance_mm) / 1000.0 * (100.0 / scale_pct)

        # If the requested distance is zero or negative after scaling,
        # there is nothing to do. Early-return to avoid division by zero
        # when computing proportional deceleration.
        if target_m <= 0:
            return

        # Deceleration zone: start slowing at this fraction of the target
        DECEL_START = 0.70  # begin slowing at 70% of target
        decel_start_m = target_m * DECEL_START
        decel_range_m = target_m - decel_start_m  # distance over which we ramp to 0
        # Minimum motor speed during deceleration (below this we just stop)
        min_frac = 0.15

        self._busy = True
        # wait a moment to let any previous motion settle, then calibrate the accelerometer bias
        await asyncio.sleep_ms(250)
        self._calibrate_accel()
        self._reset_distance()
        elapsed = 0
        last_time = time.ticks_ms()
        _diag_tick = 0

        # --- DIAGNOSTICS: start ---
        if self._logging:
            target_mm = target_m * 1000
            print("[MC-DIAG] === distance_drive START ===")
            print("[MC-DIAG]   requested   = %.1f mm" % distance_mm)
            print("[MC-DIAG]   accel_scale = %d%%" % scale_pct)
            print("[MC-DIAG]   target_m    = %.4f m  (%.1f mm)" % (target_m, target_mm))
            print("[MC-DIAG]   decel_start = %.1f mm  decel_range = %.1f mm" % (decel_start_m * 1000, decel_range_m * 1000))
            print("[MC-DIAG]   bias_x=%.4f  bias_y=%.4f" % (self._accel_bias_x, self._accel_bias_y))
            print("[MC-DIAG]   lpf_alpha=%s  deadband=%s" % (self._accel_lpf_alpha, self._accel_deadband))
            print("[MC-DIAG]   motor_target=%s  timeout=%d ms" % (str(target), timeout))
            theta_deg = 0.0
            if self._front_face_setting is not None:
                theta_deg = -(int(self._front_face_setting.v) * 30)
            print("[MC-DIAG]   front_face_angle=%d deg" % theta_deg)

        try:
            self._power_on()

            while elapsed < timeout:
                cur_time = time.ticks_ms()
                delta = time.ticks_diff(cur_time, last_time)
                last_time = cur_time

                # Proportional speed: full speed up to decel_start,
                # then linearly taper to min_frac, then stop.
                if self.distance_m >= target_m:
                    # Target reached — stop immediately
                    break
                elif self.distance_m > decel_start_m:
                    # In deceleration zone — scale speed linearly
                    remaining = max(0, target_m - self.distance_m)
                    frac = remaining / decel_range_m
                    frac = max(min_frac, min(1.0, frac))
                    scaled = (int(target[0] * frac), int(target[1] * frac))
                    self._ramp_toward(scaled, delta)
                else:
                    # Full speed
                    self._ramp_toward(target, delta)

                self._send_output()
                self._read_accel(delta)
                elapsed += delta

                # --- DIAGNOSTICS: periodic (every ~100 ms) ---
                _diag_tick += delta
                if _diag_tick >= 100:
                    _diag_tick = 0
                    d_mm = self.distance_m * 1000
                    spd_pct = 100
                    if self.distance_m > decel_start_m and decel_range_m > 0:
                        remaining = max(0, target_m - self.distance_m)
                        spd_pct = int(max(min_frac, min(1.0, remaining / decel_range_m)) * 100)
                    if self._logging:
                        print("[MC-DIAG]  t=%5dms a=%+.4f m/s2 v=%+.5f m/s d=%.2f/%.1f mm spd=%d%% mot=%s" % (
                            elapsed, self.accel_mps2, self.velocity_mps, d_mm, target_mm, spd_pct, str(self.motor_output)))

                await asyncio.sleep_ms(self._update_ms)
            else:
                d_mm = self.distance_m * 1000
                if self._logging:
                    print("[MC-DIAG]  TIMEOUT after %d ms  dist=%.2f mm" % (elapsed, d_mm))

            d_mm = self.distance_m * 1000
            if self._logging:
                print("[MC-DIAG]  STOP at dist=%.2f mm (elapsed=%d ms)" % (d_mm, elapsed))

            # Ramp down remaining motor output while still reading accel
            ramp_start_dist = self.distance_m
            await self._ramp_stop_with_accel()
            ramp_dist = self.distance_m - ramp_start_dist

            # --- DIAGNOSTICS: end ---
            if self._logging:
                final_mm = self.distance_m * 1000
                ramp_mm = ramp_dist * 1000
                over_mm = (self.distance_m - target_m) * 1000
                print("[MC-DIAG] === distance_drive END ===")
                print("[MC-DIAG]   final_distance = %.2f mm  (target was %.1f mm, scaled target %.1f mm)" % (final_mm, distance_mm, target_mm))
                print("[MC-DIAG]   ramp_down_dist = %.2f mm" % ramp_mm)
                print("[MC-DIAG]   final_velocity = %.5f m/s" % self.velocity_mps)
                print("[MC-DIAG]   elapsed        = %d ms" % elapsed)
                print("[MC-DIAG]   overshoot      = %+.2f mm" % over_mm)
        finally:
            self._busy = False

    def _estimate_ramp_overshoot(self):
        """Estimate distance the robot will coast while motors ramp to zero.

        Note: this is kept for potential future use but the main drive
        loop now uses proportional deceleration instead of prediction.
        """
        v = abs(self.velocity_mps)
        if v < 0.001:
            return 0.0
        step_per_iter = self.acceleration * max(1, int(self._avg_loop_ms) // _TICK_MS)
        ramp_iters = max(1, abs(int(self.motor_output[0])) // max(1, step_per_iter))
        ramp_secs = ramp_iters * self._avg_loop_ms / 1000.0
        return v * ramp_secs * 0.75

    def _read_gyro(self, delta_ms):
        """Read the gyro and accumulate into ``integrated_deg``."""
        if _imu is None:
            return
        try:
            raw = _imu.gyro_read()
            self.gyro_dps = float(raw[self._gyro_axis])
        except Exception:       # pylint: disable=broad-exception-caught
            self.gyro_dps = 0.0
            return
        magnitude = abs(self.gyro_dps)
        if magnitude > self._gyro_deadband:
            self.integrated_deg += magnitude * (delta_ms / 1000.0)

    def _read_accel(self, delta_ms):
        """Read the accelerometer, filter, and double-integrate into distance.

        The raw X and Y axes are rotated by the ``front_face`` angle so
        that the resulting value represents acceleration along the
        robot's actual forward direction — regardless of how the motors
        are mounted on the badge.

        The projected value is bias-corrected (see ``_calibrate_accel``),
        passed through a simple exponential low-pass filter to smooth out
        motor vibration, then integrated twice:

            acceleration → velocity → distance

        Only the absolute magnitude of displacement is accumulated so the
        result works for both forward and backward drives.
        """
        if _imu is None:
            return
        try:
            raw = _imu.acc_read()
            raw_x = float(raw[0])
            raw_y = float(raw[1])
        except Exception:       # pylint: disable=broad-exception-caught
            return

        dt = delta_ms / 1000.0  # seconds

        # Remove per-axis bias (gravity component + sensor offset at rest)
        cx = raw_x - self._accel_bias_x
        cy = raw_y - self._accel_bias_y

        # Project onto the robot's forward direction using front_face angle.
        # This is the dot product of (cx, cy) with the forward unit vector
        # (cos(theta), sin(theta)), NOT a coordinate rotation.
        theta = self._get_front_angle_rad()
        fwd_accel = cx * cos(theta) + cy * sin(theta)

        # Exponential low-pass filter to suppress vibration
        alpha = self._accel_lpf_alpha
        self._accel_filtered = alpha * fwd_accel + (1.0 - alpha) * self._accel_filtered

        # Deadband — treat very small readings as zero (noise)
        if abs(self._accel_filtered) < self._accel_deadband:
            self.accel_mps2 = 0.0
        else:
            self.accel_mps2 = self._accel_filtered

        # Integrate acceleration → velocity (m/s)
        self.velocity_mps += self.accel_mps2 * dt

        # Integrate velocity → distance (m), accumulate absolute displacement
        self.distance_m += abs(self.velocity_mps) * dt
