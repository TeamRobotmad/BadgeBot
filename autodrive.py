"""Autonomous drive mode for BadgeBot - obstacle avoidance via ToF spin-scan."""
import asyncio
from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels

try:
    import imu as _imu
except ImportError:
    _imu = None

from .constants import (
    _TICK_MS, STATE_MENU,
    _AUTO_SUB_DRIVE, _AUTO_SUB_SCAN, _AUTO_SUB_TURN, _AUTO_SUB_TURN_BACK, _AUTO_SUB_REVERSE,
    _AUTO_SENSOR_READ_MS, _AUTO_MIN_FWD_MS,
    _AUTO_CRUISE_MIN_PWM, _AUTO_SCAN_FORWARD_ONLY,
    _AUTO_CLEAR_DIST_MM,
    _AUTO_SCAN_TIMEOUT_MS,
    _AUTO_TURN_BACK_TOLERANCE_MM, _AUTO_TURN_BACK_TIMEOUT_MS, _AUTO_TURN_BACK_SPEED_FRAC,
    _AUTO_GYRO_AXIS, _AUTO_GYRO_DEADBAND_DPS,
    _AUTO_BACKUP_MS, _AUTO_BACKUP_MM, _AUTO_BACKUP_SPEED_FRAC,
)


class AutoDrive:
    """Encapsulates autonomous driving state and logic.

    Receives a reference to the main BadgeBotApp so it can access shared
    resources (hexdrive_app, sensor_mgr, settings) without duplicating them.
    """

    def __init__(self, app):
        self._app = app
        self._mc = app.motor_controller   # may be None
        self._mc_task = None              # async task for MC turn/drive
        self.sub_state: int = _AUTO_SUB_DRIVE
        self.distance = None            # latest ToF reading in mm (int or None)
        self.lux = None                 # latest lux reading (float or None)
        self.sensor_timer: int = 0      # ms since last sensor read
        self.scan_data: list = []       # list of (angle_deg, dist_mm) pairs
        self.scan_slot: int = 0         # sample count (for display)
        self.scan_timer: int = 0        # ms elapsed in current scan (safety timeout)
        self.turn_ms: int = 0           # total ms for this turn
        self.turn_timer: int = 0        # ms turned so far
        self.turn_dir: int = 1          # +1 = right, -1 = left
        self.best_dist: int = 0         # target distance for sensor-feedback back-sweep
        self.turn_back_timer: int = 0   # ms elapsed in back-sweep phase
        self.reverse_timer: int = 0     # ms elapsed while backing away
        self.forward_hold_ms: int = _AUTO_MIN_FWD_MS
        # IMU gyro tracking
        self.gyro_dps: float = 0.0      # latest yaw rate reading (degrees/s, display only)
        self.imu_deg: float = 0.0       # degrees accumulated since phase entry
        self.target_deg: float = 0.0    # degrees to turn to reach best heading
        self.target_output: tuple = (0, 0)   # desired motor output
        self.motor_output: tuple = (0, 0)    # ramped output sent to motors
        self.status: str = "Starting..."

    # ------------------------------------------------------------------
    # Public interface called by BadgeBotApp
    # ------------------------------------------------------------------

    def update(self, delta: int):
        """Main update tick, called from BadgeBotApp.update when in STATE_AUTO."""
        # CANCEL always exits cleanly
        if self._app.button_states.get(BUTTON_TYPES["CANCEL"]):
            self._app.button_states.clear()
            self.stop()
            self._app._update_period = 50
            self._app.current_state = STATE_MENU
            return

        # Sub-state logic runs first so that distance cleared on a state
        # transition (e.g. scan->drive) is not immediately overwritten by a fresh
        # sensor read in the same tick, which would cause an instant re-scan.
        if self.sub_state == _AUTO_SUB_DRIVE:
            self._update_drive(delta)
        elif self.sub_state == _AUTO_SUB_REVERSE:
            self._update_reverse(delta)
        elif self.sub_state == _AUTO_SUB_SCAN:
            self._update_scan(delta)
        elif self.sub_state == _AUTO_SUB_TURN:
            self._update_turn(delta)
        elif self.sub_state == _AUTO_SUB_TURN_BACK:
            self._update_turn_back(delta)

        self._apply_output_ramp(delta)
        self._integrate_gyro(delta)

        # Sensor read runs after sub-state so the new value is used next tick
        self.sensor_timer += delta
        if self.sensor_timer >= _AUTO_SENSOR_READ_MS:
            self.sensor_timer = 0
            self._read_sensor()
            # Record (angle, dist) snapshot on every sensor tick during a scan
            if self.sub_state == _AUTO_SUB_SCAN:
                self._scan_record_sample()

        self._app._refresh = True

    def draw(self, ctx):
        """Draw the auto-drive UI overlay."""
        sub_labels = {_AUTO_SUB_DRIVE:      "Driving",
                      _AUTO_SUB_REVERSE:    "Reversing",
                      _AUTO_SUB_SCAN:       "Scanning",
                      _AUTO_SUB_TURN:       "Turning",
                      _AUTO_SUB_TURN_BACK:  "Returning"}
        sub_label = sub_labels.get(self.sub_state, "?")
        d_str   = f"{self.distance}mm" if self.distance is not None else "---"
        lx_str  = f"{self.lux:.0f}lx"  if self.lux      is not None else "---"
        g_str   = f"{self.gyro_dps:+.1f}dps"
        deg_str = f"{self.imu_deg:.1f}deg/{self.target_deg:.1f}deg" if self.target_deg else f"{self.imu_deg:.1f}deg"
        lines   = ["Auto Drive", sub_label, f"Dist:{d_str} Lux:{lx_str}",
                   f"Gyro:{g_str} {deg_str}", self.status]
        colours = [(1,1,1), (0,1,1), (1,1,0), (0,0.9,0.4), (0.8,0.8,0.8)]
        self._app.draw_message(ctx, lines, colours, label_font_size)

        # Polar bar chart of scan (angle -> distance)
        if self.scan_data:
            max_dist = max(d for _, d in self.scan_data)
            max_dist = max(max_dist, 1)
            ctx.save()
            ctx.translate(-90, 55)
            for angle, d in self.scan_data:
                x = int(angle / 360.0 * 180)   # 0..180 px across the display
                h = int(20 * d / max_dist)
                ctx.rgb(0, 0.6, 0.6).rectangle(x, -h, 2, h).fill()
            # Mark the best-heading angle with a white tick
            if self.target_deg and (self.sub_state in (_AUTO_SUB_TURN, _AUTO_SUB_TURN_BACK)):
                # Reconstruct the original best_angle from turn_dir and target_deg
                best_angle = self.target_deg if self.turn_dir > 0 else 360.0 - self.target_deg
                bx = int(best_angle / 360.0 * 180)
                ctx.rgb(1, 1, 1).rectangle(bx, -22, 2, 22).fill()
            ctx.restore()

        button_labels(ctx, cancel_label="Stop")

    def background_update(self, delta: int):
        """Feed motors from background loop so the HexDrive watchdog stays happy."""
        # When a MotorController task is running it drives the motors itself
        if self._mc_task is not None and not self._mc_task.done():
            return
        if self._app.hexdrive_app is not None:
            self._app.hexdrive_app.set_motors(
                self._app._apply_fwd_dir(self.motor_output))

    def stop(self):
        """Clean shutdown - zero motors and cut power."""
        if self._mc_task is not None:
            self._mc_task.cancel()
            self._mc_task = None
        if self._mc is not None:
            self._mc.stop()
        self.motor_output = (0, 0)
        self.target_output = (0, 0)
        if self._app.hexdrive_app is not None:
            self._app.hexdrive_app.set_motors((0, 0))
            self._app.hexdrive_app.set_power(False)
        self.status = ""

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _integrate_gyro(self, delta: int):
        """Read gyro yaw axis and accumulate into imu_deg for the current phase."""
        if _imu is None:
            return
        try:
            raw = _imu.gyro_read()
            self.gyro_dps = float(raw[_AUTO_GYRO_AXIS])
        except Exception:
            self.gyro_dps = 0.0
            return
        magnitude = abs(self.gyro_dps)
        if magnitude > _AUTO_GYRO_DEADBAND_DPS:
            self.imu_deg += magnitude * (delta / 1000.0)

    def _read_sensor(self):
        """Read the current sensor and store numeric distance and lux."""
        if self._app._sensor_mgr is None or not self._app._sensor_mgr.is_open:
            return
        try:
            data = self._app._sensor_mgr.read_current()
            r = str(data.get("range_mm", ""))
            self.distance = int(r.replace("mm", "")) if "mm" in r else None
            lx = str(data.get("lux", ""))
            self.lux = float(lx.replace("lx", "")) if "lx" in lx else None
        except Exception:
            self.distance = None
            self.lux = None
        if self._app._settings['logging'].v:
            print(f"A:sens dist={self.distance} lux={self.lux}")

    def _stop_motors(self):
        self.target_output = (0, 0)
        self.motor_output = (0, 0)

    @staticmethod
    def _slew(current: int, target: int, step: int) -> int:
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def _apply_output_ramp(self, delta: int):
        accel = max(1, int(self._app._settings['acceleration'].v))
        ticks = max(1, delta // _TICK_MS)
        step = accel * ticks
        max_power = int(self._app._settings['max_power'].v)

        target_l = max(-max_power, min(max_power, int(self.target_output[0])))
        target_r = max(-max_power, min(max_power, int(self.target_output[1])))
        cur_l = int(self.motor_output[0])
        cur_r = int(self.motor_output[1])

        self.motor_output = (
            self._slew(cur_l, target_l, step),
            self._slew(cur_r, target_r, step),
        )

    def _enter_drive(self):
        """Enter forward-drive mode with scan holdoff."""
        self.sub_state = _AUTO_SUB_DRIVE
        self.imu_deg = 0.0
        self.target_deg = 0.0
        self.distance = None
        self.forward_hold_ms = _AUTO_MIN_FWD_MS
        speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
        self.target_output = (speed, speed)
        self.status = "Fwd"

    def _update_drive(self, delta: int):
        """Drive forward; trigger a scan if an obstacle is detected within threshold.

        A None distance means the sensor timed out (nothing in range) - treated
        as clear.  Only a valid reading *below* auto_obstacle triggers a scan.
        """
        if self.forward_hold_ms > 0:
            self.forward_hold_ms = max(0, self.forward_hold_ms - delta)

        obstacle_detected = (self.distance is not None and
                             self.distance < self._app._settings['auto_obstacle'].v)
        if obstacle_detected and self.forward_hold_ms == 0:
            if self.distance is not None and self.distance < _AUTO_BACKUP_MM:
                self._enter_reverse()
            else:
                self._enter_scan()
        else:
            speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
            self.target_output = (speed, speed)
            d = f"{self.distance}mm" if self.distance is not None else "---"
            self.status = f"Fwd {d}"

    def _enter_reverse(self):
        """Back away briefly when the obstacle is very close before scanning."""
        self.sub_state = _AUTO_SUB_REVERSE
        self.reverse_timer = 0
        if self._app._settings['logging'].v:
            print("A:Obstacle " + str(self.distance) + "mm - reversing before scan")
        # Use MotorController distance-based reverse if available
        if self._mc is not None:
            backup_mm = _AUTO_BACKUP_MM - (self.distance if self.distance is not None else 0)
            backup_mm = max(30, backup_mm)  # at least 30mm
            self._mc_task = asyncio.get_event_loop().create_task(
                self._mc.backward_mm(backup_mm, speed_frac=_AUTO_BACKUP_SPEED_FRAC))
            self.status = "Reverse %dmm (MC)" % backup_mm
            if self._app._settings['logging'].v:
                print("A:MC backward_mm(%d)" % backup_mm)
        else:
            speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
            back_speed = max(1, int(speed * _AUTO_BACKUP_SPEED_FRAC))
            self.target_output = (-back_speed, -back_speed)
            self.status = "Reverse " + str(self.distance) + "mm"

    def _update_reverse(self, delta: int):
        """Reverse for a fixed time (or until MC task completes) then start the scan."""
        if self._mc_task is not None:
            # MC distance-based reverse in progress
            if self._mc_task.done():
                self._mc_task = None
                self.status = "Reverse done"
                self._enter_scan()
            else:
                dist_m = self._mc.distance_m if self._mc else 0
                self.status = "Reverse %.0fmm (MC)" % (dist_m * 1000)
            return
        # Legacy time-based fallback
        speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
        back_speed = max(1, int(speed * _AUTO_BACKUP_SPEED_FRAC))
        self.target_output = (-back_speed, -back_speed)
        self.reverse_timer += delta
        self.status = "Reverse " + str(self.reverse_timer) + "ms"
        if self.reverse_timer >= _AUTO_BACKUP_MS:
            self._enter_scan()

    def _enter_scan(self):
        """Transition into the 360deg scan, spinning clockwise."""
        self.scan_data = []
        self.scan_slot = 0
        self.scan_timer = 0
        self.imu_deg = 0.0
        self.sub_state = _AUTO_SUB_SCAN
        speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
        if _AUTO_SCAN_FORWARD_ONLY:
            self.target_output = (speed, 0)
        else:
            self.target_output = (speed, -speed)
        self.status = "Scan 0deg"
        if self._app._settings['logging'].v:
            print("A:Starting scan dist=" + str(self.distance) + "mm")

    def _scan_record_sample(self):
        """Record the current ToF reading paired with the current gyro angle."""
        dist = self.distance if self.distance is not None else _AUTO_CLEAR_DIST_MM
        self.scan_data.append((self.imu_deg, dist))
        self.scan_slot = len(self.scan_data)

    def _update_scan(self, delta: int):
        """Spin clockwise while the gyro integrates to 360deg, recording (angle, dist) per
        sensor tick.  At completion the exact measured angle of the clearest reading is
        used directly as the turn target - no slot arithmetic needed.
        """
        speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)

        if _AUTO_SCAN_FORWARD_ONLY:
            self.target_output = (speed, 0)
        else:
            self.target_output = (speed, -speed)

        self.scan_timer += delta
        self.status = f"Scan {self.imu_deg:.0f}deg ({self.scan_slot}pts)"

        # Terminate when gyro hits 360deg or safety timeout fires
        gyro_full = (self.imu_deg >= 360.0)
        timed_out = (self.scan_timer >= _AUTO_SCAN_TIMEOUT_MS)
        if not (gyro_full or timed_out):
            return

        if not self.scan_data:
            if self._app._settings['logging'].v:
                print("A:Scan aborted - no samples collected")
            self._enter_drive()
            return

        # Find the heading (angle) with the greatest clear distance
        best_idx = max(range(len(self.scan_data)), key=lambda i: self.scan_data[i][1])
        best_angle, best_dist = self.scan_data[best_idx]
        self.best_dist = best_dist
        scan_total_ms = self.scan_timer

        if self._app._settings['logging'].v:
            reason = "360deg" if gyro_full else "timeout"
            n = len(self.scan_data)
            print("A:Scan done (" + reason + ") imu=" + str(round(self.imu_deg, 1)) + "deg in " + str(scan_total_ms) + "ms, best=" + str(round(best_angle, 1)) + "deg=" + str(best_dist) + "mm samples=" + str(n))

        # Shortest path to the best heading (robot is back at 0deg after full spin)
        if best_angle <= 180.0:
            self.turn_dir = 1           # clockwise
            self.target_deg = best_angle
        else:
            self.turn_dir = -1          # anti-clockwise
            self.target_deg = 360.0 - best_angle

        # Time-based fallback proportional to actual measured scan rate
        scanned_deg = self.imu_deg if self.imu_deg > 0 else 360.0
        self.turn_ms = int(scan_total_ms * self.target_deg / scanned_deg)
        self.turn_timer = 0
        self.imu_deg = 0.0

        if self.target_deg < 2.0:
            self._enter_drive()
            return

        self.sub_state = _AUTO_SUB_TURN
        lbl = "right" if self.turn_dir > 0 else "left"

        # Use MotorController gyro turn if available
        if self._mc is not None:
            turn_degrees = self.target_deg * self.turn_dir  # signed
            self._mc_task = asyncio.get_event_loop().create_task(
                self._mc.turn(turn_degrees))
            self.status = "Turn %s %.0fdeg (MC)" % (lbl, self.target_deg)
            if self._app._settings['logging'].v:
                print("A:MC turn(%.1f) for best_angle=%.1f best_dist=%d"
                      % (turn_degrees, best_angle, best_dist))
        else:
            if _AUTO_SCAN_FORWARD_ONLY:
                self.target_output = (speed, 0) if self.turn_dir > 0 else (0, speed)
            else:
                self.target_output = (speed * self.turn_dir, -speed * self.turn_dir)
            self.status = "Turn %s %.0fdeg=%dmm" % (lbl, best_angle, best_dist)

    def _update_turn(self, delta: int):
        """Rotate toward the best heading, then hand off to sensor-feedback back-sweep.

        When a MotorController task is active, waits for it to complete
        then goes directly to DRIVE (gyro turn is accurate, no back-sweep
        needed).  Otherwise falls back to time/gyro integration and uses
        the sensor-feedback TURN_BACK phase.
        """
        if self._mc_task is not None:
            # MC gyro turn in progress
            if self._mc_task.done():
                if self._app._settings['logging'].v:
                    actual = self._mc.integrated_deg if self._mc else 0
                    print("A:MC turn done  actual=%.1fdeg  target=%.1fdeg"
                          % (actual, self.target_deg))
                self._mc_task = None
                self._enter_drive()
            else:
                turned = self._mc.integrated_deg if self._mc else 0
                self.status = "Turn %.0f/%.0fdeg (MC)" % (turned, self.target_deg)
            return
        # Legacy time/gyro-based turn
        speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
        if _AUTO_SCAN_FORWARD_ONLY:
            self.target_output = (speed, 0) if self.turn_dir > 0 else (0, speed)
        else:
            self.target_output = (speed  * self.turn_dir,
                                  -speed * self.turn_dir)
        self.turn_timer += delta
        gyro_done = (self.target_deg > 0 and self.imu_deg >= self.target_deg)
        time_done = (self.turn_timer >= self.turn_ms)
        if gyro_done or time_done:
            if self._app._settings['logging'].v:
                reason = "gyro" if gyro_done else "timeout"
                print("A:Turn done (%s) imu=%.1fdeg target=%.1fdeg t=%dms"
                      % (reason, self.imu_deg, self.target_deg, self.turn_timer))
            self._enter_turn_back()

    def _enter_turn_back(self):
        """Transition to sensor-feedback reverse sweep after the time-based turn."""
        self.sub_state = _AUTO_SUB_TURN_BACK
        self.turn_back_timer = 0
        self.imu_deg = 0.0   # reset integrator for the back-sweep
        lbl = "left" if self.turn_dir > 0 else "right"  # reverse of initial turn dir
        self.status = f"Return {lbl} tgt={self.best_dist}mm"
        if self._app._settings['logging'].v:
            print(f"A:TurnBack start - target={self.best_dist}mm dir={'left' if self.turn_dir > 0 else 'right'} deg_target={self.target_deg:.1f}deg")

    def _update_turn_back(self, delta: int):
        """Sweep back in the opposite direction until the ToF reading matches best_dist.

        This closes the loop on heading: rather than relying solely on timing the
        robot backs off until the sensor sees the same distance recorded at the best
        scan slot, confirming it is actually pointed in the right direction.
        A timeout fallback drives on anyway if the reading never converges.
        """
        speed = max(self._app._settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)
        back_speed = max(1, int(speed * _AUTO_TURN_BACK_SPEED_FRAC))
        back_dir = -self.turn_dir  # opposite of the initial time-based turn

        if _AUTO_SCAN_FORWARD_ONLY:
            self.target_output = (back_speed, 0) if back_dir > 0 else (0, back_speed)
        else:
            self.target_output = (back_speed * back_dir, -back_speed * back_dir)

        self.turn_back_timer += delta

        # Sensor-feedback: check whether we have reached the target heading
        if self.best_dist == _AUTO_CLEAR_DIST_MM:
            # Target was "clear" (no obstacle in range) - match when sensor is None
            tof_matched = self.distance is None
        elif self.distance is not None:
            tof_matched = abs(self.distance - self.best_dist) <= _AUTO_TURN_BACK_TOLERANCE_MM
        else:
            tof_matched = False

        # IMU-feedback: stop when we've swept back at least as far as we turned
        gyro_matched = (self.target_deg > 0 and self.imu_deg >= self.target_deg)

        if tof_matched or gyro_matched or self.turn_back_timer >= _AUTO_TURN_BACK_TIMEOUT_MS:
            if self._app._settings['logging'].v:
                reasons = []
                if tof_matched:   reasons.append(f"tof={self.distance}mm")
                if gyro_matched:  reasons.append(f"gyro={self.imu_deg:.1f}deg")
                if not reasons:   reasons.append(f"timeout")
                print(f"A:TurnBack done ({', '.join(reasons)}) target={self.best_dist}mm deg={self.target_deg:.1f}deg")
            self._enter_drive()
