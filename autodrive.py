"""Autonomous drive mode for BadgeBot.

Behavior:
- Drive forward until a close obstacle is seen by the range sensor.
- Back up briefly if very close.
- Spin 360 degrees while sampling range and find the clearest heading.
- Turn to that heading using gyro integration.
- Repeat.
"""

from events.input import BUTTON_TYPES
from app_components.tokens import label_font_size, button_labels
from app_components.notification import Notification

try:
    import imu as _imu
except ImportError:
    _imu = None

from .app import MOTOR_ENABLE_USER_STATE


# Sub-state constants
_AUTO_SUB_DRIVE = 0
_AUTO_SUB_REVERSE = 1
_AUTO_SUB_SCAN = 2
_AUTO_SUB_TURN = 3

# Behavior constants
_TICK_MS = 10
_AUTO_SENSOR_READ_MS = 100
_AUTO_MIN_FWD_MS = 400
_AUTO_CRUISE_MIN_PWM = 36000
_AUTO_CLEAR_DIST_MM = 800
_AUTO_SCAN_TARGET_DEG = 360.0
_AUTO_SCAN_TIMEOUT_MS = 14000
_AUTO_TURN_STOP_MARGIN_DEG = 5.0
_AUTO_TURN_SPEED_FRAC = 0.65
_AUTO_SCAN_SPEED_FRAC = 0.25
_AUTO_TURN_TIMEOUT_MIN_MS = 1200
_AUTO_TURN_TIMEOUT_MAX_MS = 7000
_AUTO_SCAN_EXCLUDE_DEG = 45.0
_AUTO_GYRO_AXIS = 2
_AUTO_GYRO_DEADBAND_DPS = 3.0
_AUTO_BACKUP_MS = 600
_AUTO_BACKUP_NEAR_MM = 200
_AUTO_BACKUP_SPEED_FRAC = 0.7
_AUTO_OBSTACLE_CONFIRM_SAMPLES = 2
_AUTO_SCAN_PAUSE_MS = 200
_AUTO_PLOT_INTERVAL_MS = 100
_AUTO_LOG_INTERVAL_MS = 500

# Default settings
_AUTO_DRIVE_SPEED = 56000    # ~43% max power default for auto driving
_AUTO_OBSTACLE_MM = 250      # mm — trigger scan below this distance


# ---- Settings initialisation -----------------------------------------------

def init_settings(s, MySetting: type):
    """Register auto-drive-specific settings in the shared settings dict."""
    s['auto_speed'] = MySetting(
        s, _AUTO_DRIVE_SPEED, 1000, 65535,
        group=MySetting.GROUP_AUTO_DRIVE, order=10, title="Auto speed",
        description="Motor power used while auto-driving")
    s['auto_obstacle'] = MySetting(
        s, _AUTO_OBSTACLE_MM, 20, 500,
        group=MySetting.GROUP_AUTO_DRIVE, order=20, title="Obstacle range",
        description="Distance (mm) below which auto-drive starts avoidance")
    s['auto_scan_speed'] = MySetting(
        s, max(1000, int(_AUTO_DRIVE_SPEED * _AUTO_SCAN_SPEED_FRAC)), 1000, 65535,
        group=MySetting.GROUP_AUTO_DRIVE, order=30, title="Scan speed",
        description="Motor power used while spinning to scan for a clear heading")

# ---- Auto Drive manager ----------------------------------------------------

class AutoDriveMgr:
    """Manages autonomous obstacle-avoidance driving."""

    def __init__(self, app, logging: bool = False):
        self._app = app
        self._logging: bool = logging
        self._sensor_mgr = app.sensor_test_mgr
        self._range_hexdrive = None
        self._active = False

        self.sub_state: int = _AUTO_SUB_DRIVE
        self.distance: int | None = None
        self.scan_data: list[tuple[float, int]] = []

        self.forward_hold_ms: int = _AUTO_MIN_FWD_MS
        self.reverse_timer: int = 0
        self.scan_timer: int = 0
        self.scan_progress_deg: float = 0.0
        self.turn_timer: int = 0
        self.turn_progress_deg: float = 0.0
        self.turn_timeout_ms: int = _AUTO_TURN_TIMEOUT_MIN_MS
        self.turn_dir: int = 1
        self.target_deg: float = 0.0
        self.best_angle_deg: float = 0.0
        self.best_dist_mm: int = 0
        self.scan_trigger_deg: float = 0.0
        self.scan_start_yaw_deg: float = 0.0
        self.yaw_deg: float = 0.0
        self.scan_pause_ms: int = 0
        self.obstacle_hits: int = 0

        self.gyro_dps: float = 0.0
        self.target_output: tuple[int, int] = (0, 0)
        self.motor_output: tuple[int, int] = (0, 0)
        self.status: str = ""

        self._plot_ms: int = 0
        self._range_age_ms: int = 0
        self._range_log_ms: int = 0
        self._range_samples: int = 0

        if self._logging:
            print("A:AutoDriveMgr initialised")

    @property
    def logging(self) -> bool:
        return self._logging

    @logging.setter
    def logging(self, value: bool):
        self._logging = value

    def start(self) -> bool:
        """Enter the auto-drive flow from the main menu."""
        app = self._app
        self._sensor_mgr = app.sensor_test_mgr

        if not app.enable_motors(True, MOTOR_ENABLE_USER_STATE):
            if self._logging:
                print("A:Failed to enable motors")
            return False

        if not self._enable_sensors():
            app.enable_motors(False, MOTOR_ENABLE_USER_STATE)
            app.notification = Notification("Range Sensor not available")
            return False

        app.set_menu(None)
        app.button_states.clear()
        app.refresh = True

        self._active = True
        self.sub_state = _AUTO_SUB_DRIVE
        self.distance = None
        self.scan_data = []
        self.forward_hold_ms = _AUTO_MIN_FWD_MS
        self.reverse_timer = 0
        self.scan_timer = 0
        self.scan_progress_deg = 0.0
        self.turn_timer = 0
        self.turn_progress_deg = 0.0
        self.turn_timeout_ms = _AUTO_TURN_TIMEOUT_MIN_MS
        self.turn_dir = 1
        self.target_deg = 0.0
        self.best_angle_deg = 0.0
        self.best_dist_mm = 0
        self.scan_trigger_deg = 0.0
        self.scan_start_yaw_deg = 0.0
        self.yaw_deg = 0.0
        self.scan_pause_ms = 0
        self.obstacle_hits = 0
        self.gyro_dps = 0.0
        self._app.refresh = True
        self.target_output = (0, 0)
        self.motor_output = (0, 0)
        self.status = "Starting"

        self._plot_ms = 0
        self._range_age_ms = 0
        self._range_log_ms = 0
        self._range_samples = 0

        self._enter_drive()
        return True

    def update(self, delta: int):
        """Main update tick, called from BadgeBotApp.update."""
        if not self._active:
            return

        if self._app.button_states.get(BUTTON_TYPES["CANCEL"]):
            self._app.button_states.clear()
            self.stop()
            self._app.return_to_menu()
            return

        gyro_delta = self._read_gyro_delta(delta)

        if self.sub_state == _AUTO_SUB_DRIVE:
            self._update_drive(delta)
        elif self.sub_state == _AUTO_SUB_REVERSE:
            self._update_reverse(delta)
        elif self.sub_state == _AUTO_SUB_SCAN:
            self._update_scan(delta, gyro_delta)
        elif self.sub_state == _AUTO_SUB_TURN:
            self._update_turn(delta, gyro_delta)

        self._apply_output_ramp(delta)

    def draw(self, ctx):
        """Draw the auto-drive UI overlay."""
        sub_labels = {
            _AUTO_SUB_DRIVE: "Driving",
            _AUTO_SUB_REVERSE: "Reversing",
            _AUTO_SUB_SCAN: "Scanning",
            _AUTO_SUB_TURN: "Turning",
        }
        sub_label = sub_labels.get(self.sub_state, "?")
        dist_str = f"{self.distance}mm" if self.distance is not None else "---"

        deg_info = ""
        if self.sub_state == _AUTO_SUB_SCAN:
            deg_info = f"{self.scan_progress_deg:.0f}/{_AUTO_SCAN_TARGET_DEG:.0f}deg"
        elif self.sub_state == _AUTO_SUB_TURN:
            deg_info = f"{self.turn_progress_deg:.0f}/{self.target_deg:.0f}deg"

        status_line = f"{self.status} Age:{self._range_age_ms}ms N:{self._range_samples}"
        chosen_angle = f"Chosen: {self.best_angle_deg:.0f}deg" if self.best_angle_deg > 0.0 else ""

        if chosen_angle:
            lines = [
                "Auto Drive",
                sub_label,
                f"Dist: {dist_str}",
                f"Gyro: {self.gyro_dps:+.1f}dps {deg_info}",
                chosen_angle,
                status_line,
            ]
        else:
            lines = [
                "Auto Drive",
                sub_label,
                f"Dist: {dist_str}",
                f"Gyro: {self.gyro_dps:+.1f}dps {deg_info}",
                status_line,
            ]
        colours = [(1, 1, 1), (0, 1, 1), (1, 1, 0), (0, 0.9, 0.4), (0.8, 0.8, 0.8)]
        self._app.draw_message(ctx, lines, colours, label_font_size)

        if self.scan_data:
            max_dist = max(1, max(d for _, d in self.scan_data))
            ctx.save()
            ctx.translate(-90, 55)
            for angle, dist_mm in self.scan_data:
                x = int((angle % 360.0) / 360.0 * 180)
                h = int(20 * dist_mm / max_dist)
                ctx.rgb(0, 0.6, 0.6).rectangle(x, -h, 2, h).fill()

            if self.best_angle_deg > 0.0:
                bx = int((self.best_angle_deg % 360.0) / 360.0 * 180)
                ctx.rgb(1, 1, 1).rectangle(bx, -22, 2, 22).fill()
            ctx.restore()

        button_labels(ctx, cancel_label="Stop")

    def background_update(self, delta: int) -> tuple[int, int] | None:
        """Poll range sensor and return motor outputs while active."""
        if self._active and self._sensor_mgr is not None and self._range_hexdrive is not None:
            self._range_age_ms += delta
            self._range_log_ms += delta

            # Defensive explicit read mirrors line follower behavior.
            range_sensor = getattr(self._range_hexdrive, "range_sensor", None)
            if range_sensor is not None:
                _ = range_sensor.read()

            new_sample, range_mm = self._sensor_mgr.read_range(self._range_hexdrive)
            if new_sample:
                if range_mm is None or range_mm <= 0:
                    self.distance = None
                else:
                    self.distance = range_mm

                self._range_age_ms = 0
                self._range_samples += 1

                if self._logging and self._range_log_ms >= _AUTO_LOG_INTERVAL_MS:
                    self._range_log_ms = 0
                    shown = self.distance if self.distance is not None else "---"
                    print(f"A:Range sample={shown}mm count={self._range_samples}")

                if self.sub_state == _AUTO_SUB_SCAN:
                    self._scan_record_sample()
            elif self._logging and self._range_log_ms >= _AUTO_LOG_INTERVAL_MS:
                self._range_log_ms = 0
                print(f"A:Waiting for range sample age={self._range_age_ms}ms")

        self._plot_ms += delta
        if self._plot_ms >= _AUTO_PLOT_INTERVAL_MS and self._app.bluetooth_mgr is not None:
            self._plot_ms = 0

            if self.sub_state == _AUTO_SUB_TURN:
                plot_angle = self.target_deg if self.target_deg > 0.0 else self.best_angle_deg
                plot_score = self.target_deg
            elif self.sub_state == _AUTO_SUB_SCAN:
                if self.best_angle_deg > 0.0:
                    plot_angle = self.best_angle_deg
                elif self.target_deg > 0.0:
                    plot_angle = self.target_deg
                else:
                    plot_angle = self.scan_progress_deg

                current_dist = self.distance if self.distance is not None else _AUTO_CLEAR_DIST_MM
                if current_dist > _AUTO_CLEAR_DIST_MM:
                    current_dist = _AUTO_CLEAR_DIST_MM
                if self.scan_data:
                    last_angle, last_dist = self.scan_data[-1]
                    prev_dist = self.scan_data[-2][1] if len(self.scan_data) > 1 else last_dist
                    next_dist = self.scan_data[0][1] if len(self.scan_data) == 1 else current_dist
                    plot_score = current_dist + (prev_dist * 0.5) + (next_dist * 0.5)
                else:
                    plot_score = float(current_dist)
            else:
                plot_angle = 0.0
                plot_score = 0.0

            plot_distance = self.distance if self.distance is not None else 0
            if plot_distance > 0:
                plot_distance = max(0, min(360, int(plot_distance / 4)))

            if self.sub_state == _AUTO_SUB_SCAN:
                plot_score = max(0, min(360, int(plot_score / 4)))
            else:
                plot_score = int(plot_score)

            self._app.bluetooth_mgr.send_plotter_data([
                plot_distance,
                plot_score,
                int(plot_angle),
            ])

        if not self._active:
            return None
        return self.motor_output

    def stop(self):
        """Clean shutdown: zero motors, disable range sensor, cut power."""
        self._active = False
        self.target_output = (0, 0)
        self.motor_output = (0, 0)
        self._disable_sensors()
        self._app.enable_motors(False, MOTOR_ENABLE_USER_STATE)
        if len(self._app.hexdrive_apps) > 0:
            self._app.hexdrive_apps[0].set_motors((0, 0))
        self.status = ""

    def _read_gyro_delta(self, delta_ms: int) -> float:
        """Read gyro yaw and return signed delta degrees for this tick."""
        if _imu is None:
            self.gyro_dps = 0.0
            return 0.0
        try:
            raw = _imu.gyro_read()
            self.gyro_dps = float(raw[_AUTO_GYRO_AXIS])
        except Exception:  # pylint: disable=broad-except
            self.gyro_dps = 0.0
            return 0.0

        if abs(self.gyro_dps) <= _AUTO_GYRO_DEADBAND_DPS:
            return 0.0

        delta_deg = self.gyro_dps * (delta_ms / 1000.0)
        self.yaw_deg += delta_deg
        return delta_deg

    @staticmethod
    def _slew(current: int, target: int, step: int) -> int:
        if current < target:
            return min(current + step, target)
        if current > target:
            return max(current - step, target)
        return current

    def _apply_output_ramp(self, delta: int):
        accel = max(1, int(self._app.acceleration))
        ticks = max(1, delta // _TICK_MS)
        step = accel * ticks
        max_power = int(self._app.max_power)

        target_l = max(-max_power, min(max_power, int(self.target_output[0])))
        target_r = max(-max_power, min(max_power, int(self.target_output[1])))
        cur_l = int(self.motor_output[0])
        cur_r = int(self.motor_output[1])

        self.motor_output = (
            self._slew(cur_l, target_l, step),
            self._slew(cur_r, target_r, step),
        )

    def _drive_speed(self) -> int:
        return max(self._app.settings['auto_speed'].v, _AUTO_CRUISE_MIN_PWM)

    def _turn_speed(self) -> int:
        return max(1, int(self._drive_speed() * _AUTO_TURN_SPEED_FRAC))

    def _scan_speed(self) -> int:
        return max(1, int(self._app.settings['auto_scan_speed'].v))

    def _heading_to_turn(self, heading_deg: float) -> tuple[int, float]:
        """Convert a target heading to the shortest signed turn from the current yaw."""
        target = heading_deg % 360.0
        current = self.yaw_deg % 360.0
        delta = (target - current) % 360.0
        if delta > 180.0:
            return -1, 360.0 - delta
        return 1, delta

    def _enter_drive(self):
        self.sub_state = _AUTO_SUB_DRIVE
        self.forward_hold_ms = _AUTO_MIN_FWD_MS
        self.obstacle_hits = 0
        self.turn_progress_deg = 0.0
        self.target_deg = 0.0
        speed = self._drive_speed()
        self.target_output = (speed, speed)
        self.status = "Fwd"

    def _update_drive(self, delta: int):
        speed = self._drive_speed()
        self.target_output = (speed, speed)

        if self.forward_hold_ms > 0:
            self.forward_hold_ms = max(0, self.forward_hold_ms - delta)

        obstacle_mm = self._app.settings['auto_obstacle'].v
        obstacle_detected = self.distance is not None and self.distance < obstacle_mm

        if obstacle_detected and self.forward_hold_ms == 0:
            self.obstacle_hits += 1
        else:
            self.obstacle_hits = 0

        if self.obstacle_hits >= _AUTO_OBSTACLE_CONFIRM_SAMPLES:
            if self.distance is not None and self.distance < _AUTO_BACKUP_NEAR_MM:
                self._enter_reverse()
            else:
                self._enter_scan()
            return

        dist_label = f"{self.distance}mm" if self.distance is not None else "---"
        self.status = f"Fwd {dist_label}"

    def _enter_reverse(self):
        self.sub_state = _AUTO_SUB_REVERSE
        self.reverse_timer = 0
        back_speed = max(1, int(self._drive_speed() * _AUTO_BACKUP_SPEED_FRAC))
        self.target_output = (-back_speed, -back_speed)
        self.status = "Reverse"
        if self._logging:
            print(f"A:Obstacle at {self.distance}mm, reversing")

    def _update_reverse(self, delta: int):
        back_speed = max(1, int(self._drive_speed() * _AUTO_BACKUP_SPEED_FRAC))
        self.target_output = (-back_speed, -back_speed)
        self.reverse_timer += delta
        self.status = f"Reverse {self.reverse_timer}ms"
        if self.reverse_timer >= _AUTO_BACKUP_MS:
            self._enter_scan()

    def _enter_scan(self):
        self.sub_state = _AUTO_SUB_SCAN
        self.scan_data = []
        self.scan_timer = 0
        self.scan_progress_deg = 0.0
        self.best_angle_deg = 0.0
        self.best_dist_mm = 0
        self.scan_trigger_deg = self.yaw_deg % 360.0
        self.scan_start_yaw_deg = self.yaw_deg

        spin = self._scan_speed()
        self.target_output = (spin, -spin)
        self.status = "Scan 0deg"

        if self._logging:
            print(f"A:Scan start, obstacle={self.distance}")

    @staticmethod
    def _effective_scan_dist(dist_mm: int | float) -> float:
        """Treat saturated sensor values as unknown rather than as a clear opening."""
        if dist_mm is None:
            return 0.0
        if dist_mm >= _AUTO_CLEAR_DIST_MM:
            return 0.0
        return float(dist_mm)

    def _scan_record_sample(self):
        dist = self.distance if self.distance is not None else _AUTO_CLEAR_DIST_MM
        if dist > _AUTO_CLEAR_DIST_MM:
            dist = _AUTO_CLEAR_DIST_MM
        angle = min(self.scan_progress_deg, _AUTO_SCAN_TARGET_DEG)
        self.scan_data.append((angle, dist))
        if self._logging:
            print(f"A:ScanSample angle={angle:.1f}deg dist={dist}mm")

    def _select_best_scan_heading(self) -> tuple[float, int] | None:
        """Pick the clearest heading using a local rolling average and exclude the trigger sector."""
        if not self.scan_data:
            return None
        if len(self.scan_data) == 1:
            angle, dist_mm = self.scan_data[0]
            if abs((angle - self.scan_trigger_deg) % 360.0) <= _AUTO_SCAN_EXCLUDE_DEG:
                return None
            return self.scan_data[0]

        best_idx = -1
        best_score = -1.0
        best_turn_cost = 9999.0
        sample_count = len(self.scan_data)

        for idx, (angle, dist_mm) in enumerate(self.scan_data):
            delta_to_trigger = abs((angle - self.scan_trigger_deg) % 360.0)
            if delta_to_trigger > 180.0:
                delta_to_trigger = 360.0 - delta_to_trigger

            window = []
            for offset in range(-2, 3):
                sample_idx = (idx + offset) % sample_count
                window.append(self._effective_scan_dist(self.scan_data[sample_idx][1]))
            rolling_avg = sum(window) / len(window)
            current_dist = self._effective_scan_dist(dist_mm)
            score = (current_dist * 0.75) + (rolling_avg * 1.25)

            if delta_to_trigger <= _AUTO_SCAN_EXCLUDE_DEG:
                score -= 10000.0

            turn_cost = angle % 360.0
            if turn_cost > 180.0:
                turn_cost = 360.0 - turn_cost

            if (score > best_score + 0.001
                    or (abs(score - best_score) <= 0.001 and turn_cost < best_turn_cost)):
                best_idx = idx
                best_score = score
                best_turn_cost = turn_cost

        if best_idx < 0:
            return None
        return self.scan_data[best_idx]

    def _update_scan(self, delta: int, gyro_delta_deg: float):
        if self.scan_pause_ms > 0:
            self._update_scan_pause(delta)
            return

        spin = self._scan_speed()
        self.target_output = (spin, -spin)

        self.scan_timer += delta
        self.scan_progress_deg += abs(gyro_delta_deg)
        samples = len(self.scan_data)
        self.status = f"Scan {self.scan_progress_deg:.0f}deg ({samples}pts)"

        full_turn = self.scan_progress_deg >= _AUTO_SCAN_TARGET_DEG
        timed_out = self.scan_timer >= _AUTO_SCAN_TIMEOUT_MS
        if not (full_turn or timed_out):
            return

        if not self.scan_data:
            if self._logging:
                print("A:Scan done but no range samples, continuing forward")
            self._enter_drive()
            return

        best = self._select_best_scan_heading()
        if best is None:
            self._enter_drive()
            return

        self.best_angle_deg, self.best_dist_mm = best

        # The selected scan angle is measured relative to the scan start, so convert it
        # to an absolute heading in world space and then choose the shortest signed turn
        # from the robot's current heading.
        target_heading = (self.scan_start_yaw_deg + self.best_angle_deg) % 360.0
        turn_dir, turn_deg = self._heading_to_turn(target_heading)
        self.turn_dir = turn_dir
        self.target_deg = turn_deg

        if self._logging:
            reason = "360" if full_turn else "timeout"
            print(
                "A:Scan done "
                + reason
                + f" best={self.best_angle_deg:.1f}deg dist={self.best_dist_mm}mm"
                + f" target={self.target_deg:.1f}deg"
            )

        if self.best_dist_mm >= _AUTO_CLEAR_DIST_MM:
            self._enter_drive()
            self.status = "Clear ahead"
            return

        if self.target_deg <= _AUTO_TURN_STOP_MARGIN_DEG:
            self._enter_drive()
            return

        self.scan_pause_ms = _AUTO_SCAN_PAUSE_MS
        self.target_output = (0, 0)
        self.status = f"Pause {self.scan_pause_ms}ms"
        self._app.refresh = True

    def _update_scan_pause(self, delta: int):
        self.scan_pause_ms = max(0, self.scan_pause_ms - delta)
        self.target_output = (0, 0)
        self.status = f"Pause {self.scan_pause_ms}ms"
        self._app.refresh = True
        if self.scan_pause_ms <= 0:
            self._enter_turn()

    def _enter_turn(self):
        self.sub_state = _AUTO_SUB_TURN
        self.turn_timer = 0
        self.turn_progress_deg = 0.0

        spin = self._turn_speed()
        self.target_output = (spin * self.turn_dir, -spin * self.turn_dir)

        frac = self.target_deg / 180.0
        self.turn_timeout_ms = int(_AUTO_TURN_TIMEOUT_MIN_MS + frac * 2500)
        if self.turn_timeout_ms > _AUTO_TURN_TIMEOUT_MAX_MS:
            self.turn_timeout_ms = _AUTO_TURN_TIMEOUT_MAX_MS

        lbl = "right" if self.turn_dir > 0 else "left"
        self.status = f"Turn {lbl} {self.target_deg:.0f}deg"

    def _update_turn(self, delta: int, gyro_delta_deg: float):
        spin = self._turn_speed()
        self.target_output = (spin * self.turn_dir, -spin * self.turn_dir)

        self.turn_timer += delta
        self.turn_progress_deg += gyro_delta_deg

        lbl = "right" if self.turn_dir > 0 else "left"
        self.status = f"Turn {lbl} {self.turn_progress_deg:.0f}/{self.target_deg:.0f}deg"

        signed_turn = self.turn_progress_deg * self.turn_dir
        gyro_done = signed_turn >= max(0.0, self.target_deg - _AUTO_TURN_STOP_MARGIN_DEG)
        time_done = self.turn_timer >= self.turn_timeout_ms

        if gyro_done or time_done:
            if self._logging:
                reason = "gyro" if gyro_done else "timeout"
                print(
                    f"A:Turn done {reason} turned={self.turn_progress_deg:.1f} "
                    f"target={self.target_deg:.1f} signed={signed_turn:.1f}"
                )
            self._enter_drive()

    def _enable_sensors(self) -> bool:
        """Enable range sensing through SensorTestMgr polling APIs."""
        self._range_hexdrive = None
        if self._sensor_mgr is None:
            return False

        range_hexdrive = self._sensor_mgr.active_range_hexdrive()
        if range_hexdrive is None:
            return False

        if not self._sensor_mgr.enable_range_sensor(
            range_hexdrive,
            period=_AUTO_SENSOR_READ_MS,
            events=False,
            interrupts=False,
        ):
            return False

        self._range_hexdrive = range_hexdrive
        return True

    def _disable_sensors(self):
        """Disable range sensing via SensorTestMgr APIs."""
        if self._sensor_mgr is not None and self._range_hexdrive is not None:
            self._sensor_mgr.disable_range_sensor(self._range_hexdrive)
        self._range_hexdrive = None
