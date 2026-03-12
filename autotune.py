# PID Auto-Tuning Module for BadgeBot Line Follower
#
# Implements relay feedback auto-tuning (Åström-Hägglund method) to determine
# PID gains for line-following steering.  The robot is driven over a line with
# relay (bang-bang) control while oscillation characteristics are measured.
# PID gains are then calculated using Ziegler-Nichols tuning rules.
#
# Algorithm overview:
#   1. Apply relay control: output switches between +relay_amplitude and
#      -relay_amplitude as the error signal crosses zero (with hysteresis).
#   2. Measure the resulting oscillation period (Tu) and amplitude (a).
#   3. Compute the ultimate gain:  Ku = 4*d / (pi * a)
#   4. Derive PID gains via Ziegler-Nichols:
#        Kp = 0.6  * Ku
#        Ki = 1.2  * Ku / Tu
#        Kd = 0.075 * Ku * Tu
#
# The training line should ideally include gentle curves so that the controller
# is exercised across a range of error magnitudes, but a straight line will
# also work for basic tuning.  Curves help produce more representative
# oscillation data and result in gains that generalise better to real tracks.
#
# Reference: https://github.com/lily-osp/AutoTunePID

# Tuning method constants
METHOD_ZIEGLER_NICHOLS  = 0
METHOD_TYREUS_LUYBEN    = 1
METHOD_SOME_OVERSHOOT   = 2
METHOD_NO_OVERSHOOT     = 3

_METHOD_NAMES = [
    "Ziegler-Nichols",
    "Tyreus-Luyben",
    "Some Overshoot",
    "No Overshoot",
]

# Auto-tune states
_AT_IDLE       = 0
_AT_RELAY      = 1
_AT_DONE       = 2
_AT_FAILED     = 3

# Quality thresholds
_MIN_CYCLES          = 4    # Minimum oscillation half-cycles to accept
_MAX_CYCLES          = 20   # Stop after this many half-cycles
_MIN_AMPLITUDE       = 0.01 # Minimum normalised amplitude to be meaningful
_SETTLE_IGNORE       = 2    # Ignore first N half-cycles (transient)


class PIDAutoTuner:
    """Relay-feedback PID auto-tuner.

    Parameters
    ----------
    relay_amplitude : int
        Motor power magnitude applied during relay control (e.g. max_power).
    base_power : int
        Forward drive power applied to both motors (keeps the robot moving).
    hysteresis : float
        Dead-band half-width around zero error for relay switching (normalised
        error units, 0-1).  Prevents chatter when the error is near zero.
    target_cycles : int
        Number of oscillation half-cycles to collect before finishing.
    method : int
        Tuning rule to use (one of METHOD_* constants).
    logging : bool
        If True, emit detailed diagnostic prints during the tuning process.
    """

    def __init__(self, relay_amplitude, base_power=0, hysteresis=0.05,
                 target_cycles=12, method=METHOD_ZIEGLER_NICHOLS, logging=True):
        self.relay_amplitude = relay_amplitude
        self.base_power = base_power
        self.hysteresis = hysteresis
        self.target_cycles = max(target_cycles, _MIN_CYCLES + _SETTLE_IGNORE + 1)
        self.method = method
        self.logging = logging

        # State
        self.state = _AT_IDLE
        self.relay_sign = 1          # +1 or -1
        self._prev_error = 0.0
        self._elapsed_ms = 0         # accumulated elapsed time (ms)

        # Oscillation measurement
        self._crossing_times = []    # timestamps (ms) of zero-crossings
        self._peaks = []             # peak error values (positive half-cycles)
        self._troughs = []           # trough error values (negative half-cycles)
        self._cur_extreme = 0.0      # running extreme in current half-cycle

        # Results
        self._Ku = 0.0               # Ultimate gain
        self._Tu = 0.0               # Ultimate period (ms)
        self._Kp = 0.0
        self._Ki = 0.0
        self._Kd = 0.0
        self._quality = 0.0

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def start(self):
        """Begin the auto-tune process.  Call update() in each control loop."""
        self.state = _AT_RELAY
        self.relay_sign = 1
        self._prev_error = 0.0
        self._crossing_times = []
        self._peaks = []
        self._troughs = []
        self._cur_extreme = 0.0
        self._elapsed_ms = 0
        if self.logging:
            print("AUTOTUNE: Started relay feedback auto-tune")
            print("AUTOTUNE: relay_amp=" + str(self.relay_amplitude) + "  base_power=" + str(self.base_power) + "  hysteresis=" + str(self.hysteresis) + "  target_cycles=" + str(self.target_cycles) + "  method=" + _METHOD_NAMES[self.method])

    def update(self, error, delta):
        """Feed a new error measurement and return the motor output tuple.

        Parameters
        ----------
        error : float
            Normalised line-position error in range [-1, +1].
            Negative = line is to the left, positive = line is to the right.
        delta : int
            Elapsed time in milliseconds since the last call to update().

        Returns
        -------
        tuple (left_motor_power, right_motor_power)
        """
        if self.state != _AT_RELAY:
            return (0, 0)

        self._elapsed_ms += delta

        # --- Detect zero crossing with hysteresis ---
        crossed = False
        if self.relay_sign == 1 and error < -self.hysteresis:
            # Was positive relay, error crossed below -hysteresis → switch
            crossed = True
            self._peaks.append(self._cur_extreme)
        elif self.relay_sign == -1 and error > self.hysteresis:
            # Was negative relay, error crossed above +hysteresis → switch
            crossed = True
            self._troughs.append(self._cur_extreme)

        if crossed:
            self.relay_sign = -self.relay_sign
            self._crossing_times.append(self._elapsed_ms)
            self._cur_extreme = error  # reset extreme tracking
            n = len(self._crossing_times)
            if self.logging:
                sign_ch = "+" if self.relay_sign > 0 else "-"
                print("AUTOTUNE: crossing #" + str(n) + "  t=" + str(self._elapsed_ms) + "ms  error=" + str(error) + "  relay->" + sign_ch + "  peaks=" + str(len(self._peaks)) + " troughs=" + str(len(self._troughs)))
            if n >= self.target_cycles:
                self._finish()
                return (0, 0)

        # Track the extreme (peak or trough) in this half-cycle
        if self.relay_sign == 1:
            if error > self._cur_extreme:
                self._cur_extreme = error
        else:
            if error < self._cur_extreme:
                self._cur_extreme = error

        self._prev_error = error

        # --- Compute relay output ---
        steering = self.relay_amplitude * self.relay_sign
        left  = self.base_power + steering
        right = self.base_power - steering
        return (left, right)

    @property
    def is_running(self):
        return self.state == _AT_RELAY

    @property
    def is_complete(self):
        return self.state == _AT_DONE

    @property
    def is_failed(self):
        return self.state == _AT_FAILED

    def get_gains(self):
        """Return the computed PID gains as (Kp, Ki, Kd).

        Returns None if tuning has not completed successfully.
        """
        if self.state != _AT_DONE:
            return None
        return (self._Kp, self._Ki, self._Kd)

    def get_diagnostics(self):
        """Return a dict of diagnostic values for display/logging."""
        n_crossings = len(self._crossing_times)
        return {
            "state":     self.state,
            "crossings": n_crossings,
            "target":    self.target_cycles,
            "Ku":        self._Ku,
            "Tu_ms":     self._Tu,
            "Kp":        self._Kp,
            "Ki":        self._Ki,
            "Kd":        self._Kd,
            "quality":   self._quality,
            "method":    _METHOD_NAMES[self.method],
            "elapsed":   self._elapsed_ms,
        }

    def get_quality(self):
        """Return a quality score 0-100 for the tuning result.

        The score is based on:
          - Consistency of oscillation period (lower variance → higher score)
          - Consistency of oscillation amplitude
          - Number of valid cycles collected
        Returns 0 if tuning has not completed.
        """
        return self._quality

    def get_status_text(self):
        """Return a short human-readable status string."""
        if self.state == _AT_IDLE:
            return "Idle"
        elif self.state == _AT_RELAY:
            n = len(self._crossing_times)
            return f"Tuning {n}/{self.target_cycles}"
        elif self.state == _AT_DONE:
            return f"Done Q={self._quality:.0f}%"
        else:
            return "Failed"

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _finish(self):
        """Analyse collected oscillation data and compute PID gains."""
        if self.logging:
            print("AUTOTUNE: Finishing - analysing oscillation data")
            print("AUTOTUNE: " + str(len(self._crossing_times)) + " crossings, " + str(len(self._peaks)) + " peaks, " + str(len(self._troughs)) + " troughs")

        # Need at least _MIN_CYCLES half-cycles after settling
        usable_crossings = len(self._crossing_times) - _SETTLE_IGNORE
        if usable_crossings < _MIN_CYCLES:
            if self.logging:
                print("AUTOTUNE: FAILED - only " + str(usable_crossings) + " usable crossings (need " + str(_MIN_CYCLES) + ")")
            self.state = _AT_FAILED
            return

        # --- Calculate oscillation period ---
        # Period = time between every other crossing (full cycle)
        periods = []
        ct = self._crossing_times
        for i in range(_SETTLE_IGNORE + 2, len(ct)):
            p = ct[i] - ct[i - 2]
            if p > 0:
                periods.append(p)

        if not periods:
            if self.logging:
                print("AUTOTUNE: FAILED - no valid periods measured")
            self.state = _AT_FAILED
            return

        Tu = sum(periods) / len(periods)  # average period in ms
        if self.logging:
            print(f"AUTOTUNE: Periods (ms): {periods}")
            print(f"AUTOTUNE: Average period Tu = {Tu:.1f} ms")

        # --- Calculate oscillation amplitude ---
        # Use peaks/troughs after the settling window
        p_start = _SETTLE_IGNORE // 2
        valid_peaks   = self._peaks[p_start:]   if len(self._peaks)   > p_start else self._peaks
        valid_troughs = self._troughs[p_start:] if len(self._troughs) > p_start else self._troughs

        if not valid_peaks or not valid_troughs:
            if self.logging:
                print("AUTOTUNE: FAILED - insufficient peak/trough data")
            self.state = _AT_FAILED
            return

        avg_peak   = sum(abs(p) for p in valid_peaks)   / len(valid_peaks)
        avg_trough = sum(abs(t) for t in valid_troughs) / len(valid_troughs)
        amplitude = (avg_peak + avg_trough) / 2.0  # average half-amplitude

        if self.logging:
            print("AUTOTUNE: Peaks: " + str(["%.4f" % p for p in valid_peaks]))
            print("AUTOTUNE: Troughs: " + str(["%.4f" % t for t in valid_troughs]))
            print(f"AUTOTUNE: Average amplitude a = {amplitude:.4f}")

        if amplitude < _MIN_AMPLITUDE:
            if self.logging:
                print("AUTOTUNE: FAILED - amplitude " + str(amplitude) + " too small (min " + str(_MIN_AMPLITUDE) + ")")
            self.state = _AT_FAILED
            return

        # --- Calculate ultimate gain ---
        # Ku = 4*d / (pi * a)  where d = relay_amplitude, a = amplitude
        pi = 3.14159265
        self._Ku = (4.0 * self.relay_amplitude) / (pi * amplitude)
        self._Tu = Tu

        if self.logging:
            print(f"AUTOTUNE: Ultimate gain Ku = {self._Ku:.4f}")
            print(f"AUTOTUNE: Ultimate period Tu = {self._Tu:.1f} ms")

        # --- Apply tuning rules ---
        self._apply_tuning_rules()

        # --- Calculate quality score ---
        self._quality = self._calc_quality(periods, valid_peaks, valid_troughs)

        self.state = _AT_DONE
        if self.logging:
            print(f"AUTOTUNE: SUCCESS - Kp={self._Kp:.4f}  Ki={self._Ki:.6f}  Kd={self._Kd:.4f}")
            print(f"AUTOTUNE: Quality score = {self._quality:.1f}%")
            print(f"AUTOTUNE: Method = {_METHOD_NAMES[self.method]}")

    def _apply_tuning_rules(self):
        """Compute Kp, Ki, Kd from Ku and Tu using the selected method."""
        Ku = self._Ku
        Tu = self._Tu

        if self.method == METHOD_ZIEGLER_NICHOLS:
            # Classic Ziegler-Nichols PID
            self._Kp = 0.6 * Ku
            self._Ki = 1.2 * Ku / Tu   # = 2 * Kp / Tu
            self._Kd = 0.075 * Ku * Tu # = Kp * Tu / 8

        elif self.method == METHOD_TYREUS_LUYBEN:
            # Tyreus-Luyben: less aggressive, reduced oscillation
            self._Kp = 0.4545 * Ku     # Ku / 2.2
            self._Ki = self._Kp / (2.2 * Tu)
            self._Kd = self._Kp * Tu / 6.3

        elif self.method == METHOD_SOME_OVERSHOOT:
            # Moderate tuning: Kp/3, with some overshoot tolerance
            self._Kp = 0.33 * Ku
            self._Ki = 0.66 * Ku / Tu
            self._Kd = 0.11 * Ku * Tu

        elif self.method == METHOD_NO_OVERSHOOT:
            # Conservative: minimal overshoot
            self._Kp = 0.2 * Ku
            self._Ki = 0.4 * Ku / Tu
            self._Kd = 0.066 * Ku * Tu

    def _calc_quality(self, periods, peaks, troughs):
        """Compute a quality score 0-100 based on oscillation consistency."""
        score = 100.0

        # 1. Period consistency (coefficient of variation)
        if len(periods) >= 2:
            mean_p = sum(periods) / len(periods)
            if mean_p > 0:
                variance = sum((p - mean_p) ** 2 for p in periods) / len(periods)
                # Use integer square root approximation for MicroPython compatibility
                std_p = variance ** 0.5
                cv_period = std_p / mean_p
                # Penalise: cv > 0.3 → score drops significantly
                period_score = max(0, 100 - cv_period * 200)
                if self.logging:
                    print(f"AUTOTUNE: Period CV={cv_period:.3f}  period_score={period_score:.1f}")
            else:
                period_score = 0
        else:
            period_score = 50  # too few periods to judge

        # 2. Amplitude consistency
        all_extremes = [abs(p) for p in peaks] + [abs(t) for t in troughs]
        if len(all_extremes) >= 2:
            mean_a = sum(all_extremes) / len(all_extremes)
            if mean_a > 0:
                variance = sum((a - mean_a) ** 2 for a in all_extremes) / len(all_extremes)
                std_a = variance ** 0.5
                cv_amp = std_a / mean_a
                amp_score = max(0, 100 - cv_amp * 200)
                if self.logging:
                    print(f"AUTOTUNE: Amplitude CV={cv_amp:.3f}  amp_score={amp_score:.1f}")
            else:
                amp_score = 0
        else:
            amp_score = 50

        # 3. Number of cycles bonus (more cycles → more confidence)
        n_cycles = len(periods)
        cycle_score = min(100, n_cycles * 15)  # 7+ cycles for full marks

        score = (period_score * 0.4 + amp_score * 0.4 + cycle_score * 0.2)

        if self.logging:
            print("AUTOTUNE: Quality breakdown: period=" + str(period_score) + " amplitude=" + str(amp_score) + " cycles=" + str(cycle_score) + " total=" + str(score) + "%")

        return score


def compute_error(left_raw, right_raw):
    """Compute a normalised error from raw sensor discharge times.

    Parameters
    ----------
    left_raw : int
        Raw discharge time (µs) from the left sensor.
    right_raw : int
        Raw discharge time (µs) from the right sensor.

    Returns
    -------
    float
        Error in range [-1, +1].
        Negative = line is to the left (steer left).
        Positive = line is to the right (steer right).

    The sensor with the *shorter* discharge time is closer to the line
    (higher reflectance).  So if left_raw < right_raw the line is to the
    left and the error is negative.
    """
    total = left_raw + right_raw
    if total == 0:
        return 0.0
    return (left_raw - right_raw) / total
