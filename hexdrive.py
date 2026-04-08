"""HexDrive Hexpansion App for BadgeBot."""

# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import ota
from machine import PWM, Pin
from system.eventbus import eventbus
from system.hexpansion.config import HexpansionConfig
from system.scheduler.events import RequestStopAppEvent

import app

# HexDrive.py App Version - used to check if upgrade is required
VERSION = 7

# HexDrive Hexpansion constants
# Hardware defintions:
_ENABLE_PIN  = 0  # First LS pin used to enable the SMPSU
#_DETECT_PIN  = 1  # Second LS pin used to sense if the SMPSU has a source of power

# Default values and limits:
_DEFAULT_PWM_FREQ = 20000           # 20kHz is a good default for motors as it is above the audible range for most people and works with most motors and ESCs
_DEFAULT_SERVO_FREQ = 50            # 50Hz = 20mS period
_DEFAULT_KEEP_ALIVE_PERIOD = 1000   # 1 second
_MAX_NUM_CHANNELS = 4               # Max number of PWM channels supported by any type of HexDrive (Hexpansion limitation, not BadgeBot limit)
_MAX_NUM_MOTORS = 2                 # Max number of motor channels supported by any type of HexDrive

# Servo Constants
_MAX_SERVO_FREQ = 200               # 200Hz = 5mS period (can work with some Servos but not all)
_SERVO_CENTRE    = 1500             # 1500us pulse width is the centre position for most RC servos (but some may be different, so we allow this to be trimmed)
_MAX_SERVO_RANGE = 1400             # 1400us either side of centre (VERY WIDE)
_SERVO_MAX_TRIM  = 1000             # 1000us either side of centre for trimming the centre position

# Stepper Motor Constants
_STEPPER_NUM_PHASES    = 8          # Number of phases in the stepping sequence (this includes half steps)
_STEPPER_SEQUENCE = (
    (1, 0, 1, 0),
    (0, 0, 1, 0),
    (0, 1, 1, 0),
    (0, 1, 0, 0),
    (0, 1, 0, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
    (1, 0, 0, 0),
)

# EEPROM Constants
_EEPROM_ADDR  = 0x50                # I2C address of the EEPROM on the HexDrive and HexSense Hexpansion
_EEPROM_NUM_ADDRESS_BYTES = 2       # Number of bytes used for the memory address when reading from the EEPROM (e.g. 2 for 16-bit addressing)
_PID_ADDR     = 0x12                # Address in the EEPROM where the Product ID (PID) byte is stored - used to identify the type of Hexpansion

# PID MSByte value for RobotMad HexDrive Hexpansion modules (used to identify the type of HexDrive when reading the EEPROM)
_HEXDRIVE_PID_MSB = 0xCB


class HexDriveType:
    """Represents a sub-type of HexDrive Hexpansion module."""
    __slots__ = ("pid", "name", "motors", "servos", "steppers")

    def __init__(self, pid_byte: int, motors: int = 0, servos: int = 0, steppers: int = 0, name: str = "Unknown"):
        self.pid: int = pid_byte         # Product ID byte read from the EEPROM to identify the type of HexDrive
        self.name: str = name            # A friendly name for the type of HexDrive
        self.motors: int = motors        # Number of motor channels supported by this type of HexDrive (0, 1 or 2)
        self.servos: int = servos        # Number of servo channels supported by this type of HexDrive (0, 2 or 4)
        self.steppers: int = steppers    # Number of stepper motors supported by this type of HexDrive (0 or 1)

_HEXDRIVE_TYPES = (
    HexDriveType(0xCA, motors=2, name="2 Motor"),
    HexDriveType(0xCB, motors=2, servos=4, steppers=1),
    HexDriveType(0xCC, servos=4, name="4 Servo"),
    HexDriveType(0xCD, motors=1, servos=2, name="1 Mot 2 Srvo"),
    HexDriveType(0xCE, steppers=1, name="1 Stepper"),
)

class HexDriveApp(app.App):         # pylint: disable=no-member
    """ HexDrive Hexpansion App for BadgeBot."""
    def __init__(self, config: HexpansionConfig | None = None):
        super().__init__()
        self.config: HexpansionConfig | None = config
        self._hexdrive_type: HexDriveType | None = None
        self._logging: bool = True
        self._keep_alive_period: int = _DEFAULT_KEEP_ALIVE_PERIOD
        self._power_state: bool = False
        self._pwm_setup: bool = False
        self._time_since_last_update: int = 0
        self._outputs_energised: bool = False
        self.PWMOutput: list[PWM | None] = [None] * _MAX_NUM_CHANNELS
        self._freq: list[int] = [0] * _MAX_NUM_CHANNELS
        self._motor_output: list[int] = [0] * _MAX_NUM_MOTORS
        self._stepper: bool = False
        if config is None:
            print("D:No Config")
            return
        # LS Pins
        #self._power_detect  = self.config.ls_pin[_DETECT_PIN]
        self._power_control = self.config.ls_pin[_ENABLE_PIN]

        self._servo_centre = [_SERVO_CENTRE] * _MAX_NUM_CHANNELS
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)
        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            #print(f"D:S/W {ver}")
            # e.g. v1.9.0-beta.1
            if ver >= [1, 9, 0]:
                # we need v1.9.0+ to be able to read the EEPROM with 16-bit addressing, so if we are running on an older version then we cannot continue
                pass
            else:
                print("D:BadgeOS Upgrade to v1.9.0+ required")
                return
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:Ver check failed {e}")
        self.initialise()


    def initialise(self) -> bool:
        """Initialise the app - return True if successful, False if failed."""
        self._pwm_setup = False
        if self.config is None:
            return False
        # report app starting and which port it is running on
        print(f"D:HexDrive V{VERSION} by RobotMad on port {self.config.port}")
        # Initialise HS Pins
        for _, hs_pin in enumerate(self.config.pin):
            # Set HexDrive Hexpansion HS pins to low level outputs
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)
        # Initialise LS Pins
        try:
            #self._power_detect.init(mode=Pin.IN)
            self._power_control.init(mode=Pin.OUT)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:ls_pin setup failed {e}")
            return False
        # ensure SMPSU is turned off to start with
        self.set_power(False)

        # read hexpansion header from EEPROM to find out which sub-type we are
        # and allocate PWM outputs accordingly
        self._hexdrive_type = self._check_port_for_hexdrive(self.config.port)
        if self._logging and self._hexdrive_type is not None:
            print(f"D:{self.config.port}:Type:'{self._hexdrive_type.name}'")

        return self._pwm_init()


    def deinitialise(self) -> bool:
        """ De-initialise the app - return True if successful, False if failed."""
        # Turn off all PWM outputs & release resources
        self.set_power(False)
        self._pwm_deinit() 
        for hs_pin in self.config.pin:
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)                  
        return True


    async def _handle_stop_app(self, event):
        """ Handle the RequestStopAppEvent so that we can release resources """
        try:
            if event.app == self:
                if self._logging:
                    print(f"D:{self.config.port}:Stop")
                self.deinitialise()
        except (AttributeError, TypeError):
            pass


    def background_update(self, delta: int):
        """ This is called from the main loop of the BadgeOS to allow the app to do any background processing it needs to do. """
        if (self.config is None) or not (self._pwm_setup or self._stepper):
            # if we are not properly initialised then do not attempt to do anything
            return
        # Check keep alive period and turn off PWM outputs if exceeded
        self._time_since_last_update += delta
        if self._time_since_last_update > self._keep_alive_period:
            self._time_since_last_update = 0
            if self._outputs_energised:
                self._outputs_energised = False
                # First time the keep alive period has expired so report it
                if self._logging:
                    print(f"D:{self.config.port}:Timeout")
            if self._pwm_setup:
                for channel,pwm in enumerate(self.PWMOutput):
                    if pwm is not None:
                        try:
                            pwm.duty_u16(0)
                        except Exception as e:          # pylint: disable=broad-except
                            print(self._pwm_log_string(channel) + f"Off failed {e}")
                            self.PWMOutput[channel] = None  # Tidy Up
            elif self._stepper:
                self.motor_release()
            # we keep retriggering in case anything else has corrupted the PWM outputs


    def get_version(self) -> int:
        """ Get the version of the app - this is used to determine if an upgrade is required. """
        return VERSION


    def get_status(self) -> bool:
        """ Get the current status of the app - True if the app is running and able to respond to commands, False if not. """
        return (self._pwm_setup or self._stepper)


    def set_logging(self, state: bool):
        """ Set the logging state - True to enable logging, False to disable logging. """
        self._logging = state


    def set_power(self, state: bool) -> bool:
        """ Turn the SMPSU on or off. Returns the new power state.
            Note that just because the SMPSU is turned off does not mean that the outputs are NOT energised as there could be external battery power. """
        if (self.config is None) or (state == self._power_state):
            return False
        if self._logging:
            print(f"D:{self.config.port}:Power={'On' if state else 'Off'}")
        #if self.get_booster_power():
            # if the power detect pin is high then the SMPSU has a power source so enable it
        try:
            self._power_control.init(mode=Pin.OUT)
            self._power_control.value(state)
        except Exception as e:      # pylint: disable=broad-except
            print(f"D:{self.config.port}:power control failed {e}")
            return False    
        self._power_state = state
        return self._power_state    


    def get_power(self) -> bool:
        """ Get the current state of the SMPSU enable pin. Returns True if enabled, False if disabled. """
        return self._power_state


    def set_keep_alive(self, period: int):
        """ Set the keep alive period in milliseconds:
            This is the period of time that can elapse without any commands being received before the app automatically
            turns off all outputs to prevent damage to motors or servos if something goes wrong. """
        self._keep_alive_period = period


    def set_freq(self, freq: int, channel: int | None = None) -> bool:
        """ Set the PWM frequency for a specific output, or all outputs if channel is None. Returns True if successful, False if failed.
            Use 50 to 200 for Servos and 5000 to 20000 for motors. """
        if not self._pwm_setup:
            return False
        for this_channel, pwm in enumerate(self.PWMOutput):
            if (channel is None or this_channel == channel) and pwm is not None:
                try:
                    pwm.freq(freq)
                    if self._logging:
                        print(self._pwm_log_string(this_channel) + f"{freq}Hz set")
                except Exception as e:  # pylint: disable=broad-except
                    print(self._pwm_log_string(this_channel) + f"set freq {freq} failed {e}")
                    print(f"pwm: {pwm}")
                    return False
                self._freq[this_channel] = freq
        return True


    def _pwm_log_string(self, channel: int | None) -> str:
        """ Helper method to generate a log string for a PWM output change. """
        if channel is None:
            return f"D:{self.config.port}:PWM:[All]:"
        return f"D:{self.config.port}:PWM[{channel}]:"


    def set_servoposition(self, channel: int | None = None, position: int | None = None) -> bool:
        """ Set the position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            The pulse width for a specific servo output is position + the centre offset (in us)
            Based on standard RC servos with centre at 1500us and range of 1000-2000us.
            The position is a signed value from -1000 to 1000 which is scaled to 500-2500us.
            This is a very wide range and may not be suitable for all servos, some will 
            only be happy with 1000-2000us (i.e. position in the range -500 to 500). """
        if not self._pwm_setup:
            return False
        if position is None:
            # position == None -> Turn off PWM (some servos will then turn off, others will stay in last position)
            if channel is None:
                # channel == None -> Turn off all PWM outputs
                for ch, pwm in enumerate(self.PWMOutput):
                    if pwm is not None:
                        try:
                            pwm.duty_ns(0)
                        except Exception as e:  # pylint: disable=broad-except
                            print(self._pwm_log_string(ch) + f"Off failed {e}")
                if self._logging:
                    print(self._pwm_log_string(None) + "Off")
                self._outputs_energised = False
                return True
            elif channel < 0 or channel >= self._hexdrive_type.servos:
                return False
            else:
                try:
                    self.PWMOutput[channel].duty_ns(0)
                    if self._logging:
                        print(self._pwm_log_string(channel) + "Off")
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"Off failed {e}")
                    return False
            # check if all channels are now off and set outputs_energised accordingly
            self._check_outputs_energised()          
        elif channel is not None:           
            if channel < 0 or channel >= self._hexdrive_type.servos:
                return False
            if abs(position) > _MAX_SERVO_RANGE:
                return False
            pulse_width_in_ns = (self._servo_centre[channel] + position) * 1000 # convert from us to ns
            if self.PWMOutput[channel] is None:
                # Channel hasn't been setup yet so we need to initialise it from scratch
                self._freq[channel] = self._freq[channel] if (0 < self._freq[channel]) and (self._freq[channel] <= _MAX_SERVO_FREQ) else _DEFAULT_SERVO_FREQ
                try:
                    self.PWMOutput[channel] = PWM(self.config.pin[channel], freq = self._freq[channel], duty_ns = pulse_width_in_ns)
                    if self._logging:
                        print(self._pwm_log_string(channel) + f"{self.PWMOutput[channel]} init")
                except Exception as e:      # pylint: disable=broad-except
                    # There are a finite number of PWM resources so it is possible that we run out
                    print(self._pwm_log_string(channel) + f"PWM(init) failed {e}")
                    return False
            else:
                # Channel is already setup so we just need to change the duty cycle and possibly the frequency if it is too high for the servo
                try:
                    if _MAX_SERVO_FREQ < self.PWMOutput[channel].freq():
                        # Ensure the frequency is suitable for use with Servos
                        # otherwise the pulse width will not be accepted
                        self._freq[channel] = _DEFAULT_SERVO_FREQ
                        self.PWMOutput[channel].freq(_DEFAULT_SERVO_FREQ)
                        if self._logging:
                            print(self._pwm_log_string(channel) + f"{_DEFAULT_SERVO_FREQ}Hz for Servo")                    
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"set freq failed {e}")
                    return False
                # Scale servo position to PWM duty cycle (500-2500us)
                try:
                    if 2000 < abs(pulse_width_in_ns - self.PWMOutput[channel].duty_ns()):    # allow tolerance of 2us to avoid unnecessary updates
                        if self._logging:
                            print(self._pwm_log_string(channel) + f"{pulse_width_in_ns}ns")
                        self.PWMOutput[channel].duty_ns(pulse_width_in_ns)
                        if self._logging:
                            print(self._pwm_log_string(channel) + f"{self.PWMOutput[channel]} duty")  
                except Exception as e:          # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"set duty failed {e}")
                    return False

            self._outputs_energised = True
            self._stepper = False
        self._time_since_last_update = 0
        return True


    def set_servocentre(self, centre: int, channel: int | None = None) -> bool:
        """ Set the centre position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            Note this does not change the current position of the servo.
            It will only affect the position next time it is set.
            You can use this to trim the centre position of the servo. """
        if not self._pwm_setup:
            return False
        if channel is not None and (channel < 0 or channel >= self._hexdrive_type.servos):
            return False
        if centre < (_SERVO_CENTRE - _SERVO_MAX_TRIM ) or centre > (_SERVO_CENTRE + _SERVO_MAX_TRIM): 
            return False
        if channel is None:
            self._servo_centre = [centre] * 4
        else:
            self._servo_centre[channel] = centre
        return True


    # Set pairs of PWM duty cycles in one go using a signed value per motor channel (0-65535)
    def set_motors(self, outputs: tuple[int, ...]) -> bool:
        """ Set the motor outputs using a signed value for each motor channel. Returns True if successful, False if failed.
            The outputs are signed values in a tuple from -65535 to 65535 which are scaled to the PWM duty cycle range of 0-65535.
            A positive value will drive the motor in one direction, a negative value will drive it in the opposite direction,
            and a value of 0 will stop the motor. """
        if not self._pwm_setup or len(outputs) != self._hexdrive_type.motors:
            return False
        for motor, output in enumerate(outputs):
            if abs(output) > 65535:
                return False
            if output == self._motor_output[motor]:
                continue
            try:
                # if the output is changing direction then we need to switch which signal is being driven as the PWM output
                # rather than test for change of direction and also test that PWMOutput to be disabled exists we just do the latter check.
                output_to_enable = (motor<<1) if output > 0 else ((motor<<1)+1)
                output_to_disable = (motor<<1)+1 if output > 0 else (motor<<1)
                # switch off the currently active output before switching the other one on to prevent both outputs being on at the same time
                if self.PWMOutput[output_to_disable] is not None:
                    # we need to set the frequency of the output that is to be enabled to match the frequency of the output that is to be disabled
                    self._freq[output_to_enable] = self._freq[output_to_disable]
                    self.PWMOutput[output_to_disable].deinit()
                    self.PWMOutput[output_to_disable] = None
                    self.config.pin[output_to_disable].value(0)
                self._set_pwmoutput(output_to_enable, abs(output))                  
            except Exception as e:          # pylint: disable=broad-except
                print(f"D:{self.config.port}:Motor{motor}:{output} set failed {e}")
            self._motor_output[motor] = output
        self._check_outputs_energised()
        self._time_since_last_update = 0
        return True


    # Set all 4 PWM duty cycles in one go using a tuple (0-65535)
    def set_pwm(self, duty_cycles: tuple[int, ...]) -> bool:
        """ Set the PWM duty cycle for all outputs at once using a tuple of values. Returns True if successful, False if failed.
            The duty_cycles are values from 0 to 65535. """
        if not self._pwm_setup:
            return False
        self._outputs_energised = any(duty_cycles)
        for channel, duty_cycle in enumerate(duty_cycles): 
            if not self._set_pwmoutput(channel, duty_cycle):
                return False
        self._time_since_last_update = 0
        return True


### Stepper Motor Support

# --------------------------------------------------
# Public methods for controlling stepper motors.
# ---------------------------------------------------

    def motor_step(self, phase: int) -> int | None:
        """ Step the motor to a specific phase in the stepping sequence. Returns None if failed (e.g. invalid phase or not configured for stepper),
            otherwise returns the phase that was set. The phase is a value from 0 to _STEPPER_NUM_PHASES-1 which corresponds to the 
            stepping sequence defined in _STEPPER_SEQUENCE."""
        if phase >= _STEPPER_NUM_PHASES or self._hexdrive_type.steppers == 0:
            return None
        if not self._stepper:
            # not currently configured for stepper motor - configure
            self._pwm_deinit() 
            self._stepper = True
        for channel, value in enumerate(_STEPPER_SEQUENCE[phase]):
            self.config.pin[channel].value(value)
        self._outputs_energised = True
        self._time_since_last_update = 0
        return phase


    def motor_release(self):
        """ Release the motor by setting all outputs to low. This will stop the motor and allow it to be turned by hand. """
        for channel in range(4 if self._stepper else (self._hexdrive_type.motors * 2)):
            self.config.pin[channel].value(0)
        self._outputs_energised = False
        self._time_since_last_update = 0


# --------------------------------------------------
# Private methods for internal use only.
# --------------------------------------------------

    def _pwm_init(self) -> bool:
        self._pwm_setup = False
        # HS Pins
        if self.config.pin is not None and len(self.config.pin) == 4:             
            # Allocate PWM generation to pins
            for channel, _ in enumerate(self.config.pin):
                self._freq[channel] = 0             
                if self._hexdrive_type is not None:
                    if channel < (2 * self._hexdrive_type.motors):
                        # First channels are for motors (can be 0, 1 or 2 motors)
                        if 0 == channel % 2:
                            # initialise motor PWM output on even channel
                            self._motor_output[(channel>>1)] = 0
                            self._freq[channel] = _DEFAULT_PWM_FREQ
                            #print(f"D:{self.config.port}:Motor PWM[{channel}]")
                        else:
                            # ignore the motor PWM output on odd channel - we will switch it on when needed
                            pass
                    elif channel < ((2 * self._hexdrive_type.motors) + self._hexdrive_type.servos):
                        # Remaining channels are for servos (can be 4, 2 or 0 servos
                        self._freq[channel] = _DEFAULT_SERVO_FREQ
                        #print(f"D:{self.config.port}:Servo PWM[{channel}]")
                    else:
                        # ignore the remaining channels - we will switch them on when needed
                        pass
                if 0 < self._freq[channel]:
                    if self._set_pwmoutput(channel, 0):
                        self._stepper = False
                    else:
                        return False        
        self._pwm_setup = True
        return self._pwm_setup
    

    # De-initialise all PWM outputs
    def _pwm_deinit(self):
        for channel, pwm in enumerate(self.PWMOutput):
            if pwm is not None:
                try:
                    pwm.deinit()
                except Exception:       # pylint: disable=broad-except
                    pass
                self.PWMOutput[channel] = None
            self._freq[channel] = 0
            self._motor_output[(channel>>1)] = 0
        self._pwm_setup = False


    # are any of the PWM outputs energised?
    def _check_outputs_energised(self):
        energised_output = False
        for channel, pwm in enumerate(self.PWMOutput):
            if pwm is not None:
                try:
                    if 0 < pwm.duty_ns():
                        energised_output = True
                        break
                except Exception as e:        # pylint: disable=broad-except
                    print(self._pwm_log_string(channel) + f"Check failed {e}")
        if self._outputs_energised != energised_output:
            if self._logging:
                print(f"D:{self.config.port}:Outputs {'Energised' if energised_output else 'De-energised'}")
            self._outputs_energised = energised_output


    # Set a single PWM duty cycle (0-65535) for a specific output
    # if the channel has not been setup yet then we initialise it from scratch, otherwise we just change the duty cycle
    def _set_pwmoutput(self, channel: int, duty_cycle: int) -> bool:
        if duty_cycle < 0 or duty_cycle > 65535:
            return False   
        try:
            if self.PWMOutput[channel] is None:
                # Channel hasn't been setup yet so we need to initialise it from scratch
                self.PWMOutput[channel] = PWM(self.config.pin[channel], freq = self._freq[channel], duty_u16 = duty_cycle)
                if self._logging:
                    print(self._pwm_log_string(channel) + f"{self.PWMOutput[channel]} init")                
            elif duty_cycle != self.PWMOutput[channel].duty_u16():
                self.PWMOutput[channel].duty_u16(duty_cycle)
            if self._logging:
                print(self._pwm_log_string(channel) + f"{duty_cycle}")
        except Exception as e:              # pylint: disable=broad-except
            print(self._pwm_log_string(channel) + f"set {duty_cycle} failed {e}")
            return False
        return True


    def _check_port_for_hexdrive(self, port: int) -> HexDriveType | None:
        #just read the part of the header which contains the PID
        try:
            pid_bytes = self.config.i2c.readfrom_mem(_EEPROM_ADDR, _PID_ADDR, 2, addrsize = (8*_EEPROM_NUM_ADDRESS_BYTES))
        except OSError as e:      # pylint: disable=broad-except
            # no EEPROM on this port
            print(f"D:{port}:EEPROM error: {e}")
            return None
        # check the MSByte of PID for HexDrive Family
        if len(pid_bytes) < 2:
            return None
        if pid_bytes[1] != _HEXDRIVE_PID_MSB:
            return None
        # check if this is a HexDrive header by scanning the HEXDRIVE_TYPES list
        for _, hexpansion_type in enumerate(_HEXDRIVE_TYPES):
            if pid_bytes[0] == hexpansion_type.pid:
                return hexpansion_type
        # we are not interested in this type of hexpansion
        return None
    

    def _parse_version(self, version):
        """ Parse a version string, e.g. that of BadgeOS, into a list of components for comparison. Handles versions in the format v1.9.0-beta.1+build.123
            The version is split into components based on the delimiters '.' '-' and '+'."""
        #pre_components = ["final"]
        #build_components = ["0", "000000z"]
        #build = ""
        components = []
        if "+" in version:
            version, build = version.split("+", 1)          # pylint: disable=unused-variable
        #    build_components = build.split(".")
        if "-" in version:
            version, pre_release = version.split("-", 1)    # pylint: disable=unused-variable
        #    if pre_release.startswith("rc"):
        #        # Re-write rc as c, to support a1, b1, rc1, final ordering
        #        pre_release = pre_release[1:]
        #    pre_components = pre_release.split(".")
        version = version.strip("v").split(".")
        components = [int(item) if item.isdigit() else item for item in version]
        #components.append([int(item) if item.isdigit() else item for item in pre_components])
        #components.append([int(item) if item.isdigit() else item for item in build_components])
        return components


__app_export__ = HexDriveApp
