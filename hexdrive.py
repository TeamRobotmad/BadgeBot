"""HexDrive Hexpansion App for BadgeBot."""

# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import ota
from machine import PWM, Pin
from system.eventbus import eventbus
from system.scheduler.events import RequestStopAppEvent

import app

# HexDrive.py App Version - used to check if upgrade is required
HEXDRIVE_APP_VERSION = 6

# HexDrive Hexpansion constants
# Hardware defintions:
_ENABLE_PIN  = 0  # First LS pin used to enable the SMPSU
_DETECT_PIN  = 1  # Second LS pin used to sense if the SMPSU has a source of power

# Default values and limits:
_DEFAULT_PWM_FREQ = 20000           # 20kHz is a good default for motors as it is above the audible range for most people and works with most motors and ESCs
_DEFAULT_SERVO_FREQ = 50            # 50Hz = 20mS period
_DEFAULT_KEEP_ALIVE_PERIOD = 1000   # 1 second

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

class HexDriveType:
    """Represents a sub-type of HexDrive Hexpansion module."""
    __slots__ = ("pid", "name", "motors", "servos", "steppers")

    def __init__(self, pid_byte, motors=0, servos=0, steppers=0, name="Unknown"):
        self.pid = pid_byte         # Product ID byte read from the EEPROM to identify the type of HexDrive
        self.name = name            # A friendly name for the type of HexDrive
        self.motors = motors        # Number of motor channels supported by this type of HexDrive (0, 1 or 2)
        self.servos = servos        # Number of servo channels supported by this type of HexDrive (0, 2 or 4)
        self.steppers = steppers    # Number of stepper motors supported by this type of HexDrive (0 or 1)

_HEXDRIVE_TYPES = (
    HexDriveType(0xCA, motors=2, name="2 Motor"),
    HexDriveType(0xCB, motors=2, servos=4),
    HexDriveType(0xCC, servos=4, name="4 Servo"),
    HexDriveType(0xCD, motors=1, servos=2, name="1 Mot 2 Srvo"),
    HexDriveType(0xCE, steppers=1, name="1 Stepper"),
)

class HexDriveApp(app.App):         # pylint: disable=no-member
    """ HexDrive Hexpansion App for BadgeBot."""
    def __init__(self, config=None):
        super().__init__()
        self.config = config
        self._hexdrive_type = None
        self._logging = True
        self._keep_alive_period = _DEFAULT_KEEP_ALIVE_PERIOD
        self._power_state = None
        self._pwm_setup = False
        self._time_since_last_update = 0
        self._outputs_energised = False
        self.PWMOutput = [None] * 4
        self._freq = [0] * 4
        self._motor_output  = [0] * 2
        self._stepper = False
        if config is None:
            print("H:No Config!")
            return
        # LS Pins
        self._power_detect  = self.config.ls_pin[_DETECT_PIN]
        self._power_control = self.config.ls_pin[_ENABLE_PIN]

        self._servo_centre = [_SERVO_CENTRE] * 4
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)
        # What version of BadgeOS are we running on?
        try:
            ver = self._parse_version(ota.get_version())
            print(f"H:S/W {ver}")
            # e.g. v1.9.0-beta.1
            if ver >= [1, 9, 0]:
                # we need v1.9.0+ to be able to read the EEPROM with 16-bit addressing, so if we are running on an older version then we cannot continue
                pass
            else:
                print("H:BadgeOS Upgrade to v1.9.0+ required")
                return
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:Ver check failed {e}")
        self.initialise()

    def initialise(self) -> bool:
        """Initialise the app - return True if successful, False if failed."""
        self._pwm_setup = False
        if self.config is None:
            return False
        # report app starting and which port it is running on
        print(f"H:HexDrive V{HEXDRIVE_APP_VERSION} by RobotMad on port {self.config.port}")
        # Initialise HS Pins
        for _, hs_pin in enumerate(self.config.pin):
            # Set HexDrive Hexpansion HS pins to low level outputs
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)
        # Initialise LS Pins
        try:
            self._power_detect.init(mode=Pin.IN)
            self._power_control.init(mode=Pin.OUT)
        except Exception as e:      # pylint: disable=broad-except
            print(f"H:{self.config.port}:ls_pin setup failed {e}")
            return False
        # ensure SMPSU is turned off to start with
        self.set_power(False)

        # read hexpansion header from EEPROM to find out which sub-type we are
        # and allocate PWM outputs accordingly
        self._hexdrive_type = self._check_port_for_hexdrive(self.config.port)
        if self._logging and self._hexdrive_type is not None:
            print(f"H:{self.config.port}:Type:'{self._hexdrive_type.name}'")

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
                    print(f"H:{self.config.port}:Stop")
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
                    print(f"H:{self.config.port}:Timeout")                  
            if self._pwm_setup:
                for channel,pwm in enumerate(self.PWMOutput):
                    if pwm is not None:
                        try:
                            pwm.duty_u16(0)
                        except Exception as e:          # pylint: disable=broad-except
                            print(f"H:{self.config.port}:PWM[{channel}]:Off failed {e}")
                            self.PWMOutput[channel] = None  # Tidy Up
            elif self._stepper:
                self.motor_release()
            # we keep retriggering in case anything else has corrupted the PWM outputs


    def get_version(self) -> int:
        """ Get the version of the app - this is used to determine if an upgrade is required. """
        return HEXDRIVE_APP_VERSION


    def get_status(self) -> bool:
        """ Get the current status of the app - True if the app is running and able to respond to commands, False if not. """
        return (self._pwm_setup or self._stepper)


    def set_logging(self, state):
        """ Set the logging state - True to enable logging, False to disable logging. """
        self._logging = state


    def get_logging(self) -> bool:
        """ Get the current logging state - True if logging is enabled, False if logging is disabled. """
        return self._logging


    def set_power(self, state: bool) -> bool:
        """ Turn the SMPSU on or off. Returns the new power state.
            Note that just because the SMPSU is turned off does not mean that the outputs are NOT energised as there could be external battery power. """
        if (self.config is None) or (state == self._power_state):
            return False
        if self._logging:
            print(f"H:{self.config.port}:Power={'On' if state else 'Off'}")
        if self.get_booster_power():
            # if the power detect pin is high then the SMPSU has a power source so enable it
            try:
                self._power_control.init(mode=Pin.OUT)
                self._power_control.value(state)
            except Exception as e:      # pylint: disable=broad-except
                print(f"H:{self.config.port}:power control failed {e}")
                return False    
        self._power_state = state
        return self._power_state    


    def get_power(self) -> bool:
        """ Get the current state of the SMPSU enable pin. Returns True if enabled, False if disabled. """
        return self._power_state


    # Get the current state of the SMPSU power source
    def get_booster_power(self) -> bool:
        """ Get the current state of the SMPSU power source. Returns True if a power source is detected, False if not. """
        try:
            return self._power_detect.value()
        except Exception as e:          # pylint: disable=broad-except
            print(f"H:{self.config.port}:power detect failed {e}")
            return False


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
                    pwm.freq(int(freq))
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:{int(freq)}Hz")
                except Exception as e:  # pylint: disable=broad-except
                    print(f"H:{self.config.port}:PWM[{channel}]:set freq failed {e}")
                    return False
                self._freq[this_channel] = int(freq)
        return True
    

    def get_freq(self, channel: int = 0) -> int:
        """ Get the current PWM frequency for a specific output. Returns the frequency in Hz, or 0 if not available. """
        if not self._pwm_setup:
            return 0
        if channel < 0 or channel >= 4:
            return 0
        try:
            f = self.PWMOutput[channel].freq()
        except Exception:          # pylint: disable=broad-except
            f = 0
        return f
    


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
                for channel, pwm in enumerate(self.PWMOutput):
                    if pwm is not None:
                        try:
                            pwm.duty_ns(0)
                        except Exception as e:  # pylint: disable=broad-except
                            print(f"H:{self.config.port}:PWM[{channel}]:Off failed {e}")
                if self._logging:
                    print(f"H:{self.config.port}:PWM:[All]:Off")
                self._outputs_energised = False
                return True
            elif channel < 0 or channel >= 4:
                return False
            else:
                try:
                    self.PWMOutput[channel].duty_ns(0)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:Off")
                except Exception as e:          # pylint: disable=broad-except
                    print(f"H:{self.config.port}:PWM[{channel}]:Off failed {e}")
                    return False
            # check if all channels are now off and set outputs_energised accordingly
            self._check_outputs_energised()          
        else:           
            if channel < 0 or channel >= 4:
                return False            
            if abs(position) > _MAX_SERVO_RANGE:
                return False
            self._outputs_energised = True
            self._stepper = False                
            try:             
                if _MAX_SERVO_FREQ < self.PWMOutput[channel].freq():
                    # Ensure PWM frequency is suitable for use with Servos
                    # otherwise the pulse width will not be accepted
                    self.PWMOutput[channel].freq(_DEFAULT_SERVO_FREQ)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:{_DEFAULT_SERVO_FREQ}Hz for Servo")                    
            except Exception as e:          # pylint: disable=broad-except
                print(f"H:{self.config.port}:PWM[{channel}]:set freq failed {e}")
                return False
            # Scale servo position to PWM duty cycle (500-2500us)
            pulse_width = int((self._servo_centre[channel] + position) * 1000)
            try:
                if pulse_width != self.PWMOutput[channel].duty_ns():
                    self.PWMOutput[channel].duty_ns(pulse_width)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:{pulse_width//1000}us")
            except Exception as e:          # pylint: disable=broad-except
                print(f"H:{self.config.port}:PWM[{channel}]:set pwm failed {e}")
                return False
        self._time_since_last_update = 0
        return True


    def set_servocentre(self, centre: int, channel: int | None = None) -> bool:
        """ Set the centre position for a specific servo output, or all servo outputs if channel is None. Returns True if successful, False if failed.
            Note this does not change the current position of the servo.
            It will only affect the position next time it is set.
            You can use this to trim the centre position of the servo. """
        if not self._pwm_setup:
            return False
        if channel is not None and (channel < 0 or channel >= 4):
            return False
        if centre < (_SERVO_CENTRE - _SERVO_MAX_TRIM ) or centre > (_SERVO_CENTRE + _SERVO_MAX_TRIM): 
            return False
        if channel is None:
            self._servo_centre = [centre] * 4
        else:
            self._servo_centre[channel] = centre
        return True


    # Set pairs of PWM duty cycles in one go using a signed value per motor channel (0-65535)
    def set_motors(self, outputs) -> bool:
        """ Set the motor outputs using a signed value for each motor channel. Returns True if successful, False if failed.
            The outputs are signed values in a tuple from -65535 to 65535 which are scaled to the PWM duty cycle range of 0-65535.
            A positive value will drive the motor in one direction, a negative value will drive it in the opposite direction,
            and a value of 0 will stop the motor. """
        if not self._pwm_setup:
            return False
        for motor, output in enumerate(outputs):
            if abs(output) > 65535:
                return False
            try:
                if output >= 0:
                    if 0 > self._motor_output[motor]:
                        # switch which signal is being driven as the PWM output
                        self.PWMOutput[(motor<<1)+1].deinit()
                        self.PWMOutput[(motor<<1)+1] = None
                        self.config.pin[(motor<<1)+1].value(0)
                        self.PWMOutput[(motor<<1)] = PWM(self.config.pin[(motor<<1)], freq = self._freq[(motor<<1)], duty_u16 = int(output))
                        if self._logging:
                            print(f"H:{self.config.port}:<>PWM[{(motor<<1)}]:{output}")                      
                    else:
                        self._set_pwmoutput((motor<<1), int(output))                  
                else:
                    if 0 <= self._motor_output[motor]:
                        # switch which signal is being driven as the PWM output
                        self.PWMOutput[(motor<<1)].deinit()
                        self.PWMOutput[(motor<<1)] = None
                        self.config.pin[(motor<<1)].value(0)
                        self.PWMOutput[(motor<<1)+1] = PWM(self.config.pin[(motor<<1)+1], freq = self._freq[(motor<<1)], duty_u16 = -int(output))
                        if self._logging:
                            print(f"H:{self.config.port}:<>PWM[{(motor<<1)+1}]:{-int(output)}")                            
                    else:        
                        self._set_pwmoutput((motor<<1)+1, -int(output))
            except Exception as e:          # pylint: disable=broad-except
                print(f"H:{self.config.port}:Motor{motor}:{output} set failed {e}")   
            self._motor_output[motor] = output
        self._check_outputs_energised()
        self._time_since_last_update = 0
        return True


    # Set all 4 PWM duty cycles in one go using a tuple (0-65535)
    def set_pwm(self, duty_cycles) -> bool:
        """ Set the PWM duty cycle for all outputs at once using a tuple of values. Returns True if successful, False if failed.
            The duty_cycles are values from 0 to 65535. """
        if not self._pwm_setup:
            return False
        self._outputs_energised = any(duty_cycles)
        for channel, duty_cycle in enumerate(duty_cycles): 
            if not self._set_pwmoutput(channel, int(duty_cycle)):
                return False
        self._time_since_last_update = 0
        return True


    # Get the current PWM duty cycle for a specific output (0-65535)
    def get_pwm(self, channel: int = 0) -> int:
        """ Get the current PWM duty cycle for a specific output. Returns the duty cycle as a value from 0 to 65535, or 0 if not available. """
        if not self._pwm_setup:
            return 0
        if channel >= len(self.PWMOutput):
            return 0
        try:
            pwm = self.PWMOutput[channel].duty_u16()
        except Exception:       # pylint: disable=broad-except
            pwm = 0
        return pwm

### Stepper Motor Support
    def motor_step(self, phase: int) -> int | None:
        """ Step the motor to a specific phase in the stepping sequence. Returns None if failed (e.g. invalid phase or not configured for stepper),
            otherwise returns the phase that was set. The phase is a value from 0 to _STEPPER_NUM_PHASES-1 which corresponds to the 
            stepping sequence defined in _STEPPER_SEQUENCE."""
        if phase >= _STEPPER_NUM_PHASES:
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
        for channel in range(4):
            self.config.pin[channel].value(0)
        self._outputs_energised = False
        self._time_since_last_update = 0


    def _pwm_init(self) -> bool:
        self._pwm_setup = False
        # HS Pins
        if self.config.pin is not None and len(self.config.pin) == 4:             
            # Allocate PWM generation to pins
            for channel, hs_pin in enumerate(self.config.pin):
                self._freq[channel] = 0             
                if self._hexdrive_type is not None:
                    if channel < (2 * self._hexdrive_type.motors):
                        # First channels are for motors (can be 0, 1 or 2 motors)
                        if 0 == channel % 2:
                            # initialise motor PWM output on even channel
                            self._motor_output[(channel>>1)] = 0
                            self._freq[channel] = _DEFAULT_PWM_FREQ
                            #print(f"H:{self.config.port}:Motor PWM[{channel}]")
                        else:
                            # ignore the motor PWM output on odd channel - we will switch it on when needed
                            pass
                    elif channel < ((2 * self._hexdrive_type.motors) + self._hexdrive_type.servos):
                        # Remaining channels are for servos (can be 4, 2 or 0 servos
                        self._freq[channel] = _DEFAULT_SERVO_FREQ
                        #print(f"H:{self.config.port}:Servo PWM[{channel}]")
                    else:
                        # ignore the remaining channels - we will switch them on when needed
                        pass
                if 0 < self._freq[channel]:        
                    try:
                        self.PWMOutput[channel] = PWM(hs_pin, freq = self._freq[channel], duty_u16 = 0)
                        self._stepper = False
                        if self._logging:
                            print(f"H:{self.config.port}:PWM[{channel}]:{self.PWMOutput[channel]}")
                    except Exception as e:      # pylint: disable=broad-except
                        # There are a finite number of PWM resources so it is possible that we run out
                        print(f"H:{self.config.port}:PWM[{channel}]:PWM(init) failed {e}")
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
                    print(f"H:{self.config.port}:PWM[{channel}]:Check failed {e}")
        if self._outputs_energised != energised_output:
            if self._logging:
                print(f"H:{self.config.port}:Outputs {'Energised' if energised_output else 'De-energised'}")
            self._outputs_energised = energised_output


    # Set a single PWM duty cycle (0-65535) for a specific output
    def _set_pwmoutput(self, channel: int, duty_cycle: int) -> bool:
        if duty_cycle < 0 or duty_cycle > 65535:
            return False   
        try:
            if duty_cycle != self.PWMOutput[channel].duty_u16():
                self.PWMOutput[channel].duty_u16(duty_cycle)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{channel}]:{duty_cycle}")
        except Exception as e:              # pylint: disable=broad-except
            print(f"H:{self.config.port}:PWM[{channel}]:set {duty_cycle} failed {e}")
            return False
        return True


    def _check_port_for_hexdrive(self, port: int) -> HexDriveType | None:
        #just read the part of the header which contains the PID
        try:
            pid_bytes = self.config.i2c.readfrom_mem(_EEPROM_ADDR, _PID_ADDR, 2, addrsize = (8*_EEPROM_NUM_ADDRESS_BYTES))
        except OSError as e:      # pylint: disable=broad-except
            # no EEPROM on this port
            print(f"H:{port}:EEPROM error: {e}")
            return None
        # check the MSByte of PID for HexDrive Family
        if len(pid_bytes) < 2:
            return None
        if pid_bytes[1] != 0xCB:
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
