# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import asyncio
import ota
from machine import PWM, Pin
from system.eventbus import eventbus
from system.scheduler.events import RequestStopAppEvent

import app

# HexDrive.py App Version - used to check if upgrade is required
APP_VERSION = 6 


_ENABLE_PIN  = 0  # First LS pin used to enable the SMPSU
_DETECT_PIN  = 1  # Second LS pin used to sense if the SMPSU has a source of power

_DEFAULT_PWM_FREQ = 20000    
_DEFAULT_SERVO_FREQ = 50            # 20mS period
_MAX_SERVO_FREQ = 200               # 5mS period (can work with some Servos but not all)
_DEFAULT_KEEP_ALIVE_PERIOD = 1000   # 1 second
_SERVO_CENTRE    = 1500             # 1500us
_MAX_SERVO_RANGE = 1400             # 1400us either side of centre (VERY WIDE)
_SERVO_MAX_TRIM  = 1000             # us

_STEPPER_NUM_PHASES    = 8

_EEPROM_ADDR  = 0x50
_EEPROM_NUM_ADDRESS_BYTES = 2
_PID_ADDR     = 0x12

class HexDriveApp(app.App):

    def __init__(self, config=None):
        super().__init__()
        self.config = config        
        self._logging = True
        self._HEXDRIVE_TYPES = [HexDriveType(0xCB, motors=2, servos=4), 
                                HexDriveType(0xCA, motors=2, name="2 Motor"), 
                                HexDriveType(0xCC, servos=4, name="4 Servo"), 
                                HexDriveType(0xCD, motors=1, servos=2, name = "1 Mot 2 Srvo"),
                                HexDriveType(0xCE, steppers=1, name = "Stepper")]
        self._hexdrive_type_index = None
        self._keep_alive_period = _DEFAULT_KEEP_ALIVE_PERIOD
        self._power_state = None
        self._pwm_setup = False
        self._time_since_last_update = 0
        self._outputs_energised = False
        self.PWMOutput = [None] * 4
        self._freq = [0] * 4
        self._motor_output  = [0] * 2
        # define the stepping sequence for a 8-phase stepper motor as a list of 4-tuples
        self._step = [(1,0,1,0), (0,0,1,0), (0,1,1,0), (0,1,0,0), (0,1,0,1), (0,0,0,1), (1,0,0,1), (1,0,0,0)]        
        self._stepper = False
        if config is None:
            print("H:No Config!")
            return
        # LS Pins
        self._power_detect  = self.config.ls_pin[_DETECT_PIN]
        self._power_control = self.config.ls_pin[_ENABLE_PIN]

        self._servo_centre = [_SERVO_CENTRE] * 4
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)
        try:
            ver = self._parse_version(ota.get_version())
            print(f"H:S/W {ver}")
            # e.g. v1.9.0-beta.1
            if ver >= [1, 9, 0]:
                pass
            else:
                print(f"H:BadgeOS Upgrade to v1.9.0+ required")
                return
        except Exception as e:
            print(f"H:Ver check failed {e}")
        self.initialise()


    def initialise(self) -> bool:
        self._pwm_setup = False
        if self.config is None:
            return False        
        # report app starting and which port it is running on
        print(f"H:HexDrive V{APP_VERSION} by RobotMad on port {self.config.port}")
        # HS Pins
        for _, hs_pin in enumerate(self.config.pin):
            # Set HexDrive Hexpansion HS pins to low level outputs
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)            
        # LS Pins
        try:
            self._power_detect.init(mode=Pin.IN)
            self._power_control.init(mode=Pin.OUT)
        except Exception as e:
            print(f"H:{self.config.port}:ls_pin setup failed {e}")
            return False
        self.set_power(False)
    
        # read hexpansion header from EEPROM to find out which type we are
        # and allocate PWM outputs accordingly
        self._hexdrive_type_index = self._check_port_for_hexdrive(self.config.port)
        if self._logging and self._hexdrive_type_index is not None:
            print(f"H:{self.config.port}:Type:'{self._HEXDRIVE_TYPES[self._hexdrive_type_index]._name}'")

        return(self._pwm_init())


    def deinitialise(self) -> bool:
        # Turn off all PWM outputs & release resources
        self.set_power(False)
        self._pwm_deinit() 
        for hs_pin in self.config.pin:
            hs_pin.init(mode=Pin.OUT)
            hs_pin.value(0)                  
        return True


    # Handle the RequestStopAppEvent so that we can release resources
    async def _handle_stop_app(self, event):
        try:
            if event.app == self:
                if self._logging:
                    print(f"H:{self.config.port}:Stop")
                self.deinitialise()
        except:
            pass      


    # Check keep alive period and turn off PWM outputs if exceeded
    def background_update(self, delta: int):
        if (self.config is None) or not (self._pwm_setup or self._stepper):
            return
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
                        except Exception as e:
                            print(f"H:{self.config.port}:PWM[{channel}]:Off failed {e}")
                            self.PWMOutput[channel] = None  # Tidy Up
            elif self._stepper:
                self.motor_release()
      
            # we keep retriggering in case anything else has corrupted the PWM outputs


    def get_version(self) -> int:
        return APP_VERSION
    

    # Get the current status of the HexDrive App
    def get_status(self) -> bool:
        return self._pwm_setup


    # Set the logging state
    def set_logging(self, state):
        self._logging = state


    # Get the current logging state
    def get_logging(self) -> bool:
        return self._logging


    # Turn the SMPPSU on or off
    # Just because the SPMSU is turned off does not mean that the outputs are NOT energised
    # as there could be external battery power
    def set_power(self, state: bool) -> bool:
        if (self.config is None) or (state == self._power_state):
            return False
        if self._logging:
            print(f"H:{self.config.port}:Power={'On' if state else 'Off'}")
        if self.get_booster_power():
            # if the power detect pin is high then the SMPSU has a power source so enable it
            try:
                self._power_control.init(mode=Pin.OUT)
                self._power_control.value(state)
            except Exception as e:
                print(f"H:{self.config.port}:power control failed {e}")
                return False    
        self._power_state = state
        return self._power_state    


    # Get the current state of the SMPSU enable pin
    def get_power(self) -> bool:
        return self._power_state


    # Get the current state of the SMPSU power source
    def get_booster_power(self) -> bool:
        try:
            return self._power_detect.value()
        except Exception as e:
            print(f"H:{self.config.port}:power detect failed {e}")
            return False


    def set_keep_alive(self, period: int):
        self._keep_alive_period = period

    
    # Use 50 to 200 for Servos and 5000 to 20000 for motors
    def set_freq(self, freq: int, channel: int | None = None) -> bool:
        if not self._pwm_setup:
            return False
        for this_channel, pwm in enumerate(self.PWMOutput):
            if (channel is None or this_channel == channel) and pwm is not None:
                try:
                    pwm.freq(int(freq))
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:{int(freq)}Hz")
                except Exception as e:
                    print(f"H:{self.config.port}:PWM[{channel}]:set freq failed {e}")
                    return False
                self._freq[this_channel] = int(freq)
        return True
    

    # Get the current PWM frequency for a specific output
    def get_freq(self, channel: int = 0) -> int:
        if not self._pwm_setup:
            return 0
        if channel < 0 or channel >= 4:
            return 0
        try:
            f = self.PWMOutput[channel].freq()
        except:
            f = 0
        return f
    

    # set the pulse width for a specific servo output
    # position is the offset (in us) from the centre position of the servo
    # Based on standard RC servos with centre at 1500us and range of 1000-2000us
    # The position is a signed value from -1000 to 1000 which is scaled to 500-2500us
    # This is a very wide range and may not be suitable for all servos, some will 
    # only be happy with 1000-2000us (i.e. position in the range -500 to 500)
    def set_servoposition(self, channel: int | None = None, position: int | None = None) -> bool:
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
                        except:
                            pass
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
                except Exception as e:
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
            except Exception as e:
                print(f"H:{self.config.port}:PWM[{channel}]:set freq failed {e}")
                return False
            # Scale servo position to PWM duty cycle (500-2500us)
            pulse_width = int((self._servo_centre[channel] + position) * 1000)
            try:
                if pulse_width != self.PWMOutput[channel].duty_ns():
                    self.PWMOutput[channel].duty_ns(pulse_width)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:{pulse_width//1000}us")
            except Exception as e:
                print(f"H:{self.config.port}:PWM[{channel}]:set pwm failed {e}")
                return False
        self._time_since_last_update = 0
        return True


    # Set the centre position for a specific servo output
    # Note this does not change the current position of the servo
    # it will only affect the position next time it is set
    # you can use this to trim the centre position of the servo
    def set_servocentre(self, centre: int, channel: int | None = None) -> bool:
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
            except Exception as e:
                print(f"H:{self.config.port}:Motor{motor}:{output} set failed {e}")    
            self._motor_output[motor] = output
        self._check_outputs_energised()
        self._time_since_last_update = 0
        return True


    # Set all 4 PWM duty cycles in one go using a tuple (0-65535)
    def set_pwm(self, duty_cycles) -> bool:
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
        if not self._pwm_setup:
            return 0
        if channel >= len(self.PWMOutput):
            return 0
        try:
            pwm = self.PWMOutput[channel].duty_u16()
        except:
            pwm = 0
        return pwm

## Stepper Motor Support
    # Stepper Motor Support - force output to a specific phase
    def motor_step(self, phase: int):
        if phase >= _STEPPER_NUM_PHASES:
            return None
        if not self._stepper:
            # not currently configured for stepper motor - configure
            self._pwm_deinit() 
            self._stepper = True
        for channel, value in enumerate(self._step[phase]):
            self.config.pin[channel].value(value)
        self._outputs_energised = True  
        self._time_since_last_update = 0

    def motor_release(self):
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
                if self._hexdrive_type_index is not None:
                    if channel < (2 * self._HEXDRIVE_TYPES[self._hexdrive_type_index]._motors):
                        # First channels are for motors (can be 0, 1 or 2 motors)
                        if 0 == channel % 2:
                            # initialise motor PWM output on even channel
                            self._motor_output[(channel>>1)] = 0
                            self._freq[channel] = _DEFAULT_PWM_FREQ
                            #print(f"H:{self.config.port}:Motor PWM[{channel}]")
                        else:
                            # ignore the motor PWM output on odd channel - we will switch it on when needed
                            pass
                    elif channel < ((2 * self._HEXDRIVE_TYPES[self._hexdrive_type_index]._motors) + self._HEXDRIVE_TYPES[self._hexdrive_type_index]._servos):
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
                    except Exception as e:
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
                except:
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
                except Exception as e:
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
        except Exception as e:
            print(f"H:{self.config.port}:PWM[{channel}]:set {duty_cycle} failed {e}")
            return False
        return True


    def _check_port_for_hexdrive(self, port: int) -> int:
        #just read the part of the header which contains the PID
        try:
            pid_bytes = self.config.i2c.readfrom_mem(_EEPROM_ADDR, _PID_ADDR, 2, addrsize = (8*_EEPROM_NUM_ADDRESS_BYTES))
        except OSError as e:
            # no EEPROM on this port
            print(f"H:{port}:EEPROM error: {e}")
            return None
        # check the MSByte of PID for HexDrive Family
        if len(pid_bytes) < 2:
            return None
        if pid_bytes[1] != 0xCB:
            return None
        # check if this is a HexDrive header by scanning the _HEXDRIVE_TYPES list
        for index, hexpansion_type in enumerate(self._HEXDRIVE_TYPES):
            if pid_bytes[0] == hexpansion_type.pid_byte:
                return index
        # we are not interested in this type of hexpansion
        return None
    

    def _parse_version(self, version):
        #pre_components = ["final"]
        #build_components = ["0", "000000z"]
        #build = ""
        components = []
        if "+" in version:
            version, build = version.split("+", 1)
        #    build_components = build.split(".")
        if "-" in version:
            version, pre_release = version.split("-", 1)
        #    if pre_release.startswith("rc"):
        #        # Re-write rc as c, to support a1, b1, rc1, final ordering
        #        pre_release = pre_release[1:]
        #    pre_components = pre_release.split(".")
        version = version.strip("v").split(".")
        components = [int(item) if item.isdigit() else item for item in version]
        #components.append([int(item) if item.isdigit() else item for item in pre_components])
        #components.append([int(item) if item.isdigit() else item for item in build_components])
        return components


      
class HexDriveType:
    def __init__(self, pid_byte: int , motors: int = 0, servos: int = 0, steppers: int = 0, name="Unknown"):
        self.pid_byte = pid_byte
        self._name     = name
        self._motors   = motors
        self._servos   = servos
        self._steppers = steppers


__app_export__ = HexDriveApp
