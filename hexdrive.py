# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import asyncio

from machine import I2C, PWM
from system.eventbus import eventbus
from system.scheduler.events import RequestStopAppEvent

import app

# HexDrive.py App Version - used to check if upgrade is required
APP_VERSION = 4 


_ENABLE_PIN = 0	  # First LS pin used to enable the SMPSU
_DETECT_PIN = 1   # Second LS pin used to sense if the SMPSU has a source of power

_DEFAULT_PWM_FREQ = 20000    
_DEFAULT_SERVO_FREQ = 50            # 20mS period
_MAX_SERVO_FREQ = 200               # 5mS period (can work with some Servos but not all)
_DEFAULT_KEEP_ALIVE_PERIOD = 1000   # 1 second
_SERVO_CENTRE_NS = 1500000          # 1500us

_SYSTEM_I2C_BUS = 7

class HexDriveApp(app.App):

    def __init__(self, config=None):
        self.config = config
        self._logging = True
        self.keep_alive_period = _DEFAULT_KEEP_ALIVE_PERIOD
        self.power_state = None
        self.pwm_setup_failed = True
        self.time_since_last_update = 0
        self.outputs_energised = False
        self.PWMOutput = [None] * 4
        self.power_detect = self.config.ls_pin[_DETECT_PIN]
        self.power_control = self.config.ls_pin[_ENABLE_PIN]       
        self._servo_centre_ns = [_SERVO_CENTRE_NS] * 4
        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)

        self.initialise()

    def initialise(self) -> bool:
        self.pwm_setup_failed = True
        if self.config is None:
            return False        
        # report app starting and which port it is running on
        print(f"HexDrive V{APP_VERSION} by RobotMad on port {self.config.port}")
        # Set Power Detect Pin to Input and Power Enable Pin to Output
        self._set_pin_direction(self.power_detect.pin,  1)  # input
        self._set_pin_direction(self.power_control.pin, 0)  # output
        self.set_power(False)
        # Set all HexDrive Hexpansion HS pins to low level outputs
        for hs_pin in self.config.pin:
            hs_pin.value(0)    
        if self.config.pin is not None and len(self.config.pin) == 4:
            # Allocate PWM generation to pins
            for channel, hs_pin in enumerate(self.config.pin):
                try:
                    self.PWMOutput[channel] = PWM(hs_pin, freq = _DEFAULT_PWM_FREQ, duty_u16 = 0)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:{self.PWMOutput[channel]}")
                except:
                    # There are a finite number of PWM resources so it is possible that we run out
                    print(f"H:{self.config.port}:PWM[{channel}]: allocation failed")
                    return False
            self.pwm_setup_failed = False
        return not self.pwm_setup_failed


    def deinitialise(self) -> bool:
        # Turn off all PWM outputs & release resources
        for channel, pwm in enumerate(self.PWMOutput):
            pwm.deinit()
            self.PWMOutput[channel] = None
        self.set_power(False)
        for hs_pin in self.config.pin:
            hs_pin.value(0)          
        return True


    # Handle the RequestStopAppEvent so that ew can release resources
    async def _handle_stop_app(self, event):
        if event.app == self:
            if self._logging:
                print(f"H:{self.config.port}:Stopping HexDrive App & Releasing PWM resources")
            self.deinitialise()


    # Check keep alive period and turn off PWM outputs if exceeded
    def background_update(self, delta):
        if (self.config is None) or self.pwm_setup_failed:
            return
        self.time_since_last_update += delta
        if self.time_since_last_update > self.keep_alive_period:
            self.time_since_last_update = 0
            for pwm in enumerate(self.PWMOutput):
                try:
                    pwm.duty_u16(0)
                except:
                    pass
            if self.outputs_energised:
                self.outputs_energised = False
                # First time the keep alive period has expired so report it
                if self._logging:
                    print(f"H:{self.config.port}:Keep Alive Timeout")            
            # we keep retriggering in case anything else has corrupted the PWM outputs


    def get_version(self) -> int:
        return APP_VERSION
    
    # Get the current status of the HexDrive App
    def get_status(self) -> bool:
        return not self.pwm_setup_failed


    # Set the logging state
    def set_logging(self, state):
        self._logging = state


    # Get the current logging state
    def get_logging(self) -> bool:
        return self._logging


    # Turn the SMPPSU on or off
    # Just because the SPMSU is turned off does not mean that the outputs are NOT energised
    # as there could be external battery power
    def set_power(self, state) -> bool:
        if (self.config is None) or (state == self.power_state):
            return False
        if self._logging:
            print(f"H:{self.config.port}:Power={'On' if state else 'Off'}")
        if self.get_booster_power():
            # if the power detect pin is high then the SMPSU has a power source so enable it
            self._set_pin_state(self.power_control.pin, state)
            self._set_pin_direction(self.power_control.pin, 0)  # in case it gets corrupted by other code
        self.power_state = state
        return self.power_state    


    # Get the current state of the SMPSU enable pin
    def get_power(self) -> bool:
        return self.power_state


    # Get the current state of the SMPSU power source
    def get_booster_power(self) -> bool:
        return self._get_pin_state(self.power_detect.pin)


    # Set the keep alive period - this is the time in milli-seconds that the PWM outputs will be kept on
    def set_keep_alive(self, period):
        self.keep_alive_period = period

    
    # Only one PWM frequency (in Hz) is supported for all outputs due to timer limitations
    # Use 50 to 200 for Servos and 5000 to 20000 for motors
    def set_freq(self, freq) -> bool:
        if self.pwm_setup_failed:
            return False
        for channel, pwm in enumerate(self.PWMOutput):
            try:
                pwm.freq(int(freq))
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{channel}] freq: {int(freq)}Hz")
            except:
                print(f"H:{self.config.port}:PWM[{channel}] freq: set {int(freq)}Hz failed")
                return False
        return True
    

    # Get the current PWM frequency for a specific output
    def get_freq(self, channel=0) -> int:
        if self.pwm_setup_failed:
            return 0
        if channel < 0 or channel >= 4:
            return 0
        return self.PWMOutput[int(channel)].freq()
    

    # set the pulse width for a specific servo output
    # position is the offset (in us) from the centre position of the servo
    # Based on standard RC servos with centre at 1500us and range of 1000-2000us
    # The position is a signed value from -1000 to 1000 which is scaled to 500-2500us
    # This is a very wide range and may not be suitable for all servos, some will 
    # only be happy with 1000-2000us (i.e. position in the range -500 to 500)
    def set_servoposition(self, channel=None, position=None) -> bool:
        if self.pwm_setup_failed:
            return False
        if position is None:
            # position == None -> Turn off PWM (some servos will then turn off, others will stay in last position)
            if channel is None:
                # channel == None -> Turn off all PWM outputs
                for channel in self.PWMOutput:
                    try:
                        channel.duty_ns(0)
                    except:
                        pass
                if self._logging:
                    print(f"H:{self.config.port}:PWM:All Off")
                return True
            elif channel < 0 or channel >= 4:
                return False
            try:
                self.PWMOutput[int(channel)].duty_ns(0)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{int(channel)}]:Off")
            except:
                print(f"H:{self.config.port}:PWM[{int(channel)}]:Off failed")
                return False            
        else:           
            if channel < 0 or channel >= 4:
                return False            
            if abs(position) > 1000:
                return False
            try:             
                if _MAX_SERVO_FREQ < self.PWMOutput[int(channel)].freq():
                    # Ensure PWM frequency is suitable for use with Servos
                    # otherwise the pulse width will not be accepted
                    self.PWMOutput[int(channel)].freq(_DEFAULT_SERVO_FREQ)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{channel}]:Force freq to {_DEFAULT_SERVO_FREQ}Hz for Servo")                    
            except:
                print(f"H:{self.config.port}:PWM[{channel}]:freq set to {_DEFAULT_SERVO_FREQ} failed")
                return False
            # Scale servo position to PWM duty cycle (500-2500us)
            pulse_width = int(self._servo_centre_ns[channel] + (position * 1000))
            try:
                if pulse_width != self.PWMOutput[int(channel)].duty_ns():
                    self.PWMOutput[int(channel)].duty_ns(pulse_width)
                    if self._logging:
                        print(f"H:{self.config.port}:PWM[{int(channel)}]:{pulse_width//1000}us")
            except:
                print(f"H:{self.config.port}:PWM[{int(channel)}]:{position} set failed")
                return False
        self.time_since_last_update = 0
        return True

    # Set the centre position for a specific servo output
    # Note this does not change the current position of the servo
    # it will only affect the position next time it is set
    # you can use this to trim the centre position of the servo
    def set_servocentre(self, centre, channel=None) -> bool:
        if self.pwm_setup_failed:
            return False
        if channel is not None and (channel < 0 or channel >= 4):
            return False
        if centre < 500 or centre > 2500:
            return False
        if channel is None:
            self._servo_centre_ns = [int(centre * 1000)] * 4
        else:    
            self._servo_centre_ns[int(channel)] = int(centre * 1000)
        return True
    

    # Set pairs of PWM duty cycles in one go using a signed value per motor channel (0-65535)
    def set_motors(self, outputs) -> bool:
        if self.pwm_setup_failed:
            return False
        self.outputs_energised = any(outputs)
        for motor, output in enumerate(outputs):
            if abs(output) > 65535:
                return False
            pwmA =  int(output) if output > 0 else 0
            pwmB = -int(output) if output < 0 else 0
            if not self._set_pwmoutput(motor<<1, pwmA) or not self._set_pwmoutput((motor<<1)+1, pwmB):
                return False
        self.time_since_last_update = 0
        return True


    # Set all 4 PWM duty cycles in one go using a tuple (0-65535)
    def set_pwm(self, duty_cycles) -> bool:
        if self.pwm_setup_failed:
            return False
        self.outputs_energised = any(duty_cycles)
        for channel, duty_cycle in enumerate(duty_cycles): 
            if not self._set_pwmoutput(channel, int(duty_cycle)):
                return False
        self.time_since_last_update = 0
        return True


    # Get the current PWM duty cycle for a specific output (0-65535)
    def get_pwm(self, channel=0) -> int:
        if self.pwm_setup_failed:
            return 0
        if channel >= len(self.PWMOutput):
            return 0
        return self.PWMOutput[channel].duty_u16()


    # Set a single PWM duty cycle (0-65535) for a specific output
    def _set_pwmoutput(self, channel, duty_cycle) -> bool:
        if duty_cycle < 0 or duty_cycle > 65535:
            return False   
        try:
            if duty_cycle != self.PWMOutput[channel].duty_u16():
                self.PWMOutput[channel].duty_u16(duty_cycle)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{channel}]:{duty_cycle}")
        except:
            print(f"H:{self.config.port}:PWM[{channel}]:set {duty_cycle} failed")
            return False
        return True


    # Set the state of a specific LS output pin
    def _set_pin_state(self, pin, state):
        try:
            i2c = I2C(_SYSTEM_I2C_BUS)
            output_reg = i2c.readfrom_mem(pin[0], 0x02+pin[1], 1)[0]
            output_reg = (output_reg | pin[2]) if state else (output_reg & ~pin[2])
            i2c.writeto_mem(pin[0], 0x02+pin[1], bytes([output_reg]))
            #if self._logging:
            #    print(f"H:{self.config.port}:Write to {hex(pin[0])} address {hex(0x02+pin[1])} value {hex(output_reg)}")
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")


    # Get the state of a specific LS input pin
    def _get_pin_state(self, pin) -> bool:
        try:
            i2c = I2C(_SYSTEM_I2C_BUS)
            input_reg = i2c.readfrom_mem(pin[0], 0x00+pin[1], 1)[0]
            return (input_reg & pin[2]) != 0
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")
            return False


    # Set the direction of a specific LS pin
    def _set_pin_direction(self, pin, direction):
        try:
            # Use a Try in case access to i2C(7) is blocked for apps in future
            # presumably if this happens then the code will have been updated to
            # handle the GPIO direction correctly anyway.
            i2c = I2C(_SYSTEM_I2C_BUS)
            config_reg = i2c.readfrom_mem(pin[0], 0x04+pin[1], 1)[0]
            config_reg = (config_reg | pin[2]) if (1 == direction) else (config_reg & ~pin[2])
            i2c.writeto_mem(pin[0], 0x04+pin[1], bytes([config_reg]))
            #if self._logging:
            #    print(f"H:{self.config.port}:Write to {hex(pin[0])} address {hex(0x04+pin[1])} value {hex(config_reg)}")
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")
    
__app_export__ = HexDriveApp
