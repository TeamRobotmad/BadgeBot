# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py/mpy
# It is then run from the EEPROM by the BadgeOS.

import asyncio

from machine import I2C, PWM
from system.eventbus import eventbus
from system.scheduler.events import RequestStopAppEvent

import app

# HexDrive.py App Version - parsed by app.py to check if upgrade is required
APP_VERSION = 3 

_ENABLE_PIN = 0	  # First LS pin used to enable the SMPSU
_DETECT_PIN = 1   # Second LS pin used to sense if the SMPSU has a source of power

_DEFAULT_PWM_FREQ = 20000    
_DEFAULT_KEEP_ALIVE_PERIOD = 1000  # 1 second

class HexDriveApp(app.App):

    def __init__(self, config=None):
        self.config = config
        self._logging = True
        self.keep_alive_period = _DEFAULT_KEEP_ALIVE_PERIOD
        self.power_state = None
        self.pwm_setup_failed = True
        self.time_since_last_update = 0
        self.outputs_energised = False
        self.PWMOutput = [None] * len(self.config.pin)
        self.power_detect = self.config.ls_pin[_DETECT_PIN]
        self.power_control = self.config.ls_pin[_ENABLE_PIN]       

        eventbus.on_async(RequestStopAppEvent, self._handle_stop_app, self)

        self.initialise()

    def initialise(self) -> bool:
        if self.config is None:
            return False        
        # report app starting and which port it is running on
        print(f"HexDrive App on port {self.config.port}")
        # Set Power Detect Pin to Input and Power Enable Pin to Output
        self._set_pin_direction(self.power_detect.pin,  1)
        self._set_pin_direction(self.power_control.pin, 0)  
        self.set_power(False)
        # Set all HexDrive Hexpansion HS pins to low level outputs
        for hs_pin in self.config.pin:
            hs_pin.value(0)    
        # Allocate PWM generation to pins
        for i_num, hs_pin in enumerate(self.config.pin):
            try:
                self.PWMOutput[i_num] = PWM(hs_pin, freq = _DEFAULT_PWM_FREQ, duty_u16 = 0)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{i_num}]:{self.PWMOutput[i_num]}")
            except:
                # There are a finite number of PWM resources so it is possible that we run out
                print(f"H:{self.config.port}:PWM[{i_num}]:PWM allocation failed")
                return False
        self.pwm_setup_failed = False
        return not self.pwm_setup_failed


    def deinitialise(self) -> bool:
        # Turn off all PWM outputs & release resources
        for i, pwm in enumerate(self.PWMOutput):
            pwm.deinit()
            self.PWMOutput[i] = None
        self.set_power(False)
        for hs_pin in self.config.pin:
            hs_pin.value(0)          
        return True


    # Handle the RequestStopAppEvent so that ew can release resources
    async def _handle_stop_app(self, event):
        if event.app == self:
            if self._logging:
                print(f"H:{self.config.port}:Stopping HexDrive App")
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


    # Get the current status of the HexDrive App
    def get_status(self) -> bool:
        return not self.pwm_setup_failed


    # Set the logging state
    def set_logging(self, state):
        self._logging = state

    
    # Turn the SPMPSU on or off
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


    # Get the current state of the SMPSU power source
    def get_booster_power(self) -> bool:
        return self._get_pin_state(self.power_detect.pin)


    # Set the keep alive period - this is the time in milli-seconds that the PWM outputs will be kept on
    def set_keep_alive(self, period):
        self.keep_alive_period = period

    
    # Only one PWM frequency (in Hz) is supported for all outputs due to timer limitations
    def set_freq(self, freq) -> bool:
        if self.pwm_setup_failed:
            return False
        for i, pwm in enumerate(self.PWMOutput):
            try:
                pwm.freq(freq)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{i}] freq: {freq}Hz")
            except:
                print(f"H:{self.config.port}:PWM[{i}] freq: {freq}Hz set failed")
                return False
        return True
    
    # set the pulse width for a specific servo output
    # Based on standard RC servos with centre at 1500us and range of 1000-2000us
    # The position is a signed value from -1000 to 1000
    def set_servoposition(self, i, position) -> bool:
        if self.pwm_setup_failed:
            return False
        if i < 0 or i >= len(self.PWMOutput):
            return False
        # Scale servo position to PWM duty cycle (500-2500us)
        pulse_width = 1500000 + (position * 10)
        try:
            if pulse_width != self.PWMOutput[i].duty_ns():
                self.PWMOutput[i].duty_ns(pulse_width)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{i}]:{pulse_width//1000}us")
        except:
            print(f"H:{self.config.port}:PWM[{i}]:{position} set failed")
            return False
        return True

        if self.set_pwmoutput(i, pwm):
            self.time_since_last_update = 0
            return True
        return False 
    
    # Set all 4 PWM duty cycles in one go using a signed value per motor channel (0-65535)
    def set_motors(self, outputs) -> bool:
        if self.pwm_setup_failed:
            return False
        self.outputs_energised = any(outputs)
        for i, output in enumerate(outputs):
            pwmA =  output if output > 0 else 0
            pwmB = -output if output < 0 else 0
            if not self.set_pwmoutput(i<<1, pwmA) or not self.set_pwmoutput((i<<1)+1, pwmB):
                return False
        self.time_since_last_update = 0
        return True

    # Set all 4 PWM duty cycles in one go using a tuple (0-65535)
    def set_pwm(self, pwms) -> bool:
        if self.pwm_setup_failed:
            return False
        self.outputs_energised = any(pwms)
        for i, pwm in enumerate(pwms):
            if not self.set_pwmoutput(i, pwm):
                return False
        self.time_since_last_update = 0
        return True
    
    # Set a single PWM duty cycle (0-65535) for a specific output
    def set_pwmoutput(self, i, pwm) -> bool:
        if i < 0 or i >= len(self.PWMOutput):
            return False
        try:
            if pwm != self.PWMOutput[i].duty_u16():
                self.PWMOutput[i].duty_u16(pwm)
                if self._logging:
                    print(f"H:{self.config.port}:PWM[{i}]:{pwm}")
        except:
            print(f"H:{self.config.port}:PWM[{i}]:{pwm} set failed")
            return False
        return True
    
    def _set_pin_state(self, pin, state):
        try:
            i2c = I2C(7)
            output_reg = i2c.readfrom_mem(pin[0], 0x02+pin[1], 1)[0]
            output_reg = (output_reg | pin[2]) if state else (output_reg & ~pin[2])
            i2c.writeto_mem(pin[0], 0x02+pin[1], bytes([output_reg]))
            if self._logging:
                print(f"H:{self.config.port}:Write to {hex(pin[0])} address {hex(0x02+pin[1])} value {hex(output_reg)}")
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")


    def _get_pin_state(self, pin) -> bool:
        try:
            i2c = I2C(7)
            input_reg = i2c.readfrom_mem(pin[0], 0x00+pin[1], 1)[0]
            return (input_reg & pin[2]) != 0
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")
            return False


    def _set_pin_direction(self, pin, direction):
        try:
            # Use a Try in case access to i2C(7) is blocked for apps in future
            # presumably if this happens then the code will have been updated to
            # handle the GPIO direction correctly anyway.
            i2c = I2C(7)
            config_reg = i2c.readfrom_mem(pin[0], 0x04+pin[1], 1)[0]
            config_reg = (config_reg | pin[2]) if (1 == direction) else (config_reg & ~pin[2])
            i2c.writeto_mem(pin[0], 0x04+pin[1], bytes([config_reg]))
            if self._logging:
                print(f"H:{self.config.port}:Write to {hex(pin[0])} address {hex(0x04+pin[1])} value {hex(config_reg)}")
        except Exception as e:
            print(f"H:{self.config.port}:access to I2C(7) failed: {e}")
    
__app_export__ = HexDriveApp
