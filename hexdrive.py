# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py
# It is then run from the EEPROM by the BadgeOS.

import asyncio

from machine import I2C, PWM

import app

# HexDrive.py App Version - parsed by app.py to check if upgrade is required
APP_VERSION = 2 

POWER_ENABLE_PIN_INDEX = 0	# First LS pin used to enable the SMPSU
POWER_DETECT_PIN_INDEX = 1  # Second LS pin used to sense if the SMPSU has a source of power

PWM_FREQ = 20000    
class HexDriveApp(app.App):

    def __init__(self, config=None):
        self.config = config
        self.power_state = None
        # report app starting and which port it is running on
        print(f"HexDrive App Init on port {self.config.port}")
        # Set Power Enable Pin to Output
        self.power_control = self.config.ls_pin[POWER_ENABLE_PIN_INDEX]
        self.power_detect = self.config.ls_pin[POWER_DETECT_PIN_INDEX]
        self._set_pin_out(self.power_control.pin)  
        self.set_power(False)
        # Set all HexDrive Hexpansion HS pins to low level outputs
        for hs_pin in self.config.pin:
            hs_pin.value(0)
    
        # Allocate PWM generation to pins
        self.setup_failed = False
        self.PWMOutput = [None] * len(self.config.pin)
        for i_num, hs_pin in enumerate(self.config.pin):
            print(f"H:{self.config.port}:{i_num} PWM on pin {self.config.pin[i_num]}")
            try:
                self.PWMOutput[i_num] = PWM(hs_pin, freq = PWM_FREQ, duty_u16 = 0)
                print(self.PWMOutput[i_num])
            except:
                # There are a finite number of PWM resources so it is possible that we run out
                print(f"H:{self.config.port}:{i_num} PWM allocation failed")
                self.setup_failed = True


    async def background_task(self):
        while 1:
            # we will do something here probably
            await asyncio.sleep(5)


    def get_status(self) -> bool:
        if self.setup_failed:
            return False
        return True


    def set_power(self, state):
        if state == self.power_state:
            return
        print(f"HexDrive [{self.config.port}] Power={state}")
        #TODO - check power_detect pin to see if power is present  
        self._set_pin_value(self.power_control.pin, state)
        self.power_state = state    


    def _set_pin_value(self, pin, value):
        try:
            i2c = I2C(7)
            output_reg = int.from_bytes(i2c.readfrom_mem(pin[0], 0x02+pin[1], 1), 'little')
            if value:
                output_reg |= pin[2]
            else:
                output_reg &= ~(pin[2])
            #print(f"H:Write to {hex(pin[0])} address {hex(0x02+pin[1])} value {hex(output_reg)}")
            i2c.writeto_mem(pin[0], 0x02+pin[1], bytes([output_reg]))
        except:
            print(f"H:access to I2C(7) blocked")


    def _set_pin_out(self, pin):
        try:
            # Use a Try in case access to i2C(7) is blocked for apps in future
            # presumably if this happens then the code will have been updated to
            # handle the GPIO direction correctly anyway.
            i2c = I2C(7)
            config_reg = int.from_bytes(i2c.readfrom_mem(pin[0], 0x04+pin[1], 1), 'little')
            config_reg &= ~(pin[2])
            i2c.writeto_mem(pin[0], 0x04+pin[1], bytes([config_reg]))
        except:
            print(f"access to I2C(7) blocked")


    def set_pwm(self, pwms) -> bool:
        if self.setup_failed:
            return False
        for i, pwm in enumerate(pwms):
            print(f"Set PWM {i} to {pwm}")
            self.PWMOutput[i].duty_u16(pwm)
        return True
    
__app_export__ = HexDriveApp
