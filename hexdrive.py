# This is the app to be installed from the HexDrive Hexpansion EEPROM.
# it is copied onto the EEPROM and renamed as app.py
# It is then run from the EEPROM by the BadgeOS.

import asyncio

import app
from machine import I2C, Pin

from tildagon import Pin as ePin

# HexDrive.py App Version - parsed by app.py to check if upgrade is required
APP_VERSION = 1 

POWER_ENABLE_PIN_INDEX = 0	# First LS pin

class HexDriveApp(app.App):

    def __init__(self, config=None):
        self.config = config
        self.power_state = False
        # report app starting and which port it is running on
        print("HexDrive App Init on port ", self.config.port)
        # Set Power Enable Pin to Output
        HexDrivePowerEnable = self.config.ls_pin[POWER_ENABLE_PIN_INDEX]
        self.set_pin_out(HexDrivePowerEnable.pin)   # Work around as Tildagon(s) (version 1.6) is missing code to set the eGPIO direction to output
        self.set_power(False)
        # Set all HexDrive Hexpansion HS pins to low level outputs
        for hs_pin in self.config.pin:
            hs_pin.value(0)
    
    async def background_task(self):
        while 1:
            # we will do something here probably
            await asyncio.sleep(5)

    # TODO: how to expose this or register it with the event bus so that it can be called/actioned from the main App?
    def set_power(self, state):
        if state == self.power_state:
            return
        if state:
            print("Enable HexDrive Power")
            HexDrivePowerEnable.value(1)
        else:
            # Test Use - disable Hexpansion Power
            print("Disable HexDrive Power")
            HexDrivePowerEnable.value(0)      

    def set_pin_out(self, pin):
        try:
            # Use a Try in case access to i2C(7) is blocked for apps in future
            # presumably if this happens then the code will have been updated to
            # handle the GPIO direction correctly anyway.
            i2c = I2C(7)
            config_reg = int.from_bytes(i2c.readfrom_mem(pin[0], 0x04+pin[1], 1), 'little')
            config_reg &= ~(pin[2])
            i2c.writeto_mem(pin[0], 0x04+pin[1], bytes([config_reg]))
        except:
            print("access to I2C(7) blocked")

__app_export__ = HexDriveApp
