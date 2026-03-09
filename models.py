"""BadgeBot data models: StepperMode, ServoMode, HexDriveType, MySetting, Instruction."""
import settings
from events.input import BUTTON_TYPES, Button

from .constants import _TICK_MS


class StepperMode:
    OFF = 0
    POSITION = 1
    SPEED = 2
    stepper_modes = ["OFF", "POSITION", "SPEED"]

    def __init__(self, mode = OFF):
        self.mode = mode
        
    def set(self, mode):
        self.mode = mode

    def inc(self):
        self.mode = (self.mode + 1) % 3

    def __eq__(self, other):
        return self.mode == other
    
    def __str__(self):
        return self.stepper_modes[self.mode]


class ServoMode:
    OFF = 0
    TRIM = 1
    POSITION = 2
    SCANNING = 3
    servo_modes = ["OFF", "TRIM", "POSITION", "SCANNING"]
    
    def __init__(self, mode = OFF):
        self.mode = mode

    def set(self, mode):
        self.mode = mode

    def inc(self):
        self.mode = (self.mode + 1) % 4
    
    def __eq__(self, other):
        return self.mode == other
    
    def __str__(self):
        return self.servo_modes[self.mode]


class HexDriveType:
    def __init__(self, pid, vid = 0xCAFE, motors = 0, steppers = 0, servos = 0, name ="Unknown"):
        self.vid = vid
        self.pid = pid
        self.name = name
        self.motors = motors
        self.servos = servos
        self.steppers = steppers


class MySetting:
    def __init__(self, container, default, minimum, maximum):
        self._container = container
        self.d = default
        self.v = default
        self._min = minimum
        self._max = maximum


    def __str__(self):
        return str(self.v)


    def _index(self):
        for k,v in self._container.items():
            if v == self:
                return k
        return None

        
    # This returns an increase in the value passed in - subject to max and with scale of increase depending on level
    # based on the type of the setting
    # it does not affect the current value of the setting
    def inc(self, v, l=0):            
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l==0:
                v += 1
            else:
                d = 10**l
                v = ((v // d) + 1) * d   # round up to the next multiple of 10^l, being very careful not to cause big jumps when value was nearly at the next multiple 

            if v > self._max:
                v = self._max
        elif isinstance(self.v, float):
            # only float at present is brightness from 0.0 to 1.0
            v += 0.1            
            if v > self._max:
                v = self._max  
        elif self._container['logging'].v:
            print(f"H:inc type: {type(self.v)}")                               
        return v

    # This returns a decrease in the value passed in - subject to min and with scale of increase depending on level
    # based on the type of the setting
    # it does not affect the current value of the setting
    def dec(self, v, l=0):            
        if isinstance(self.v, bool):
            v = not v
        elif isinstance(self.v, int):
            if l==0:
                v -= 1
            else:
                d = 10**l
                v = (((v+(9*(10**(l-1)))) // d) - 1) * d   # round down to the next multiple of 10^l

            if v < self._min:
                v = self._min       
        elif isinstance(self.v, float):
            # only float at present is brightness from 0.0 to 1.0
            v -= 0.1            
            if v < self._min:
                v = self._min
        elif self._container['logging'].v:
            print(f"H: dec type: {type(self.v)}") 
        return v
    

    def persist(self):
        # only save non-default settings to the settings store
        try:
            if self.v != self.d:
                settings.set(f"badgebot.{self._index()}", self.v)
            else:
                settings.set(f"badgebot.{self._index()}", None)
        except Exception as e:
            print(f"H:Failed to persist setting {self._index()}: {e}")


class Instruction:
    def __init__(self, press_type: Button) -> None:
        self._press_type = press_type
        self._duration = 1
        self.power_plan = []


    @property
    def press_type(self) -> Button:
        return self._press_type


    def inc(self):
        self._duration += 1


    def __str__(self):
        return f"{self.press_type.name} {self._duration}"


    def directional_power_tuple(self, power):
        if   self._press_type == BUTTON_TYPES["UP"]:
            return ( power,  power)   # both motors forward
        elif self._press_type == BUTTON_TYPES["DOWN"]:
            return (-power, -power)   # both motors backward
        elif self._press_type == BUTTON_TYPES["LEFT"]:
            return (-power,  power)   # turn left
        elif self._press_type == BUTTON_TYPES["RIGHT"]:
            return ( power, -power)   # turn right


    def directional_duration(self, mysettings):
        if   self._press_type == BUTTON_TYPES["UP"] or self._press_type == BUTTON_TYPES["DOWN"]:
            return (mysettings['drive_step_ms'].v)            
        elif self._press_type == BUTTON_TYPES["LEFT"] or self._press_type == BUTTON_TYPES["RIGHT"]:
            return (mysettings['turn_step_ms'].v)
        

    def make_power_plan(self, mysettings):
        # return collection of tuples of power and their duration
        curr_power = 0
        ramp_up = []
        for i in range(1*(self._duration+3)):
            ramp_up.append((self.directional_power_tuple(curr_power), _TICK_MS))
            curr_power += mysettings['acceleration'].v
            if curr_power >= mysettings['max_power'].v:
                ramp_up.append((self.directional_power_tuple(mysettings['max_power'].v), _TICK_MS))
                break
        user_power_duration = (self.directional_duration(mysettings) * self._duration)-(2*(i+1)*_TICK_MS)
        power_durations = ramp_up.copy()
        if user_power_duration > 0:
            power_durations.append((self.directional_power_tuple(mysettings['max_power'].v), user_power_duration))
        ramp_down = ramp_up.copy()
        ramp_down.reverse()
        power_durations.extend(ramp_down)
        if mysettings['logging'].v:
            print("Power durations:")
            print(power_durations)
        self.power_plan = power_durations
