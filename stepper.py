"""Stepper motor software controller using a hardware Timer for step timing."""
import time
from math import pi

try:
    from machine import Timer
except ImportError:
    Timer = None

from .constants import (
    _STEPPER_NUM_PHASES,
    _STEPPER_DEFAULT_SPR,
    _STEPPER_DEFAULT_SPEED,
    _STEPPER_MAX_SPEED,
    _STEPPER_MAX_POSITION,
)


class Stepper:
    def __init__(self, container, hexdrive_app, step_size: int = 1, steps_per_rev: int = _STEPPER_DEFAULT_SPR, speed_sps: int = _STEPPER_DEFAULT_SPEED, max_sps: int = _STEPPER_MAX_SPEED, max_pos: int = _STEPPER_MAX_POSITION, timer_id: int = 0):
        self._container = container 
        self._hexdrive_app = hexdrive_app
        self._phase = 0
        self._calibrated = False
        self._timer = Timer(timer_id) if Timer is not None else None
        self._timer_is_running = False
        self._timer_mode = 0
        self._free_run_mode = 0                     # direction of free run mode
        self._enabled = False
        self._target_pos = 0
        self._pos = 0                               # current position in half steps
        self._max_sps = int(max_sps)                # max speed in full steps per second
        self._steps_per_sec = int(speed_sps)        # current speed in full steps per second
        self._steps_per_rev = int(steps_per_rev)    # full steps per revolution
        self._max_pos = 2*int(max_pos)              # max position stored in half steps
        self._freq = 0
        self._min_period = 0
        self._step_size = int(step_size)            # 1 = half steps, 2 = full steps
        self._last_step_time = 0    
        self.track_target()
        
    def step_size(self,sz=1):
        if sz < 1:
            sz = 1
        elif sz > 2:
            sz = 2
        self._step_size = int(sz)

    def speed(self,sps):    # speed in FULL steps per second
        if self._free_run_mode == 1 and sps < 0:
            self._free_run_mode = -1
        elif self._free_run_mode == -1 and sps > 0:
            self._free_run_mode = 1
        if sps > self._max_sps:
            sps = self._max_sps
        elif sps < -self._max_sps:
            sps = -self._max_sps
        self._steps_per_sec = int(sps)
        self._update_timer((2//self._step_size)*abs(self._steps_per_sec))    # steps per second

    def speed_rps(self,rps):
        self.speed(rps*self._steps_per_rev)

    def get_speed(self) -> int:
        return self._steps_per_sec

    def target(self,t):
        if self._calibrated and t < 0:
            # when already calibrated limit to 0
            self._target_pos = 0
        elif self._calibrated and (2*int(t)) > self._max_pos:
            # when already calibrated limit to max
            self._target_pos = self._max_pos
        else:
            self._target_pos = 2*int(t)

    def target_deg(self,deg):
        self.target(self._steps_per_rev*deg/360.0)  # target pos is in steps
    
    def target_rad(self,rad):
        self.target(self._steps_per_rev*rad/(2*pi)) # target pos is in steps
    
    def get_pos(self) -> int:
        return (self._pos//2)   # convert half steps to full steps
    
    def get_pos_deg(self) -> float:
        return self._pos*180.0/self._steps_per_rev      # half steps to degrees
    
    def get_pos_rad(self) -> float:
        return self._pos*pi/self._steps_per_rev         # half steps to radians
    
    def overwrite_pos(self,p=0):
        self._pos = 2*int(p)    # convert full steps to half steps
    
    def overwrite_pos_deg(self,deg):
        self._pos = deg*self._steps_per_rev/180.0      # degrees to half steps
    
    def overwrite_pos_rad(self,rad):
        self._pos = rad*self._steps_per_rev/pi         # radians to half steps

    def step(self,d=0):
        cur_time = time.ticks_ms()
        if time.ticks_diff(cur_time, self._last_step_time) < self._min_period:
            # avoid stepping too quickly as this causes skipped steps
            return
        self._last_step_time = cur_time
        if d>0:
            self._pos+=self._step_size
            self._phase = (self._phase-self._step_size)%_STEPPER_NUM_PHASES
        elif d<0:
            self._pos-=self._step_size
            self._phase = (self._phase+self._step_size)%_STEPPER_NUM_PHASES
        # Check position limits
        if self._calibrated and self._pos < 0:
            print("s/w min endstop")
            self._pos = 0
            self.speed(0)
            return
        elif self._calibrated and self._pos > self._max_pos:
            print("s/w max endstop")
            self._pos = self._max_pos
            self.speed(0)
            return
        try:
            self._hexdrive_app.motor_step(self._phase)
        except Exception as e:                       
            print(f"step phase {self._phase} failed:{e}")

    # There is no code to handle the endstop being hit at present - it needs to be specific to the hardware
    # i.e. which pin is connected to the endstop.
    def _hit_endstop(self):             
        print("Endstop - hit")
        if not self._calibrated:
            self._calibrated = True
        # set this as the new zero position
        self.overwrite_pos(0)
        # if we were moving towards the endstop, stop
        if self._free_run_mode < 0:
            self.speed(0)
        elif self._free_run_mode == 0 and self._target_pos < self._pos:
            self.speed(0)

    def _timer_callback_fwd(self,t):
        self.step(1)

    def _timer_callback_rev(self,t):
        self.step(-1)

    def _timer_callback(self,t):
        if self._target_pos>self._pos:
            self.step(1)
        elif self._target_pos<self._pos:
            self.step(-1)

    def free_run(self,d=1):
        self._free_run_mode=d
        if d!=0:
            self._update_timer((2//self._step_size)*abs(self._steps_per_sec))   # half steps per second

    def track_target(self):
        self._free_run_mode=0
        self._update_timer((2//self._step_size)*abs(self._steps_per_sec))      # half steps per second

    def _update_timer(self,freq):
        if self._timer is None:
            return
        if self._timer_is_running and freq != self._freq:
            try:
                self._timer.deinit()
                self._freq = 0
                self._timer_is_running=False
            except Exception as e:
                print(f"update_timer failed:{e}")
        if 0 != freq and (freq != self._freq or self._free_run_mode != self._timer_mode):
            try:                
                print(f"Timer: {freq}Hz")
                if self._free_run_mode>0:
                    self._timer.init(freq=freq,callback=self._timer_callback_fwd)
                elif self._free_run_mode<0:
                    self._timer.init(freq=freq,callback=self._timer_callback_rev)
                else:
                    self._timer.init(freq=freq,callback=self._timer_callback)
                self._freq = freq
                self._min_period = (1000//freq) - 1
                self._timer_is_running=True
                self._timer_mode = self._free_run_mode
            except Exception as e:
                print(f"update_timer failed:{e}")
        elif freq == 0:
            print("Timer: 0Hz")


    def stop(self):
        self._update_timer(0)
        try:
            self._hexdrive_app.motor_release()
        except Exception as e:
            print(f"stop failed:{e}")

    def enable(self,e = True):
        self._enabled=e
        try:
            if e:
                if self._free_run_mode!=0:
                    self._update_timer((2//self._step_size)*abs(self._steps_per_sec))   # half steps per second                
                self._hexdrive_app.motor_step(self._phase)
            else:
                self._update_timer(0)
                self._hexdrive_app.motor_release()
            self._hexdrive_app.set_power(e)
        except Exception as e:
            print(f"enable failed:{e}")

    def is_enabled(self) -> bool:
        return self._enabled
