# -*- coding: utf-8 -*
'''!
  @file        DFRobot_GR10_30.py
  @brief       This is the basic library of GR30_10 sensor
  @copyright   Copyright (c) 2021 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license     The MIT License (MIT)
  @author      zhixinliu(zhixinliu@dfrobot.com)
  @version     V1.0
  @date        2022-07-27
  @url         https://github.com/DFRobor/DFRobot_GR10_30
'''

import time
from machine import I2C

DEV_ADDRESS               = 0x73

#Input Register
GR30_10_INPUTREG_PID                           =0x00   #Device PID
GR30_10_INPUTREG_VID                           =0x01   #VID of the device, fixed to be 0x3343
GR30_10_INPUTREG_ADDR                          =0x02   #Device address of the module
GR30_10_INPUTREG_BAUDRATE                      =0x03   #UART baud rate
GR30_10_INPUTREG_STOPBIT                       =0x04   #UART check bit and stop bit
GR30_10_INPUTREG_VERSION                       =0x05   #Firmware version information
GR30_10_INPUTREG_DATA_READY                    =0x06   #Data ready register
GR30_10_INPUTREG_INTERRUPT_STATE               =0x07   #Gesture interrupt status
GR30_10_INPUTREG_EXIST_STATE                   =0x08   #Presence status
#Holding Register
GR30_10_HOLDINGREG_INTERRUPT_MODE               =0x09   #The gesture that can trigger an interrupt
GR30_10_HOLDINGREG_LRUP_WIN                     =0x0a   #Detection window
GR30_10_HOLDINGREG_L_RANGE                      =0x0b   #The distance your hand should move to the left
GR30_10_HOLDINGREG_R_RANGE                      =0x0c   #The distance your hand should move to the right
GR30_10_HOLDINGREG_U_RANGE                      =0x0d   #The distance your hand should move up
GR30_10_HOLDINGREG_D_RANGE                      =0x0e   #The distance your hand should move down 
GR30_10_HOLDINGREG_FORWARD_RANGE                =0x0f   #The distance your hand should move forward
GR30_10_HOLDINGREG_BACKWARD_RANGE               =0x10   #The distance your hand should move backward
GR30_10_HOLDINGREG_WAVE_COUNT                   =0x11   #The times you need to wave hands
GR30_10_HOLDINGREG_HOVR_WIN                     =0x12   #Hover detection window
GR30_10_HOLDINGREG_HOVR_TIMER                   =0x13   #The duration your hand should hover
GR30_10_HOLDINGREG_CWS_ANGLE                    =0x14   #Clockwise rotation angle, each value equals 22.5°
GR30_10_HOLDINGREG_CCW_ANGLE                    =0x15   #Counterclockwise rotation angle, each value equals 22.5°
GR30_10_HOLDINGREG_CWS_ANGLE_COUNT              =0x16   #Continuous clockwise rotation angle, each value equals 22.5°
GR30_10_HOLDINGREG_CCW_ANGLE_COUNT              =0x17   #Continuous counterclockwise rotation angle, each value equals 22.5°
GR30_10_HOLDINGREG_RESET                        =0x18   #Reset sensor

GESTURE_UP                      = (1<<0)
GESTURE_DOWN                    = (1<<1)
GESTURE_LEFT                    = (1<<2)
GESTURE_RIGHT                   = (1<<3)
GESTURE_FORWARD                 = (1<<4)
GESTURE_BACKWARD                = (1<<5)
GESTURE_CLOCKWISE               = (1<<6)
GESTURE_COUNTERCLOCKWISE        = (1<<7)
GESTURE_WAVE                    = (1<<8)
GESTURE_HOVER                   = (1<<9)
GESTURE_UNKNOWN                 = (1<<10)
GESTURE_CLOCKWISE_C             = (1<<14)
GESTURE_COUNTERCLOCKWISE_C      = (1<<15)

class DFRobot_GR10_30():
  __temp_buffer = [0]*2
  def __init__(self ,bus = 2):
    self.gestures = 0
    self.__temp_buffer = [0]*2
    self.i2cbus = I2C(bus)

  def _detect_device_address(self):
    '''!
      @brief Get sensor address
      @return  Return sensor address
    '''
    rbuf = self._read_reg(GR30_10_INPUTREG_ADDR, 2)
    data = rbuf[0]<<8 | rbuf[1]
    time.sleep(0.1)
    return data

  def begin(self) -> bool:
    '''!
      @brief Init the sensor
    '''
    if self._detect_device_address() != DEV_ADDRESS:
      return False
    self.reset_sensor()
    time.sleep(0.5)
    return True

  def en_gestures(self, gestures) -> bool:
    '''!
      @brief Set what gestures the module can recognize to trigger interrupt
      @param gestures The gesture to be enabled
      @n     GESTURE_UP
      @n     GESTURE_DOWN
      @n     GESTURE_DOWN
      @n     GESTURE_LEFT
      @n     GESTURE_RIGHT
      @n     GESTURE_FORWARD
      @n     GESTURE_BACKWARD
      @n     GESTURE_CLOCKWISE
      @n     GESTURE_COUNTERCLOCKWISE
      @n     GESTURE_WAVE               It is not suggested to enable rotation gesture (CW/CCW) and wave gesture at the same time.
      @n     GESTURE_HOVER              Disable other gestures when hover gesture enables.
      @n     GESTURE_UNKNOWN
      @n     GESTURE_CLOCKWISE_C        Rotate clockwise continuously
      @n     GESTURE_COUNTERCLOCKWISE_C Rotate counterclockwise continuously
      @return Return True if the configuration is successful, otherwise return False
    '''
    self.gestures = gestures&0xc7ff
    self.__temp_buffer[0] = (gestures>>8)&0xC7  # For changing to 8bit
    self.__temp_buffer[1] = gestures&0x00ff
    return (self._write_reg(GR30_10_HOLDINGREG_INTERRUPT_MODE, self.__temp_buffer))

  def set_udlr_win(self, ud_size, lr_size):
    '''!
      @brief Set the detection window 
      @param udSize Distance from top to bottom      distance range 1-30
      @param lrSize Distance from left to right      distance range 1-30
      @return NONE
    '''
    lr_size = lr_size&0x001f
    ud_size = ud_size&0x001f
    self.__temp_buffer[0] = lr_size
    self.__temp_buffer[1] = ud_size
    self._write_reg(GR30_10_HOLDINGREG_LRUP_WIN, self.__temp_buffer)
    time.sleep(0.1)

  def set_left_range(self, range):
    '''!
      @brief Set how far your hand should move to the left so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from left to right of the detection window
      @return NONE
    '''
    range = range&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = range
    self._write_reg(GR30_10_HOLDINGREG_L_RANGE, self.__temp_buffer)
    time.sleep(0.1)

  def set_right_range(self, range):
    '''!
      @brief Set how far your hand should move to the right so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from left to right of the detection window
    '''
    range = range&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = range
    self._write_reg(GR30_10_HOLDINGREG_R_RANGE, self.__temp_buffer)
    time.sleep(0.1)

  def set_up_range(self, range):
    '''!
      @brief Set how far your hand should move up so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from top to bottom of the detection window
    '''
    range = range&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = range
    self._write_reg(GR30_10_HOLDINGREG_U_RANGE, self.__temp_buffer)
    time.sleep(0.1)

  def set_down_range(self, range):
    '''!
      @brief Set how far your hand should move down so the sensor can recognize it
      @param range
      @n     Distance range 5-25, must be less than distance from top to bottom of the detection window
    '''
    range = range&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = range
    self._write_reg(GR30_10_HOLDINGREG_D_RANGE, self.__temp_buffer)
    time.sleep(0.1)

  def set_forward_range(self, range):
    '''!
      @brief Set how far your hand should move forward so the sensor can recognize it
      @param range
      @n     Distance range 1-15
    '''
    range = range&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = range
    self._write_reg(GR30_10_HOLDINGREG_FORWARD_RANGE, self.__temp_buffer)
    time.sleep(0.1)

  def set_backward_range(self, range):
    '''!
      @brief Set how far your hand should move backward so the sensor can recognize it 
      @param range
      @n     Distance range 1-15
    '''
    range = range&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = range
    self._write_reg(GR30_10_HOLDINGREG_BACKWARD_RANGE, self.__temp_buffer)
    time.sleep(0.1)

  def set_wave_number(self, number):
    '''!
      @brief Set how many times you need to wave hands so the sensor can recognize it
      @param number
      @n     Number range 1-15
      @return NONE
    '''
    number = number&0x0f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = number
    self._write_reg(GR30_10_HOLDINGREG_WAVE_COUNT, self.__temp_buffer)
    time.sleep(0.1)

  def set_hover_win(self, ud_size, lr_size):
    '''!
      @brief Set hover detection window
      @param udSize Distance from top to bottom      distance range 1-30
      @param lrSize Distance from left to right      distance range 1-30
      @return NONE
    '''
    lr_size = lr_size&0x001f
    ud_size = ud_size&0x001f
    self.__temp_buffer[0] = lr_size
    self.__temp_buffer[1] = ud_size
    self._write_reg(GR30_10_HOLDINGREG_HOVR_WIN, self.__temp_buffer)
    time.sleep(0.1)

  def set_hover_timer(self, timer):
    '''!
      @brief Set how long your hand should hover to trigger the gesture
      @param timer Each value is 10ms
      @n     timer Maximum is 200  default is 60 600ms
    '''
    timer = timer&0x03FF
    self.__temp_buffer[0] = (timer>>8)&0x03 # For changing to 8bit
    self.__temp_buffer[1] = timer&0x00ff    # For changing to 8bit
    self._write_reg(GR30_10_HOLDINGREG_HOVR_TIMER, self.__temp_buffer)
    time.sleep(0.1) 
    
  def set_cws_angle(self, count):
    '''!
      @brief Set how many degrees your hand should rotate clockwise to trigger the gesture
      @param count Default 16, maximum 31
      @n     count Rotation angle = 22.5 * count
      @n     For example: count = 16, 22.5*count = 360, rotate 360° to trigger the gesture
      @return NONE
    '''
    count = count&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = count
    self._write_reg(GR30_10_HOLDINGREG_CWS_ANGLE, self.__temp_buffer)
    time.sleep(0.1)  

  def set_ccw_angle(self, count):
    '''!
      @brief Set how many degrees your hand should rotate counterclockwise to trigger the gesture
      @param count Default 16, maximum 31
      @n     count Rotation angle = 22.5 * count
      @n     For example: count = 16, 22.5*count = 360, rotate 360° to trigger the gesture
      @return NONE
    '''
    count = count&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = count
    self._write_reg(GR30_10_HOLDINGREG_CCW_ANGLE, self.__temp_buffer)
    time.sleep(0.1)
    
  def set_cws_angle_count(self, count):
    '''!
      @brief Set how many degrees your hand should rotate clockwise continuously to trigger the gesture
      @param count Default 4, maximum 31
      @n     count Continuous rotation angle = 22.5 * count
      @n     For example: count = 4 22.5*count = 90
      @n     Trigger the clockwise/counterclockwise rotation gesture first, 
      @n     if keep rotating, then the continuous rotation gesture will be triggered once every 90 degrees
      @return NONE
    '''
    count = count&0x1f 
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = count
    self._write_reg(GR30_10_HOLDINGREG_CWS_ANGLE_COUNT, self.__temp_buffer)
    time.sleep(0.1)  

  def set_ccw_angle_count(self, count):
    '''!
      @brief Set how many degrees your hand should rotate counterclockwise continuously to trigger the gesture
      @param count Default 4, maximum 31
      @n     count Continuous rotation angle = 22.5 * count
      @n     For example: count = 4 22.5*count = 90
      @n     Trigger the clockwise/counterclockwise rotation gesture first, 
      @n     if keep rotating, then the continuous rotation gesture will be triggered once every 90 degrees
      @return NONE
    '''
    count = count&0x1f
    self.__temp_buffer[0] = 0
    self.__temp_buffer[1] = count
    self._write_reg(GR30_10_HOLDINGREG_CCW_ANGLE_COUNT, self.__temp_buffer)
    time.sleep(0.1)


  def reset_sensor(self):
    '''!
      @brief Reset sensor
      @return NONE
    '''
    self.__temp_buffer[0] = 0x55
    self.__temp_buffer[1] = 0x00
    self._write_reg(GR30_10_HOLDINGREG_RESET, self.__temp_buffer) 
    time.sleep(0.1)
    
    
  def get_data_ready(self) -> bool:
    '''!
      @brief Get if a gesture is detected
      @return If a gesture is detected
      @retval True  Detected
      @retval False  Not detected
    '''
    rbuf = self._read_reg(GR30_10_INPUTREG_DATA_READY, 2)
    data = rbuf[0]*256 + rbuf[1]
    if data == 0x01:
      return True
    else:
      return False
    
  def get_gestures(self) -> int:
    '''!
      @brief Get gesture type
      @return Gesture type
      @retval GESTURE_UP
      @retval GESTURE_DOWN
      @retval GESTURE_DOWN
      @retval GESTURE_LEFT
      @retval GESTURE_RIGHT
      @retval GESTURE_FORWARD
      @retval GESTURE_BACKWARD
      @retval GESTURE_CLOCKWISE
      @retval GESTURE_COUNTERCLOCKWISE
      @retval GESTURE_WAVE
      @retval GESTURE_HOVER
      @retval GESTURE_UNKNOWN
      @retval GESTURE_CLOCKWISE_C
      @retval GESTURE_COUNTERCLOCKWISE_C
    '''
    rbuf = self._read_reg(GR30_10_INPUTREG_INTERRUPT_STATE, 2)
    data = rbuf[0]*256 + rbuf[1]
    return data
  
  
  def get_exist(self) -> bool:
    '''!
      @brief Get whether the object is in the sensor detection range
      @return If the object is in the sensor detection range
      @retval 0 No
      @retval 1 Yes
    '''
    rbuf = self._read_reg(GR30_10_INPUTREG_EXIST_STATE, 2)
    data = rbuf[0]*256 + rbuf[1]
    return data
  
class DFRobot_GR30_10_I2C(DFRobot_GR10_30):
  '''!
    @brief An example of an i2c interface module
  '''
  def __init__(self ,bus ,addr):
    self._addr = addr
    DFRobot_GR10_30.__init__(self,bus)   
    
  
  def _read_reg(self, reg_addr ,length) -> list:
    '''!
      @brief read the data from the register
      @param reg_addr register address
      @param length read data
    '''
    reg = reg_addr
    try:
      rslt = self.i2cbus.readfrom_mem(self._addr, reg, length)
    except:
      rslt = [0,0]
    return rslt

  def _write_reg(self, reg_addr ,data) -> bool:
    '''!
      @brief write the data from the register
      @param reg_addr register address
      @param data Data to be written to register
    '''
    self._reg = reg_addr
    # convert data from list to bytes which supports buffer protocol
    new_data = bytearray(data)
    #print(f"write to addr:{self._addr} reg:{self._reg} data:{new_data}")
    try:
      self.i2cbus.writeto_mem(self._addr, self._reg, new_data)
      rslt = True
    except Exception as e:
        print(f"write error:{e}")
        rslt = False
    return rslt
       