"""
BadgeBot sensor drivers package.

Each driver subclass implements SensorBase and targets one specific I2C sensor.
Import ALL_SENSOR_CLASSES to iterate all supported types.
"""

from .vl53l0x  import VL53L0X
from .vl6180x  import VL6180X
from .bme280   import BME280
from .apds9960 import APDS9960
from .tcs3472  import TCS3472
from .tcs3439  import TCS3439

ALL_SENSOR_CLASSES = [VL53L0X, VL6180X, BME280, APDS9960, TCS3472, TCS3439]
