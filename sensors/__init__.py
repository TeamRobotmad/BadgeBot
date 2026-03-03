"""
BadgeBot Sensor Drivers Package
================================
One module per I2C sensor type.

The high-level manager lives one level up:
    from BadgeBot.sensor_manager import SensorManager

Each class in ALL_SENSOR_CLASSES follows the SensorBase interface
(see sensor_base.py) and is auto-discovered by the manager.
"""

from .vl53l0x  import VL53L0X
from .vl6180x  import VL6180X
from .bme280   import BME280
from .apds9960 import APDS9960
from .tcs3472  import TCS3472
from .tcs3439  import TCS3439

# Ordered list used by the manager when scanning a port.
ALL_SENSOR_CLASSES = [VL53L0X, VL6180X, BME280, APDS9960, TCS3472, TCS3439]
