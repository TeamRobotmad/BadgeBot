"""
BadgeBot Sensor Drivers Package
================================
One module per I2C sensor type.

The high-level manager lives one level up:
    from BadgeBot.sensor_manager import SensorManager

Each class in ALL_SENSOR_CLASSES follows the SensorBase interface
(see sensor_base.py) and is auto-discovered by the manager.
"""

ALL_SENSOR_CLASSES = []


def _try_add_sensor(import_name: str, class_name: str) -> None:
    """Import a sensor class if the backing module exists.

    This keeps SensorManager usable even when some driver files are missing
    from the deployed app bundle.
    """
    try:
        module = __import__(f"{__name__}.{import_name}", None, None, (class_name,), 0)
        sensor_class = getattr(module, class_name)
    except (ImportError, AttributeError):
        return
    ALL_SENSOR_CLASSES.append(sensor_class)


# Ordered list used by the manager when scanning a port.
_try_add_sensor("vl53l0x", "VL53L0X")
_try_add_sensor("vl6180x", "VL6180X")
_try_add_sensor("bme280", "BME280")
_try_add_sensor("apds9960", "APDS9960")
_try_add_sensor("tcs3472", "TCS3472")
_try_add_sensor("tcs3439", "TCS3439")
