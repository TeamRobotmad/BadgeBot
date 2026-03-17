from typing import Any

class HexpansionConfig:
    port: int
    pin: list[Any]
    ls_pin: list[Any]
    i2c: Any

    def __init__(self, port: int) -> None: ...
