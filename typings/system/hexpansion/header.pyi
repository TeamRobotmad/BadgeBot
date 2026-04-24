from __future__ import annotations

from typing import Any

class HexpansionHeader:
    vid: int
    pid: int
    unique_id: int
    friendly_name: str
    eeprom_page_size: int
    eeprom_total_size: int

    def __init__(self, *_args: Any, **_kwargs: Any) -> None: ...
    @classmethod
    def from_bytes(cls, _data: bytes) -> HexpansionHeader: ...

def write_header(*_args: Any, **_kwargs: Any) -> None: ...
def read_header(_i2c: Any, _addr: int, *_args: Any, **_kwargs: Any) -> HexpansionHeader: ...
