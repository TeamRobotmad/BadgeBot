from typing import Any

class _Leds:
    def __getitem__(self, index: int) -> tuple[int, int, int]: ...
    def __setitem__(self, index: int, value: tuple[int, int, int]) -> None: ...
    def write(self) -> None: ...

class _TildagonOS:
    leds: _Leds

tildagonos: _TildagonOS
