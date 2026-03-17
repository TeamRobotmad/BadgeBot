from typing import Any

class Button:
    pass

class ButtonUpEvent:
    button: Button

    def __init__(self, button: Button | None = ...) -> None: ...

class Buttons:
    def __init__(self, app: Any) -> None: ...
    def get(self, button: Button) -> bool: ...
    def clear(self) -> None: ...

BUTTON_TYPES: dict[str, Button]
