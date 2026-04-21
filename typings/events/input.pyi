from typing import Any

class Button:
    pass

class ButtonUpEvent:
    button: Button

    def __init__(self, _button: Button | None = ...) -> None: ...

class ButtonDownEvent:
    button: Button

    def __init__(self, _button: Button | None = ...) -> None: ...

class Buttons:
    def __init__(self, _app: Any) -> None: ...
    def get(self, _button: Button) -> bool: ...
    def clear(self) -> None: ...

BUTTON_TYPES: dict[str, Button]
