# support Any
from typing import Any

class HexpansionInsertionEvent:
    port: int

    def __init__(self, port: int) -> None: ...

class HexpansionRemovalEvent:
    port: int

    def __init__(self, port: int) -> None: ...

class HexpansionMountedEvent:
    port: int
    header: Any

    def __init__(self, port: int, header: Any) -> None: ...

class HexpansionUnmountedEvent:
    port: int

    def __init__(self, port: int) -> None: ...