from typing import Any

class RequestForegroundPushEvent:
    app: Any

    def __init__(self, app: Any) -> None: ...

class RequestForegroundPopEvent:
    app: Any

    def __init__(self, app: Any) -> None: ...

class RequestStopAppEvent:
    app: Any

    def __init__(self, app: Any) -> None: ...
