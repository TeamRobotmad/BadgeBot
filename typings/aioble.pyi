# aioble.pyi
"""
Type stubs for the MicroPython asynchronous BLE library (aioble).
"""

from typing import Any, Generator, List, Optional, Tuple, Union
import bluetooth

class DeviceConnection:
    def disconnect(self) -> None: ...
    async def disconnected(self) -> None: ...
    def is_connected(self) -> bool: ...
    def service_discovered(self) -> None: ...

class Service:
    uuid: bluetooth.UUID
    def __init__(self, uuid: bluetooth.UUID) -> None: ...

class Characteristic:
    uuid: bluetooth.UUID
    service: Service
    def __init__(
        self,
        service: Service,
        uuid: bluetooth.UUID,
        read: bool = False,
        write: bool = False,
        notify: bool = False,
        indicate: bool = False,
        write_no_response: bool = False,
        capture: bool = False,
    ) -> None: ...

    async def written(self) -> Tuple[DeviceConnection, bytes]: ...
    def notify(self, connection: DeviceConnection, data: Union[bytes, bytearray, str]) -> None: ...
    def indicate(self, connection: DeviceConnection, data: Union[bytes, bytearray, str]) -> None: ...
    def read(self) -> bytes: ...
    def write(self, data: Union[bytes, bytearray, str], send_update: bool = False) -> None: ...

def register_services(*services: Service) -> None: ...

async def advertise(
    interval_us: int,
    name: Optional[str] = None,
    services: Optional[List[bluetooth.UUID]] = None,
    appearance: Optional[int] = None,
    adv_data: Optional[Union[bytes, bytearray]] = None,
    scan_rsp: Optional[Union[bytes, bytearray]] = None,
) -> DeviceConnection: ...

class Device:
    device_id: int
    addr_type: int
    addr: bytes
    name: str
    rssi: int
    connectable: bool

class Central:
    async def scan(
        self, timeout_ms: int = 2000, interval_us: int = 1280000, window_us: int = 1125000, active: bool = False
    ) -> Generator[None, None, Device]: ...

    async def connect(self, device: Device, timeout_ms: int = 2000) -> DeviceConnection: ...

central: Central
