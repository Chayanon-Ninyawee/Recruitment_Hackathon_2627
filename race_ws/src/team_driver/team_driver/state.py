from dataclasses import dataclass

from builtin_interfaces.msg import Time
from nav_msgs.msg import OccupancyGrid


@dataclass
class DriverState:
    position: tuple | None = None
    yaw: float = 0.0
    speed: float = 0.0
    acceleration: float = 0.0
    dt: float | None = None
    map: OccupancyGrid | None = None
    stamp: Time | None = None


@dataclass
class DriverConfig:
    crawl_speed: float
