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
    path: list[tuple[float, float]] | None = None

    stamp: Time | None = None


@dataclass
class DriverConfig:
    # Speed
    max_speed: float = 6.0
    min_speed: float = 1.0

    # Vehicle
    wheelbase: float = 0.33
    max_steering: float = 0.45

    # Pure Pursuit
    min_lookahead: float = 0.45
    max_lookahead: float = 2.0

    # Lookahead tuning
    lookahead_speed_gain: float = 0.25
    lookahead_steering_reduction: float = 0.70

    # Corner speed tuning
    steering_speed_reduction: float = 0.75
