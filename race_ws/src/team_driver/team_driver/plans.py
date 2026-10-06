from dataclasses import dataclass

from nav_msgs.msg import OccupancyGrid


@dataclass
class DriverConfig:
    crawl_speed: float


@dataclass
class DriverState:
    position: tuple | None = None
    yaw: float = 0.0
    speed: float = 0.0
    acceleration: float = 0.0
    dt: float | None = None
    map: OccupancyGrid | None = None


def compute_control(state, config):
    """
    Compute the desired steering and speed.

    Parameters
    ----------
    state : DriverState
        Current vehicle state and map.

    config : DriverConfig
        Tunable driver parameters.

    Returns
    -------
    steering : float
        Steering angle in radians.

    speed : float
        Requested speed in m/s.
    """

    return 0.0, config.crawl_speed
