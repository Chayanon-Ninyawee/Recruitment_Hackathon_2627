from .state import DriverConfig, DriverState


def compute_control(
    state: DriverState,
    config: DriverConfig,
):
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
