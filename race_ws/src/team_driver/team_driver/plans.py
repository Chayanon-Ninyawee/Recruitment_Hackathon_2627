from .control import LQRTracker
from .state import DriverState


def compute_control(
    state: DriverState,
    controller: LQRTracker,
):
    if state.position is None:
        return 0.0, 0.0

    if state.path is None:
        return 0.0, 0.0

    if len(state.path) < 2:
        return 0.0, 0.0

    x, y = state.position

    closest_index = controller.find_nearest_index(
        x,
        y,
        state.path,
    )

    if closest_index is None:
        return 0.0, 0.0

    reference = controller.get_reference(
        state.path,
        closest_index,
    )

    if reference is None:
        return 0.0, 0.0

    lateral_error = controller.lateral_error(
        x,
        y,
        reference,
    )

    heading_error = controller.heading_error(
        state.yaw,
        reference,
    )

    steering, speed = controller.control(
        lateral_error=lateral_error,
        heading_error=heading_error,
        curvature=reference.curvature,
        speed=state.speed,
    )

    return steering, speed
