import math


def in_bounds(
    width,
    height,
    x,
    y,
):
    return 0 <= x < width and 0 <= y < height


def index(
    width,
    x,
    y,
):
    return y * width + x


def world_to_map(
    state,
    x,
    y,
):
    info = state.map.info

    mx = int(math.floor((x - info.origin.position.x) / info.resolution))

    my = int(math.floor((y - info.origin.position.y) / info.resolution))

    return mx, my


def map_to_world(
    state,
    x,
    y,
):
    info = state.map.info

    return (
        info.origin.position.x + (x + 0.5) * info.resolution,
        info.origin.position.y + (y + 0.5) * info.resolution,
    )


def is_obstacle(
    state,
    x,
    y,
):
    """
    Match the reference tool's occupancy behavior.

    ROS OccupancyGrid:
        0      = free
        100    = occupied
        -1     = unknown

    Unknown is treated as free.
    """

    width = state.map.info.width
    height = state.map.info.height

    if not in_bounds(
        width,
        height,
        x,
        y,
    ):
        return True

    value = state.map.data[
        index(
            width,
            x,
            y,
        )
    ]

    return value >= 65


def is_free(
    state,
    x,
    y,
):
    if not in_bounds(
        state.map.info.width,
        state.map.info.height,
        x,
        y,
    ):
        return False

    return not is_obstacle(
        state,
        x,
        y,
    )
