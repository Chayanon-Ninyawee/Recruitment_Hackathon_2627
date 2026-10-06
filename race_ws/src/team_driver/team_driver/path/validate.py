import math

from .map import in_bounds, is_obstacle, world_to_map


def footprint_is_safe(
    state,
    x,
    y,
    safety_radius,
):
    """
    Validate circular robot footprint against
    the original occupancy map.
    """

    width = state.map.info.width
    height = state.map.info.height
    resolution = state.map.info.resolution

    center_x, center_y = world_to_map(
        state,
        x,
        y,
    )

    radius_cells = safety_radius / resolution

    check_radius = int(math.ceil(radius_cells)) + 1

    radius_squared = radius_cells * radius_cells

    for dy in range(
        -check_radius,
        check_radius + 1,
    ):
        for dx in range(
            -check_radius,
            check_radius + 1,
        ):

            if dx * dx + dy * dy > radius_squared:
                continue

            mx = center_x + dx
            my = center_y + dy

            if not in_bounds(
                width,
                height,
                mx,
                my,
            ):
                return False

            if is_obstacle(
                state,
                mx,
                my,
            ):
                return False

    return True


def segment_is_safe(
    state,
    start,
    end,
    safety_radius,
):
    """
    Validate the robot footprint continuously
    along a segment.
    """

    distance = math.dist(
        start,
        end,
    )

    resolution = state.map.info.resolution

    spacing = max(
        resolution * 0.5,
        0.01,
    )

    steps = max(
        1,
        int(math.ceil(distance / spacing)),
    )

    for i in range(steps + 1):

        t = i / steps

        x = start[0] + t * (end[0] - start[0])

        y = start[1] + t * (end[1] - start[1])

        if not footprint_is_safe(
            state,
            x,
            y,
            safety_radius,
        ):
            return False

    return True


def validate_path(
    state,
    path,
    safety_radius,
):
    """
    Final geometric validation.
    """

    if len(path) < 3:
        return False

    for i, point in enumerate(path):

        if not footprint_is_safe(
            state,
            point[0],
            point[1],
            safety_radius,
        ):
            print("PATH: footprint validation " f"failed at point {i}")

            return False

    for i in range(len(path) - 1):

        if not segment_is_safe(
            state,
            path[i],
            path[i + 1],
            safety_radius,
        ):
            print(
                "PATH: segment validation " f"failed between points " f"{i} and {i + 1}"
            )

            return False

    return True
