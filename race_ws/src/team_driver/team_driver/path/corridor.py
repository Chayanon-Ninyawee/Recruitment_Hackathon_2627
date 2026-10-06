import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from .map import in_bounds, index, world_to_map


def _normal(path, i):
    """
    Return the left-hand unit normal of the path at index i.
    """

    n = len(path)

    px, py = path[(i - 1) % n]
    nx, ny = path[(i + 1) % n]

    dx = nx - px
    dy = ny - py

    length = math.hypot(dx, dy)

    if length < 1e-9:
        return 0.0, 0.0

    dx /= length
    dy /= length

    return -dy, dx


def _clearance_in_direction(
    state,
    obstacles,
    x,
    y,
    dx,
    dy,
    robot_radius,
):
    """
    Find how far the robot centre can travel in a direction
    before its footprint reaches an obstacle.

    The returned distance is the maximum safe centre distance.
    """

    resolution = state.map.info.resolution

    step = max(
        resolution * 0.5,
        0.01,
    )

    max_distance = 2.0

    distance = 0.0

    radius_cells = max(
        1,
        int(math.ceil(robot_radius / resolution)),
    )

    while distance <= max_distance:
        px = x + dx * distance
        py = y + dy * distance

        mx, my = world_to_map(
            state,
            px,
            py,
        )

        # Outside the map is a boundary.
        if not in_bounds(
            state.map.info.width,
            state.map.info.height,
            mx,
            my,
        ):
            return max(
                0.0,
                distance - step - robot_radius,
            )

        # Check the robot footprint around this position.
        for oy in range(
            -radius_cells,
            radius_cells + 1,
        ):
            for ox in range(
                -radius_cells,
                radius_cells + 1,
            ):
                cx = mx + ox
                cy = my + oy

                if not in_bounds(
                    state.map.info.width,
                    state.map.info.height,
                    cx,
                    cy,
                ):
                    return max(
                        0.0,
                        distance - step,
                    )

                if ox * ox + oy * oy > radius_cells * radius_cells:
                    continue

                if obstacles[
                    index(
                        state.map.info.width,
                        cx,
                        cy,
                    )
                ]:
                    return max(
                        0.0,
                        distance - step,
                    )

        distance += step

    return max_distance


def compute_boundaries(
    state,
    path,
    obstacles,
    robot_radius,
):
    """
    Calculate the usable left/right corridor width
    around every centreline point.

    The returned distances are for the robot centre,
    so robot_radius is already accounted for.
    """

    left_clearance = []
    right_clearance = []

    for i, (x, y) in enumerate(path):

        nx, ny = _normal(
            path,
            i,
        )

        left = _clearance_in_direction(
            state,
            obstacles,
            x,
            y,
            nx,
            ny,
            robot_radius,
        )

        right = _clearance_in_direction(
            state,
            obstacles,
            x,
            y,
            -nx,
            -ny,
            robot_radius,
        )

        left_clearance.append(
            max(
                0.0,
                left,
            )
        )

        right_clearance.append(
            max(
                0.0,
                right,
            )
        )

    return (
        left_clearance,
        right_clearance,
    )


def build_boundary_points(
    path,
    left_clearance,
    right_clearance,
):
    """
    Convert left/right clearance distances into
    actual world-coordinate boundary points.
    """

    left_points = []
    right_points = []

    n = len(path)

    for i, (x, y) in enumerate(path):

        px, py = path[(i - 1) % n]
        nx, ny = path[(i + 1) % n]

        dx = nx - px
        dy = ny - py

        length = math.hypot(
            dx,
            dy,
        )

        if length < 1e-9:
            normal_x = 0.0
            normal_y = 0.0
        else:
            dx /= length
            dy /= length

            normal_x = -dy
            normal_y = dx

        left_distance = left_clearance[i]
        right_distance = right_clearance[i]

        left_points.append(
            (
                x + normal_x * left_distance,
                y + normal_y * left_distance,
            )
        )

        right_points.append(
            (
                x - normal_x * right_distance,
                y - normal_y * right_distance,
            )
        )

    return (
        left_points,
        right_points,
    )


def build_track_corridor(
    state,
    path,
    obstacles,
    robot_radius,
):
    """
    Build the track corridor directly from the
    connected cone/barrier map.

    The corridor is represented by:

        centreline
        left clearance
        right clearance
        left/right boundary points
        per-point sections

    The generated corridor is also stored in:

        state.corridor
    """

    (
        left_clearance,
        right_clearance,
    ) = compute_boundaries(
        state,
        path,
        obstacles,
        robot_radius,
    )

    (
        left_points,
        right_points,
    ) = build_boundary_points(
        path,
        left_clearance,
        right_clearance,
    )

    sections = []

    n = len(path)

    for i, (x, y) in enumerate(path):

        px, py = path[(i - 1) % n]
        nx, ny = path[(i + 1) % n]

        heading = math.atan2(
            ny - py,
            nx - px,
        )

        sections.append(
            {
                "x": x,
                "y": y,
                "heading": heading,
                "left": max(
                    0.0,
                    left_clearance[i],
                ),
                "right": max(
                    0.0,
                    right_clearance[i],
                ),
                "left_point": left_points[i],
                "right_point": right_points[i],
            }
        )

    corridor = {
        "sections": sections,
        "left_clearance": left_clearance,
        "right_clearance": right_clearance,
        "left_points": left_points,
        "right_points": right_points,
    }

    state.corridor = corridor

    return corridor


def corridor_widths(corridor):
    """
    Return left/right corridor widths.
    """

    if corridor is None:
        return [], []

    return (
        corridor.get(
            "left_clearance",
            [],
        ),
        corridor.get(
            "right_clearance",
            [],
        ),
    )


def corridor_points(corridor):
    """
    Return left/right boundary points.
    """

    if corridor is None:
        return [], []

    return (
        corridor.get(
            "left_points",
            [],
        ),
        corridor.get(
            "right_points",
            [],
        ),
    )


def publish_corridor_marker(
    state,
    corridor,
    publisher,
):
    """
    Publish the corridor as a lightweight triangle mesh.

    This is visualization only.

    The actual MPC corridor constraint is represented
    by the left/right widths.
    """

    if publisher is None:
        return

    if corridor is None:
        return

    sections = corridor.get(
        "sections",
        [],
    )

    if len(sections) < 2:
        return

    marker = Marker()

    marker.header = state.map.header

    marker.ns = "track_corridor"
    marker.id = 0
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD

    marker.pose.orientation.w = 1.0

    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0

    marker.color.r = 0.2
    marker.color.g = 0.8
    marker.color.b = 1.0
    marker.color.a = 0.25

    for i in range(len(sections) - 1):

        a = sections[i]
        b = sections[i + 1]

        ax, ay = a["left_point"]
        bx, by = b["left_point"]

        arx, ary = a["right_point"]
        brx, bry = b["right_point"]

        marker.points.append(
            Point(
                x=ax,
                y=ay,
                z=0.02,
            )
        )

        marker.points.append(
            Point(
                x=arx,
                y=ary,
                z=0.02,
            )
        )

        marker.points.append(
            Point(
                x=bx,
                y=by,
                z=0.02,
            )
        )

        marker.points.append(
            Point(
                x=bx,
                y=by,
                z=0.02,
            )
        )

        marker.points.append(
            Point(
                x=arx,
                y=ary,
                z=0.02,
            )
        )

        marker.points.append(
            Point(
                x=brx,
                y=bry,
                z=0.02,
            )
        )

    publisher.publish(marker)
