from .clearance import build_euclidean_clearance, build_safe_map
from .cones import link_cones, publish_connected_cone_map
from .lap import find_closed_lap, find_track_component
from .map import in_bounds, index, is_free, world_to_map
from .smoothing import (
    clearance_at_world,
    enforce_clearance,
    path_length,
    resample_closed,
    rotate_path_to_start,
    smooth_closed,
)
from .validate import validate_path

ROBOT_RADIUS = 0.17

TRACK_CLEARANCE = 0.35

PATH_SPACING = 0.05

SMOOTH_WINDOW = 21

CLEARANCE_ITERATIONS = 60


def construct_path(
    state,
    connected_cone_map_pub=None,
):
    """
    Generate a safe closed centreline around the cone track.

    Pipeline:

        occupancy
            ↓
        cone detection
            ↓
        cone linking at 1.1 m
            ↓
        connected barrier map
            ↓
        Euclidean clearance
            ↓
        0.35 m safe track
            ↓
        virtual finish line
            ↓
        Dijkstra closed lap
            ↓
        smoothing
            ↓
        clearance enforcement
            ↓
        resampling
            ↓
        final footprint validation
    """

    if state.map is None:
        print("PATH: no map")

        return []

    if state.position is None:
        print("PATH: no robot position")

        return []

    info = state.map.info

    width = info.width
    height = info.height
    resolution = info.resolution

    print("PATH: " f"map={width}x{height}, " f"resolution={resolution:.3f} m")

    # ========================================================
    # ROBOT POSITION
    # ========================================================

    start_x, start_y = world_to_map(
        state,
        state.position[0],
        state.position[1],
    )

    print("PATH: robot cell=" f"({start_x}, {start_y})")

    if not in_bounds(
        width,
        height,
        start_x,
        start_y,
    ):
        print("PATH: robot is outside map")

        return []

    if not is_free(
        state,
        start_x,
        start_y,
    ):
        print("PATH: robot is in occupied space")

        return []

    # ========================================================
    # CLEARANCE
    # ========================================================

    print("PATH: robot radius=" f"{ROBOT_RADIUS:.2f} m")

    print("PATH: required track clearance=" f"{TRACK_CLEARANCE:.2f} m")

    # ========================================================
    # CONNECT CONES
    # ========================================================

    print("PATH: detecting and linking " "cone barriers...")

    obstacles = link_cones(state)

    # ========================================================
    # RVIZ PUBLICATION
    # ========================================================

    publish_connected_cone_map(
        state,
        obstacles,
        connected_cone_map_pub,
    )

    # ========================================================
    # CLEARANCE
    # ========================================================

    print("PATH: building exact Euclidean " "clearance map...")

    clearance = build_euclidean_clearance(state)

    print("PATH: clearance map complete")

    start_index = index(
        width,
        start_x,
        start_y,
    )

    start_clearance = clearance[start_index]

    print("PATH: start clearance=" f"{start_clearance:.2f} m")

    if start_clearance < TRACK_CLEARANCE:
        print("PATH: robot does not have " "enough track clearance")

        return []

    # ========================================================
    # SAFE TRACK
    # ========================================================

    safe = build_safe_map(
        obstacles,
        clearance,
        width,
        height,
        TRACK_CLEARANCE,
    )

    component = find_track_component(
        safe,
        width,
        height,
        start_x,
        start_y,
    )

    print("PATH: safe track cells=" f"{len(component)}")

    if not component:
        print("PATH: no safe track component")

        return []

    # ========================================================
    # CLOSED LAP
    # ========================================================

    print("PATH: finding closed lap...")

    path = find_closed_lap(
        state,
        safe,
        component,
        clearance,
    )

    if path is None:
        print("PATH: no closed lap found")

        return []

    raw_length = path_length(path)

    print("PATH: raw lap length=" f"{raw_length:.2f} m")

    # ========================================================
    # SMOOTH
    # ========================================================

    print("PATH: smoothing closed lap...")

    path = smooth_closed(
        path,
        SMOOTH_WINDOW,
    )

    # ========================================================
    # RESTORE CLEARANCE
    # ========================================================

    print("PATH: enforcing " f"{TRACK_CLEARANCE:.2f} m clearance...")

    path = enforce_clearance(
        state,
        path,
        clearance,
        TRACK_CLEARANCE,
        CLEARANCE_ITERATIONS,
    )

    # ========================================================
    # RESAMPLE
    # ========================================================

    print("PATH: resampling at=" f"{PATH_SPACING:.2f} m")

    path = resample_closed(
        path,
        PATH_SPACING,
    )

    # ========================================================
    # ROTATE TO ROBOT
    # ========================================================

    path = rotate_path_to_start(
        state,
        path,
    )

    if len(path) >= 2 and path[0] != path[-1]:
        path.append(path[0])

    print("PATH: final candidate points=" f"{len(path)}")

    # ========================================================
    # CLEARANCE REPORT
    # ========================================================

    tightest = float("inf")

    for x, y in path:

        room = clearance_at_world(
            state,
            clearance,
            x,
            y,
        )

        tightest = min(
            tightest,
            room,
        )

    print("PATH: tightest centreline clearance=" f"{tightest:.3f} m")

    # ========================================================
    # FINAL FOOTPRINT VALIDATION
    # ========================================================

    print("PATH: validating robot footprint...")

    if not validate_path(
        state,
        path,
        ROBOT_RADIUS,
    ):
        print("PATH: FINAL VALIDATION FAILED")

        return []

    print("PATH: FINAL VALIDATION PASSED")

    # ========================================================
    # FINAL LENGTH
    # ========================================================

    total_length = path_length(path)

    print("PATH: final path=" f"{len(path)} points")

    print("PATH: total length=" f"{total_length:.2f} m")

    print("PATH: start=" f"{path[0]}")

    print("PATH: end=" f"{path[-1]}")

    return path
