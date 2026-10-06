import math
from collections import deque

from .state import DriverConfig, DriverState

# ============================================================
# MAP HELPERS
# ============================================================


def in_bounds(width, height, x, y):
    return 0 <= x < width and 0 <= y < height


def index(width, x, y):
    return y * width + x


def world_to_map(state, x, y):
    info = state.map.info

    mx = int(math.floor((x - info.origin.position.x) / info.resolution))

    my = int(math.floor((y - info.origin.position.y) / info.resolution))

    return mx, my


def map_to_world(state, x, y):
    info = state.map.info

    return (
        info.origin.position.x + (x + 0.5) * info.resolution,
        info.origin.position.y + (y + 0.5) * info.resolution,
    )


def is_free(grid, width, height, x, y):
    if not in_bounds(
        width,
        height,
        x,
        y,
    ):
        return False

    return grid[index(width, x, y)] == 0


def is_safe(safe, width, height, x, y):
    if not in_bounds(
        width,
        height,
        x,
        y,
    ):
        return False

    return safe[index(width, x, y)] != 0


# ============================================================
# EXACT 1D EUCLIDEAN DISTANCE TRANSFORM
# ============================================================


def distance_transform_1d(values):
    """
    Exact squared Euclidean distance transform.

    values:
        0       = obstacle
        INF     = free

    Returns squared distance to the nearest obstacle.
    """

    n = len(values)

    if n == 0:
        return []

    infinity = 1.0e20

    # v stores locations of parabolas.
    v = [0] * n

    # z stores boundaries between parabolas.
    z = [0.0] * (n + 1)

    # Result.
    d = [0.0] * n

    k = 0

    v[0] = 0
    z[0] = -infinity
    z[1] = infinity

    for q in range(1, n):
        while True:
            p = v[k]

            # If q is effectively infinite, there is no useful
            # parabola to add.
            if values[q] >= infinity:
                break

            numerator = values[q] + q * q - values[p] - p * p

            denominator = 2.0 * (q - p)

            s = numerator / denominator

            if s <= z[k]:
                if k == 0:
                    break

                k -= 1
                continue

            break

        if values[q] < infinity:
            k += 1

            v[k] = q
            z[k] = s
            z[k + 1] = infinity

    k = 0

    for q in range(n):
        while z[k + 1] < q:
            k += 1

        p = v[k]

        d[q] = (q - p) * (q - p) + values[p]

    return d


# ============================================================
# EXACT EUCLIDEAN CLEARANCE
# ============================================================


def build_euclidean_clearance(state):
    """
    Calculate Euclidean distance from every cell to the nearest
    occupied or unknown cell.

    Uses the exact separable squared Euclidean distance transform.
    """

    info = state.map.info

    width = info.width
    height = info.height
    grid = state.map.data

    infinity = 1.0e20

    # --------------------------------------------------------
    # Transform along X.
    # --------------------------------------------------------

    horizontal = [[0.0] * width for _ in range(height)]

    for y in range(height):
        row = [infinity] * width

        for x in range(width):
            if grid[index(width, x, y)] != 0:
                row[x] = 0.0

        horizontal[y] = distance_transform_1d(row)

    # --------------------------------------------------------
    # Transform along Y.
    # --------------------------------------------------------

    squared = [0.0] * (width * height)

    for x in range(width):
        column = [horizontal[y][x] for y in range(height)]

        transformed = distance_transform_1d(column)

        for y in range(height):
            squared[index(width, x, y)] = transformed[y]

    # --------------------------------------------------------
    # Convert cell distance to meters.
    # --------------------------------------------------------

    resolution = info.resolution

    clearance = [0.0] * (width * height)

    for i in range(width * height):
        clearance[i] = (
            math.sqrt(
                max(
                    0.0,
                    squared[i],
                )
            )
            * resolution
        )

    return clearance


# ============================================================
# INFLATED MAP
# ============================================================


def build_safe_map(
    state,
    clearance,
    safety_radius,
):
    """
    Build the map used by the route planner.

    Occupied and unknown cells are blocked.

    Free cells are only usable when their Euclidean clearance
    is at least safety_radius.
    """

    info = state.map.info

    width = info.width
    height = info.height
    grid = state.map.data

    safe = bytearray(width * height)

    for y in range(height):
        for x in range(width):
            i = index(
                width,
                x,
                y,
            )

            if grid[i] != 0:
                continue

            if clearance[i] >= safety_radius:
                safe[i] = 1

    return safe


# ============================================================
# CONNECTED COMPONENT
# ============================================================


def find_track_component(
    safe,
    width,
    height,
    start_x,
    start_y,
):
    """
    Find the connected safe region containing the robot.
    """

    if not is_safe(
        safe,
        width,
        height,
        start_x,
        start_y,
    ):
        return set()

    component = set()

    queue = deque()

    start = (
        start_x,
        start_y,
    )

    component.add(start)
    queue.append(start)

    directions = (
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    )

    while queue:
        x, y = queue.popleft()

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy

            if not in_bounds(
                width,
                height,
                nx,
                ny,
            ):
                continue

            if not is_safe(
                safe,
                width,
                height,
                nx,
                ny,
            ):
                continue

            cell = (
                nx,
                ny,
            )

            if cell in component:
                continue

            component.add(cell)
            queue.append(cell)

    return component


# ============================================================
# FAST GRID LINE CHECK
# ============================================================


def grid_line_is_safe(
    safe,
    width,
    height,
    x0,
    y0,
    x1,
    y1,
):
    """
    Check a line through the inflated map.
    """

    dx = abs(x1 - x0)

    dy = abs(y1 - y0)

    sx = 1 if x0 < x1 else -1

    sy = 1 if y0 < y1 else -1

    error = dx - dy

    while True:
        if not is_safe(
            safe,
            width,
            height,
            x0,
            y0,
        ):
            return False

        if x0 == x1 and y0 == y1:
            return True

        error2 = 2 * error

        if error2 > -dy:
            error -= dy
            x0 += sx

        if error2 < dx:
            error += dx
            y0 += sy


# ============================================================
# NEXT POINT
# ============================================================


def find_next_point(
    state,
    safe,
    clearance,
    component,
    current,
    direction,
    step_cells,
    search_cells,
    search_angle,
):
    """
    Find a good point ahead of the current position.

    This only uses the already-inflated map, so it is cheap.
    """

    width = state.map.info.width
    height = state.map.info.height

    current_x, current_y = current

    direction_x, direction_y = direction

    current_angle = math.atan2(
        direction_y,
        direction_x,
    )

    best_cell = None
    best_score = -float("inf")

    for dy in range(
        -search_cells,
        search_cells + 1,
    ):
        for dx in range(
            -search_cells,
            search_cells + 1,
        ):
            if dx == 0 and dy == 0:
                continue

            nx = current_x + dx
            ny = current_y + dy

            if not in_bounds(
                width,
                height,
                nx,
                ny,
            ):
                continue

            if (nx, ny) not in component:
                continue

            distance = math.hypot(
                dx,
                dy,
            )

            if distance < step_cells:
                continue

            if distance > search_cells:
                continue

            candidate_angle = math.atan2(
                dy,
                dx,
            )

            angle_difference = math.atan2(
                math.sin(candidate_angle - current_angle),
                math.cos(candidate_angle - current_angle),
            )

            angle_error = abs(angle_difference)

            if angle_error > search_angle:
                continue

            if not grid_line_is_safe(
                safe,
                width,
                height,
                current_x,
                current_y,
                nx,
                ny,
            ):
                continue

            candidate_clearance = clearance[
                index(
                    width,
                    nx,
                    ny,
                )
            ]

            # Prefer the middle of the track.
            clearance_score = candidate_clearance * candidate_clearance

            # Prefer forward movement.
            forward_score = math.cos(angle_error)

            # Prefer reasonable step size.
            desired_distance = step_cells * 2.0

            distance_score = -abs(distance - desired_distance)

            score = clearance_score * 10.0 + forward_score * 10.0 + distance_score

            if score > best_score:
                best_score = score
                best_cell = (
                    nx,
                    ny,
                )

    return best_cell


# ============================================================
# FOOTPRINT VALIDATION
# ============================================================


def footprint_is_safe(
    state,
    x,
    y,
    safety_radius,
):
    """
    Check the robot's complete circular safety footprint
    against the ORIGINAL occupancy grid.

    This is intentionally separate from the inflated planning
    map.
    """

    info = state.map.info

    width = info.width
    height = info.height
    resolution = info.resolution
    grid = state.map.data

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

            if not is_free(
                grid,
                width,
                height,
                mx,
                my,
            ):
                return False

    return True


# ============================================================
# SEGMENT FOOTPRINT VALIDATION
# ============================================================


def segment_is_safe(
    state,
    start,
    end,
    safety_radius,
):
    """
    Validate the complete robot footprint along a segment.
    """

    x0, y0 = start
    x1, y1 = end

    distance = math.hypot(
        x1 - x0,
        y1 - y0,
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

        x = x0 + (x1 - x0) * t

        y = y0 + (y1 - y0) * t

        if not footprint_is_safe(
            state,
            x,
            y,
            safety_radius,
        ):
            return False

    return True


# ============================================================
# PATH RESAMPLING
# ============================================================


def resample_path(
    path,
    spacing=0.05,
):
    """
    Convert a sparse route into approximately equally spaced
    points.
    """

    if len(path) < 2:
        return path

    result = [path[0]]

    previous_x, previous_y = path[0]

    accumulated = 0.0

    for current_x, current_y in path[1:]:
        dx = current_x - previous_x
        dy = current_y - previous_y

        segment_length = math.hypot(
            dx,
            dy,
        )

        if segment_length <= 0.0:
            continue

        while accumulated + segment_length >= spacing:
            remaining = spacing - accumulated

            ratio = remaining / segment_length

            new_x = previous_x + dx * ratio

            new_y = previous_y + dy * ratio

            result.append(
                (
                    new_x,
                    new_y,
                )
            )

            previous_x = new_x
            previous_y = new_y

            dx = current_x - previous_x

            dy = current_y - previous_y

            segment_length = math.hypot(
                dx,
                dy,
            )

            accumulated = 0.0

        accumulated += segment_length

        previous_x = current_x
        previous_y = current_y

    if result[-1] != path[-1]:
        result.append(path[-1])

    return result


# ============================================================
# FINAL VALIDATION
# ============================================================


def validate_path(
    state,
    path,
    safety_radius,
):
    """
    Validate every path point and every path segment.
    """

    if len(path) < 2:
        return False

    # Check every point.
    for i, point in enumerate(path):
        if not footprint_is_safe(
            state,
            point[0],
            point[1],
            safety_radius,
        ):
            print("PATH: footprint validation failed " f"at point {i}")

            return False

    # Check every segment.
    for i in range(len(path) - 1):
        if not segment_is_safe(
            state,
            path[i],
            path[i + 1],
            safety_radius,
        ):
            print("PATH: segment validation failed " f"between points {i} and {i + 1}")

            return False

    return True


# ============================================================
# PATH GENERATION
# ============================================================


def construct_path(
    state: DriverState,
):
    """
    Generate a safe closed race-track path.
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
    grid = state.map.data

    # ========================================================
    # ROBOT POSITION
    # ========================================================

    start_x, start_y = world_to_map(
        state,
        state.position[0],
        state.position[1],
    )

    print(f"PATH: robot cell=" f"({start_x}, {start_y})")

    if not is_free(
        grid,
        width,
        height,
        start_x,
        start_y,
    ):
        print("PATH: robot is not in free space")
        return []

    # ========================================================
    # SAFETY
    # ========================================================

    robot_radius = 0.17
    safety_margin = 0.08

    safety_radius = robot_radius + safety_margin

    print(f"PATH: robot radius=" f"{robot_radius:.2f} m")

    print(f"PATH: safety margin=" f"{safety_margin:.2f} m")

    print(f"PATH: total safety radius=" f"{safety_radius:.2f} m")

    # ========================================================
    # EUCLIDEAN DISTANCE
    # ========================================================

    print("PATH: building exact Euclidean " "clearance map...")

    clearance = build_euclidean_clearance(state)

    print("PATH: clearance map complete")

    # ========================================================
    # INFLATE OBSTACLES
    # ========================================================

    safe = build_safe_map(
        state,
        clearance,
        safety_radius,
    )

    # ========================================================
    # CHECK START
    # ========================================================

    if not is_safe(
        safe,
        width,
        height,
        start_x,
        start_y,
    ):
        print("PATH: robot does not have enough " "clearance")
        return []

    # ========================================================
    # CONNECTED COMPONENT
    # ========================================================

    component = find_track_component(
        safe,
        width,
        height,
        start_x,
        start_y,
    )

    print(f"PATH: safe track cells=" f"{len(component)}")

    if not component:
        print("PATH: no safe component")
        return []

    # ========================================================
    # ROUTE PARAMETERS
    # ========================================================

    step_distance = 0.15
    search_distance = 0.80

    step_cells = max(
        1,
        int(round(step_distance / resolution)),
    )

    search_cells = max(
        step_cells + 1,
        int(round(search_distance / resolution)),
    )

    search_angle = math.radians(75.0)

    print(f"PATH: route step=" f"{step_distance:.2f} m")

    print(f"PATH: route search=" f"{search_distance:.2f} m")

    # ========================================================
    # INITIAL DIRECTION
    # ========================================================

    direction = (
        math.cos(state.yaw),
        math.sin(state.yaw),
    )

    current = (
        start_x,
        start_y,
    )

    path_cells = [current]

    travelled_cells = 0.0

    # Need to travel a meaningful distance before closing.
    minimum_loop_distance = 3.0

    minimum_loop_cells = int(minimum_loop_distance / resolution)

    max_iterations = 1500

    loop_closed = False

    # ========================================================
    # GENERATE ROUTE
    # ========================================================

    for iteration in range(max_iterations):
        distance_from_start = math.hypot(
            current[0] - start_x,
            current[1] - start_y,
        )

        # ----------------------------------------------------
        # Try to close the loop.
        # ----------------------------------------------------

        if (
            travelled_cells >= minimum_loop_cells
            and distance_from_start <= search_cells
        ):
            if grid_line_is_safe(
                safe,
                width,
                height,
                current[0],
                current[1],
                start_x,
                start_y,
            ):
                path_cells.append(
                    (
                        start_x,
                        start_y,
                    )
                )

                loop_closed = True

                print("PATH: complete loop closed")

                break

        # ----------------------------------------------------
        # Find next point.
        # ----------------------------------------------------

        next_cell = find_next_point(
            state,
            safe,
            clearance,
            component,
            current,
            direction,
            step_cells,
            search_cells,
            search_angle,
        )

        if next_cell is None:
            print("PATH: planner became stuck")
            break

        next_x, next_y = next_cell

        # ----------------------------------------------------
        # Update travel distance.
        # ----------------------------------------------------

        travelled_cells += math.hypot(
            next_x - current[0],
            next_y - current[1],
        )

        # ----------------------------------------------------
        # Update direction.
        # ----------------------------------------------------

        dx = next_x - current[0]
        dy = next_y - current[1]

        distance = math.hypot(
            dx,
            dy,
        )

        if distance <= 0.0:
            break

        direction = (
            dx / distance,
            dy / distance,
        )

        # ----------------------------------------------------
        # Move.
        # ----------------------------------------------------

        current = (
            next_x,
            next_y,
        )

        path_cells.append(current)

        # ----------------------------------------------------
        # Progress.
        # ----------------------------------------------------

        if iteration > 0 and iteration % 100 == 0:
            print(
                f"PATH: iteration="
                f"{iteration}, "
                f"points="
                f"{len(path_cells)}, "
                f"distance="
                f"{travelled_cells * resolution:.2f} m"
            )

    if not loop_closed:
        print("PATH: WARNING - loop did not close")

    # ========================================================
    # BASIC CHECK
    # ========================================================

    if len(path_cells) < 3:
        print("PATH: too few route points")
        return []

    # ========================================================
    # WORLD COORDINATES
    # ========================================================

    path = [
        map_to_world(
            state,
            x,
            y,
        )
        for x, y in path_cells
    ]

    # ========================================================
    # RESAMPLE
    # ========================================================

    path = resample_path(
        path,
        spacing=0.05,
    )

    print(f"PATH: resampled points=" f"{len(path)}")

    # ========================================================
    # FINAL FOOTPRINT VALIDATION
    # ========================================================

    print("PATH: validating robot footprint...")

    if not validate_path(
        state,
        path,
        safety_radius,
    ):
        print("PATH: FINAL VALIDATION FAILED")
        return []

    print("PATH: FINAL VALIDATION PASSED")

    # ========================================================
    # PATH LENGTH
    # ========================================================

    total_length = 0.0

    for i in range(len(path) - 1):
        total_length += math.hypot(
            path[i + 1][0] - path[i][0],
            path[i + 1][1] - path[i][1],
        )

    print(f"PATH: final path=" f"{len(path)} points")

    print(f"PATH: total length=" f"{total_length:.2f} m")

    print(f"PATH: start=" f"{path[0]}")

    print(f"PATH: end=" f"{path[-1]}")

    return path


# ============================================================
# CONTROL
# ============================================================


def compute_control(
    state: DriverState,
    config: DriverConfig,
):
    """
    Follow the generated closed path using Pure Pursuit.

    Lookahead adapts to:
        - current vehicle speed
        - current steering demand

    Speed is reduced when steering demand increases.
    """

    if state.position is None:
        return 0.0, 0.0

    if state.path is None or len(state.path) < 2:
        return 0.0, 0.0

    x, y = state.position
    yaw = state.yaw

    path = state.path
    path_length = len(path)

    # ------------------------------------------------------------
    # 1. Find the closest point on the closed path.
    # ------------------------------------------------------------

    closest_index = 0
    closest_distance_sq = float("inf")

    for i, (px, py) in enumerate(path):
        dx = px - x
        dy = py - y

        distance_sq = dx * dx + dy * dy

        if distance_sq < closest_distance_sq:
            closest_distance_sq = distance_sq
            closest_index = i

    # ------------------------------------------------------------
    # 2. Estimate steering using a preliminary lookahead.
    #
    # This gives us a steering estimate that can be used to
    # adjust the actual lookahead distance.
    # ------------------------------------------------------------

    preliminary_lookahead = config.min_lookahead

    target_index = closest_index
    accumulated_distance = 0.0

    for _ in range(path_length):
        next_index = (target_index + 1) % path_length

        x0, y0 = path[target_index]
        x1, y1 = path[next_index]

        segment_length = math.hypot(
            x1 - x0,
            y1 - y0,
        )

        if segment_length <= 1e-9:
            target_index = next_index
            continue

        accumulated_distance += segment_length
        target_index = next_index

        if accumulated_distance >= preliminary_lookahead:
            break

    target_x, target_y = path[target_index]

    # ------------------------------------------------------------
    # 3. Transform target into vehicle coordinates.
    # ------------------------------------------------------------

    dx = target_x - x
    dy = target_y - y

    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    target_forward = cos_yaw * dx + sin_yaw * dy

    target_left = -sin_yaw * dx + cos_yaw * dy

    target_distance_sq = target_forward * target_forward + target_left * target_left

    if target_distance_sq < 1e-6:
        return 0.0, state.speed

    # ------------------------------------------------------------
    # 4. Preliminary Pure Pursuit steering.
    # ------------------------------------------------------------

    curvature = 2.0 * target_left / target_distance_sq

    preliminary_steering = math.atan(config.wheelbase * curvature)

    # ------------------------------------------------------------
    # 5. Adaptive lookahead.
    #
    # Higher speed:
    #     -> longer lookahead
    #
    # Higher steering demand:
    #     -> shorter lookahead
    # ------------------------------------------------------------

    speed = abs(state.speed)

    speed_lookahead = config.min_lookahead + config.lookahead_speed_gain * speed

    steering_ratio = min(
        1.0,
        abs(preliminary_steering) / config.max_steering,
    )

    steering_factor = (
        1.0 - config.lookahead_steering_reduction * steering_ratio * steering_ratio
    )

    lookahead = speed_lookahead * steering_factor

    lookahead = max(
        config.min_lookahead,
        min(
            config.max_lookahead,
            lookahead,
        ),
    )

    # ------------------------------------------------------------
    # 6. Find the final lookahead target on the closed path.
    # ------------------------------------------------------------

    target_index = closest_index
    accumulated_distance = 0.0

    for _ in range(path_length):
        next_index = (target_index + 1) % path_length

        x0, y0 = path[target_index]
        x1, y1 = path[next_index]

        segment_length = math.hypot(
            x1 - x0,
            y1 - y0,
        )

        if segment_length <= 1e-9:
            target_index = next_index
            continue

        accumulated_distance += segment_length
        target_index = next_index

        if accumulated_distance >= lookahead:
            break

    target_x, target_y = path[target_index]

    # ------------------------------------------------------------
    # 7. Transform final target into vehicle coordinates.
    # ------------------------------------------------------------

    dx = target_x - x
    dy = target_y - y

    target_forward = cos_yaw * dx + sin_yaw * dy

    target_left = -sin_yaw * dx + cos_yaw * dy

    target_distance_sq = target_forward * target_forward + target_left * target_left

    if target_distance_sq < 1e-6:
        return 0.0, state.speed

    # ------------------------------------------------------------
    # 8. Final Pure Pursuit steering.
    # ------------------------------------------------------------

    curvature = 2.0 * target_left / target_distance_sq

    steering = math.atan(config.wheelbase * curvature)

    steering = max(
        -config.max_steering,
        min(
            config.max_steering,
            steering,
        ),
    )

    # ------------------------------------------------------------
    # 9. Speed control.
    #
    # Slow down for larger steering demands.
    # ------------------------------------------------------------

    steering_ratio = min(
        1.0,
        abs(steering) / config.max_steering,
    )

    speed_factor = (
        1.0 - config.steering_speed_reduction * steering_ratio * steering_ratio
    )

    target_speed = (
        config.min_speed + (config.max_speed - config.min_speed) * speed_factor
    )

    target_speed = max(
        config.min_speed,
        min(
            config.max_speed,
            target_speed,
        ),
    )

    return steering, target_speed
