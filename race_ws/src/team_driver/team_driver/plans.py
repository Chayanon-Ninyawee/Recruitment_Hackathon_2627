import math

from .state import DriverConfig, DriverState


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
