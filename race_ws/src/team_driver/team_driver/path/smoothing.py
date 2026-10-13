import math

from .map import in_bounds, index, world_to_map


def path_length(
    path,
):
    if len(path) < 2:
        return 0.0

    total = 0.0

    for i in range(len(path) - 1):
        total += math.dist(
            path[i],
            path[i + 1],
        )

    return total


def smooth_closed(
    path,
    window,
):
    """
    Moving-average smoothing around a closed loop.
    """

    if window < 2 or len(path) < window:
        return path

    if len(path) >= 2 and path[0] == path[-1]:
        points = path[:-1]
    else:
        points = list(path)

    n = len(points)

    if n < window:
        return path

    half = window // 2

    result = []

    for i in range(n):

        sx = 0.0
        sy = 0.0
        count = 0

        for k in range(
            -half,
            half + 1,
        ):
            px, py = points[(i + k) % n]

            sx += px
            sy += py

            count += 1

        result.append(
            (
                sx / count,
                sy / count,
            )
        )

    result.append(result[0])

    return result


def clearance_at_world(
    state,
    clearance,
    x,
    y,
):
    mx, my = world_to_map(
        state,
        x,
        y,
    )

    width = state.map.info.width
    height = state.map.info.height

    if not in_bounds(
        width,
        height,
        mx,
        my,
    ):
        return 0.0

    return clearance[
        index(
            width,
            mx,
            my,
        )
    ]


def enforce_clearance(
    state,
    path,
    clearance,
    target,
    iterations=60,
):
    """
    Push points away from walls when smoothing
    reduces their clearance.
    """

    if len(path) < 4:
        return path

    if path[0] == path[-1]:
        points = path[:-1]
    else:
        points = list(path)

    resolution = state.map.info.resolution

    probes = []

    for k in range(16):

        angle = k * math.pi / 8.0

        probes.append(
            (
                math.cos(angle),
                math.sin(angle),
            )
        )

    for _ in range(iterations):

        moved = False

        for i, (
            x,
            y,
        ) in enumerate(points):

            room = clearance_at_world(
                state,
                clearance,
                x,
                y,
            )

            if room >= target:
                continue

            best_x = x
            best_y = y
            best_room = room

            for dx, dy in probes:

                px = x + dx * resolution

                py = y + dy * resolution

                probe_room = clearance_at_world(
                    state,
                    clearance,
                    px,
                    py,
                )

                if probe_room > best_room:
                    best_room = probe_room

                    best_x = px
                    best_y = py

            if best_room > room:

                points[i] = (
                    best_x,
                    best_y,
                )

                moved = True

        if not moved:
            break

        points = smooth_closed(
            points,
            5,
        )

        if points[-1] == points[0]:
            points = points[:-1]

    result = list(points)

    result.append(result[0])

    return result


def resample_closed(
    path,
    spacing,
):
    """
    Even spacing around a closed loop.
    """

    if len(path) < 2:
        return path

    if path[0] == path[-1]:
        points = path[:-1]
    else:
        points = list(path)

    if len(points) < 2:
        return path

    loop = points + [points[0]]

    total = path_length(loop)

    if total <= 0.0:
        return path

    count = max(
        8,
        int(total / spacing),
    )

    result = []

    target = 0.0
    travelled = 0.0
    segment_index = 0

    for _ in range(count):

        while (
            segment_index < len(loop) - 2
            and travelled
            + math.dist(
                loop[segment_index],
                loop[segment_index + 1],
            )
            < target
        ):
            travelled += math.dist(
                loop[segment_index],
                loop[segment_index + 1],
            )

            segment_index += 1

        a = loop[segment_index]

        b = loop[segment_index + 1]

        segment_length = math.dist(
            a,
            b,
        )

        if segment_length <= 1.0e-9:
            result.append(a)
            continue

        ratio = (target - travelled) / segment_length

        ratio = max(
            0.0,
            min(
                1.0,
                ratio,
            ),
        )

        result.append(
            (
                a[0] + ratio * (b[0] - a[0]),
                a[1] + ratio * (b[1] - a[1]),
            )
        )

        target += total / count

    return result


def rotate_path_to_start(
    state,
    path,
):
    """
    Rotate the closed path so the point nearest
    the robot is first.

    Then orient the path according to robot yaw.
    """

    if len(path) < 2:
        return path

    points = list(path)

    if points[0] == points[-1]:
        points = points[:-1]

    if len(points) < 2:
        return path

    robot_x = state.position[0]
    robot_y = state.position[1]

    closest = 0
    closest_distance = float("inf")

    for i, (
        x,
        y,
    ) in enumerate(points):

        distance = math.hypot(
            x - robot_x,
            y - robot_y,
        )

        if distance < closest_distance:
            closest_distance = distance

            closest = i

    points = points[closest:] + points[:closest]

    if len(points) >= 3:

        next_x = points[1][0]
        next_y = points[1][1]

        dx = next_x - points[0][0]

        dy = next_y - points[0][1]

        desired_x = math.cos(state.yaw)

        desired_y = math.sin(state.yaw)

        dot = dx * desired_x + dy * desired_y

        if dot < 0.0:

            points.reverse()

            closest_point = points[0]

            closest_index = points.index(closest_point)

            points = points[closest_index:] + points[:closest_index]

    return points


def straighten_closed(
    state,
    path,
    clearance,
    target_clearance,
    spacing=0.05,
    max_span=8.0,
    min_deviation=0.15,
    sample_spacing=0.025,
    angle_limit=0.35,
):
    """
    Replace wavy path sections with straight chords
    when the complete chord stays inside safe space.
    """

    if len(path) < 5:
        return path

    if path[0] == path[-1]:
        points = list(path[:-1])
    else:
        points = list(path)

    n = len(points)

    if n < 5:
        return path

    resolution = state.map.info.resolution
    max_steps = min(n - 2, max(2, int(max_span / spacing)))

    def distance(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def direction(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.hypot(dx, dy)

        if length < 1e-9:
            return None

        return dx / length, dy / length

    def aligned(a, b):
        return (
            a is not None
            and b is not None
            and a[0] * b[0] + a[1] * b[1] >= math.cos(angle_limit)
        )

    def chord_is_safe(a, b):
        length = distance(a, b)
        samples = max(
            1,
            math.ceil(length / min(sample_spacing, resolution * 0.5)),
        )

        for k in range(samples + 1):
            t = k / samples
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])

            if clearance_at_world(state, clearance, x, y) < target_clearance:
                return False

        return True

    def max_chord_deviation(start, end, span):
        chord = direction(start, end)

        if chord is None:
            return 0.0

        maximum = 0.0
        length_squared = (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2

        if length_squared < 1e-12:
            return 0.0

        for k in range(1, span):
            p = points[(i + k) % n]

            t = (
                (p[0] - start[0]) * (end[0] - start[0])
                + (p[1] - start[1]) * (end[1] - start[1])
            ) / length_squared

            t = max(0.0, min(1.0, t))

            projected = (
                start[0] + t * (end[0] - start[0]),
                start[1] + t * (end[1] - start[1]),
            )

            maximum = max(maximum, distance(p, projected))

        return maximum

    result = []
    i = 0
    replacements = 0

    while i < n:
        best_span = 1

        start = points[i]
        incoming = direction(points[(i - 1) % n], start)

        for span in range(max_steps, 1, -1):
            if i + span > n:
                continue

            j = (i + span) % n
            end = points[j]

            chord_direction = direction(start, end)
            outgoing = direction(end, points[(j + 1) % n])

            # Only replace sections whose entry and exit
            # directions are consistent with a straight line.
            if not aligned(incoming, chord_direction):
                continue

            if not aligned(chord_direction, outgoing):
                continue

            deviation = max_chord_deviation(start, end, span)

            if deviation < min_deviation:
                continue

            if not chord_is_safe(start, end):
                continue

            best_span = span
            break

        if best_span == 1:
            result.append(start)
            i += 1
            continue

        end = points[(i + best_span) % n]
        length = distance(start, end)
        segments = max(1, math.ceil(length / spacing))

        for k in range(1, segments + 1):
            # The final endpoint is the first point of the
            # next section. Don't duplicate the loop start.
            if i + best_span == n and k == segments:
                continue

            t = k / segments
            result.append(
                (
                    start[0] + t * (end[0] - start[0]),
                    start[1] + t * (end[1] - start[1]),
                )
            )

        replacements += 1
        i += best_span

    print(
        "PATH: straight-section cleanup="
        f"replacements={replacements}, "
        f"points={n}->{len(result)}"
    )

    return result
