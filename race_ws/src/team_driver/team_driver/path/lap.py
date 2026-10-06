import heapq
import math
from collections import deque

from .clearance import is_safe
from .map import in_bounds, index, map_to_world, world_to_map

FINISH_LINE_LENGTH = 4.0


NEIGHBOURS = (
    (-1, 0, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (-1, -1, 1.41421356237),
    (-1, 1, 1.41421356237),
    (1, -1, 1.41421356237),
    (1, 1, 1.41421356237),
)


def find_track_component(
    safe,
    width,
    height,
    start_x,
    start_y,
):
    """
    Find the safe region containing the robot.
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

    queue.append(
        (
            start_x,
            start_y,
        )
    )

    component.add(
        (
            start_x,
            start_y,
        )
    )

    directions = (
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    )

    while queue:

        x, y = queue.popleft()

        for dx, dy in directions:

            nx = x + dx
            ny = y + dy

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


def make_virtual_finish_line(
    state,
):
    """
    Finish line perpendicular to robot heading.
    """

    x = state.position[0]
    y = state.position[1]

    heading_x = math.cos(state.yaw)

    heading_y = math.sin(state.yaw)

    line_x = -heading_y
    line_y = heading_x

    half = FINISH_LINE_LENGTH / 2.0

    a = (
        x - line_x * half,
        y - line_y * half,
    )

    b = (
        x + line_x * half,
        y + line_y * half,
    )

    return a, b


def rasterize_line(
    state,
    a,
    b,
):
    """
    Convert a world-coordinate line into map cells.
    """

    width = state.map.info.width
    height = state.map.info.height
    resolution = state.map.info.resolution

    cut = set()

    distance = math.dist(
        a,
        b,
    )

    samples = max(
        2,
        int(distance / (resolution / 3.0)),
    )

    for i in range(samples + 1):
        t = i / samples

        x = a[0] + t * (b[0] - a[0])

        y = a[1] + t * (b[1] - a[1])

        mx, my = world_to_map(
            state,
            x,
            y,
        )

        for dy in (
            -1,
            0,
            1,
        ):
            for dx in (
                -1,
                0,
                1,
            ):
                nx = mx + dx
                ny = my + dy

                if not in_bounds(
                    width,
                    height,
                    nx,
                    ny,
                ):
                    continue

                cut.add(
                    index(
                        width,
                        nx,
                        ny,
                    )
                )

    return cut


def line_side(
    a,
    b,
    point,
):
    """
    Signed side of point relative to directed line a -> b.
    """

    return (b[0] - a[0]) * (point[1] - a[1]) - (b[1] - a[1]) * (point[0] - a[0])


def find_closed_lap(
    state,
    safe,
    component,
    clearance,
):
    """
    Find a closed lap using the virtual finish line.
    """

    width = state.map.info.width
    height = state.map.info.height
    resolution = state.map.info.resolution

    line_a, line_b = make_virtual_finish_line(state)

    print(
        "PATH: virtual finish line="
        f"({line_a[0]:.2f}, "
        f"{line_a[1]:.2f}) -> "
        f"({line_b[0]:.2f}, "
        f"{line_b[1]:.2f})"
    )

    cut = rasterize_line(
        state,
        line_a,
        line_b,
    )

    heading_x = math.cos(state.yaw)

    heading_y = math.sin(state.yaw)

    starts = set()
    goals = set()

    for cut_index in cut:

        row = cut_index // width
        col = cut_index % width

        for dr, dc, _ in NEIGHBOURS:

            rr = row + dr
            cc = col + dc

            if not in_bounds(
                width,
                height,
                cc,
                rr,
            ):
                continue

            cell_index = index(
                width,
                cc,
                rr,
            )

            if cell_index in cut:
                continue

            if not is_safe(
                safe,
                width,
                height,
                cc,
                rr,
            ):
                continue

            if (
                cc,
                rr,
            ) not in component:
                continue

            world = map_to_world(
                state,
                cc,
                rr,
            )

            side = line_side(
                line_a,
                line_b,
                world,
            )

            normal_x = -(line_b[1] - line_a[1])

            normal_y = line_b[0] - line_a[0]

            normal_length = math.hypot(
                normal_x,
                normal_y,
            )

            if normal_length <= 1.0e-9:
                continue

            normal_x /= normal_length
            normal_y /= normal_length

            forward_side = normal_x * heading_x + normal_y * heading_y

            if side * forward_side > 0.0:
                starts.add(cell_index)

            elif side * forward_side < 0.0:
                goals.add(cell_index)

    print(f"PATH: lap start cells=" f"{len(starts)}")

    print(f"PATH: lap goal cells=" f"{len(goals)}")

    if not starts or not goals:
        print("PATH: could not find both sides " "of virtual finish line")

        return None

    infinity = float("inf")

    best = {}
    previous = {}

    queue = []

    for start in starts:

        best[start] = 0.0
        previous[start] = None

        heapq.heappush(
            queue,
            (
                0.0,
                start,
            ),
        )

    found = None

    while queue:

        cost, current = heapq.heappop(queue)

        if cost > best.get(
            current,
            infinity,
        ):
            continue

        if current in goals:
            found = current
            break

        row = current // width
        col = current % width

        for dr, dc, step_cost in NEIGHBOURS:

            rr = row + dr
            cc = col + dc

            if not in_bounds(
                width,
                height,
                cc,
                rr,
            ):
                continue

            next_index = index(
                width,
                cc,
                rr,
            )

            if next_index in cut:
                continue

            if not is_safe(
                safe,
                width,
                height,
                cc,
                rr,
            ):
                continue

            if (
                cc,
                rr,
            ) not in component:
                continue

            cell_clearance = clearance[next_index]

            penalty = 1.2 / max(
                cell_clearance,
                0.05,
            )

            next_cost = cost + step_cost * resolution * (1.0 + penalty)

            if next_cost < best.get(
                next_index,
                infinity,
            ):
                best[next_index] = next_cost

                previous[next_index] = current

                heapq.heappush(
                    queue,
                    (
                        next_cost,
                        next_index,
                    ),
                )

    if found is None:
        print("PATH: no closed lap found")

        return None

    cells = []

    current = found

    while current is not None:

        row = current // width
        col = current % width

        cells.append(
            (
                col,
                row,
            )
        )

        current = previous[current]

    cells.reverse()

    if len(cells) < 3:
        print("PATH: lap contains too few cells")

        return None

    path = [
        map_to_world(
            state,
            x,
            y,
        )
        for x, y in cells
    ]

    path.append(path[0])

    print(f"PATH: raw lap points=" f"{len(path)}")

    return path
