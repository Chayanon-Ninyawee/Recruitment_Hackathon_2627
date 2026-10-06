import math
from collections import deque

from nav_msgs.msg import OccupancyGrid

from .map import in_bounds, index, is_obstacle, map_to_world, world_to_map

CONE_LINK_DISTANCE = 1.10

CONE_MAX_SPAN = 0.90
CONE_MAX_CELLS = 400


def find_cone_components(
    state,
):
    """
    Find small isolated occupied components.

    Matches the reference tool:

        max_span = 0.9 m
        max_cells = 400
    """

    info = state.map.info

    width = info.width
    height = info.height

    visited = bytearray(width * height)

    cones = []

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

    for start_y in range(height):
        for start_x in range(width):

            start_index = index(
                width,
                start_x,
                start_y,
            )

            if visited[start_index]:
                continue

            if not is_obstacle(
                state,
                start_x,
                start_y,
            ):
                continue

            queue = deque()

            queue.append(
                (
                    start_x,
                    start_y,
                )
            )

            visited[start_index] = 1

            cells = []

            min_x = start_x
            max_x = start_x

            min_y = start_y
            max_y = start_y

            while queue:
                x, y = queue.popleft()

                cells.append(
                    (
                        x,
                        y,
                    )
                )

                min_x = min(
                    min_x,
                    x,
                )

                max_x = max(
                    max_x,
                    x,
                )

                min_y = min(
                    min_y,
                    y,
                )

                max_y = max(
                    max_y,
                    y,
                )

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

                    ni = index(
                        width,
                        nx,
                        ny,
                    )

                    if visited[ni]:
                        continue

                    if not is_obstacle(
                        state,
                        nx,
                        ny,
                    ):
                        continue

                    visited[ni] = 1

                    queue.append(
                        (
                            nx,
                            ny,
                        )
                    )

            if len(cells) > CONE_MAX_CELLS:
                continue

            span_cells = max(
                max_x - min_x + 1,
                max_y - min_y + 1,
            )

            span = span_cells * info.resolution

            if not (0.1 < span < CONE_MAX_SPAN):
                continue

            centre_x = (min_x + max_x) // 2

            centre_y = (min_y + max_y) // 2

            cones.append(
                map_to_world(
                    state,
                    centre_x,
                    centre_y,
                )
            )

    return cones


def draw_barrier(
    state,
    obstacles,
    a,
    b,
):
    """
    Draw a solid barrier between two cone centres.

    Uses the reference implementation's:
        resolution / 2
    sampling and 3x3 thickening.
    """

    width = state.map.info.width
    height = state.map.info.height
    resolution = state.map.info.resolution

    distance = math.dist(
        a,
        b,
    )

    steps = max(
        2,
        int(distance / (resolution / 2.0)),
    )

    for i in range(steps + 1):
        t = i / steps

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

                obstacles[
                    index(
                        width,
                        nx,
                        ny,
                    )
                ] = True


def link_cones(
    state,
):
    """
    Connect every pair of cones <= 1.1 m apart.

    Returns a boolean obstacle map.
    """

    cones = find_cone_components(state)

    print(f"PATH: detected cone components=" f"{len(cones)}")

    width = state.map.info.width
    height = state.map.info.height

    obstacles = bytearray(width * height)

    for y in range(height):
        for x in range(width):

            if is_obstacle(
                state,
                x,
                y,
            ):
                obstacles[
                    index(
                        width,
                        x,
                        y,
                    )
                ] = 1

    links = 0
    longest_link = 0.0

    for i, a in enumerate(cones):
        for b in cones[i + 1 :]:

            distance = math.dist(
                a,
                b,
            )

            if distance > CONE_LINK_DISTANCE:
                continue

            draw_barrier(
                state,
                obstacles,
                a,
                b,
            )

            links += 1

            longest_link = max(
                longest_link,
                distance,
            )

    print(f"PATH: cone barriers linked=" f"{links}")

    print(f"PATH: cone link distance=" f"{CONE_LINK_DISTANCE:.2f} m")

    print(f"PATH: longest cone link=" f"{longest_link:.2f} m")

    barrier_cells = sum(1 for value in obstacles if value)

    print(f"PATH: barrier cells=" f"{barrier_cells}")

    return obstacles


def publish_connected_cone_map(
    state,
    obstacles,
    publisher,
):
    """
    Publish the connected cone/barrier map.

    100 = obstacle
    0   = free
    """

    if publisher is None:
        return

    original = state.map

    msg = OccupancyGrid()

    msg.header = original.header
    msg.info = original.info

    data = [0] * (original.info.width * original.info.height)

    for i in range(len(data)):
        if obstacles[i]:
            data[i] = 100
        else:
            data[i] = 0

    msg.data = data

    publisher.publish(msg)
