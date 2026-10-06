import math

from .map import in_bounds, index, is_obstacle


def distance_transform_1d(
    values,
):
    """
    Exact squared Euclidean distance transform.

    0   = obstacle
    INF = free
    """

    n = len(values)

    if n == 0:
        return []

    infinity = 1.0e20

    v = [0] * n
    z = [0.0] * (n + 1)
    output = [0.0] * n

    k = 0

    v[0] = 0

    z[0] = -infinity
    z[1] = infinity

    for q in range(
        1,
        n,
    ):
        while True:
            p = v[k]

            if values[q] >= infinity and values[p] >= infinity:
                s = infinity

            elif values[p] >= infinity:
                s = -infinity

            elif values[q] >= infinity:
                s = infinity

            else:
                numerator = values[q] + q * q - values[p] - p * p

                denominator = 2.0 * (q - p)

                s = numerator / denominator

            if s <= z[k]:
                if k == 0:
                    break

                k -= 1
                continue

            break

        k += 1

        v[k] = q
        z[k] = s
        z[k + 1] = infinity

    k = 0

    for q in range(n):

        while z[k + 1] < q:
            k += 1

        p = v[k]

        if values[p] >= infinity:
            output[q] = infinity
        else:
            output[q] = (q - p) * (q - p) + values[p]

    return output


def build_euclidean_clearance(
    state,
):
    """
    Exact Euclidean distance from every cell
    to the nearest occupied cell.
    """

    info = state.map.info

    width = info.width
    height = info.height

    infinity = 1.0e20

    intermediate = [[0.0] * width for _ in range(height)]

    # Vertical pass.
    for x in range(width):

        column = [infinity] * height

        for y in range(height):

            if is_obstacle(
                state,
                x,
                y,
            ):
                column[y] = 0.0

        transformed = distance_transform_1d(column)

        for y in range(height):
            intermediate[y][x] = transformed[y]

    # Horizontal pass.
    squared = [0.0] * (width * height)

    for y in range(height):

        transformed = distance_transform_1d(intermediate[y])

        for x in range(width):

            squared[
                index(
                    width,
                    x,
                    y,
                )
            ] = transformed[x]

    # Convert to metres.
    resolution = info.resolution

    clearance = [0.0] * (width * height)

    for i in range(width * height):
        value = squared[i]

        if value >= infinity:
            clearance[i] = float("inf")
        else:
            clearance[i] = (
                math.sqrt(
                    max(
                        0.0,
                        value,
                    )
                )
                * resolution
            )

    return clearance


def build_safe_map(
    obstacles,
    clearance,
    width,
    height,
    minimum_clearance,
):
    """
    A cell is safe when it is not an obstacle
    and has enough clearance.
    """

    safe = bytearray(width * height)

    for i in range(width * height):
        if obstacles[i]:
            continue

        if clearance[i] >= minimum_clearance:
            safe[i] = 1

    return safe


def is_safe(
    safe,
    width,
    height,
    x,
    y,
):
    if not in_bounds(
        width,
        height,
        x,
        y,
    ):
        return False

    return bool(
        safe[
            index(
                width,
                x,
                y,
            )
        ]
    )
