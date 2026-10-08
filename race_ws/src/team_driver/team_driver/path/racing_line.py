import math

import numpy as np
from scipy.optimize import minimize


def compute_normals(path):
    """
    Compute unit left normals for a closed path.

    normal[i] points toward the corridor's
    left side.
    """

    path = np.asarray(
        path,
        dtype=float,
    )

    n = len(path)

    normals = np.zeros(
        (n, 2),
        dtype=float,
    )

    for i in range(n):

        px, py = path[(i - 1) % n]
        nx, ny = path[(i + 1) % n]

        dx = nx - px
        dy = ny - py

        length = math.hypot(
            dx,
            dy,
        )

        if length < 1e-9:
            continue

        dx /= length
        dy /= length

        normals[i] = (
            -dy,
             dx,
        )

    return normals


def build_racing_path(
    centerline,
    normals,
    offsets,
):
    """
    Move each centreline point laterally
    according to its optimized offset.
    """

    centerline = np.asarray(
        centerline,
        dtype=float,
    )

    offsets = np.asarray(
        offsets,
        dtype=float,
    )

    return (
        centerline
        + normals * offsets[:, None]
    )

def curvature(path):
    path = np.asarray(path, dtype=float)

    p_prev = np.roll(path, 1, axis=0)
    p_next = np.roll(path, -1, axis=0)

    a = path - p_prev
    b = p_next - path
    c = p_next - p_prev

    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]

    denom = (
        np.linalg.norm(a, axis=1)
        * np.linalg.norm(b, axis=1)
        * np.linalg.norm(c, axis=1)
    )

    kappa = np.zeros_like(cross)

    valid = denom > 1e-9
    kappa[valid] = 2.0 * cross[valid] / denom[valid]

    return kappa

def optimize_racing_line(
    path,
    corridor,
    curvature_weight=1.0,
    smoothness_weight=0.05,
    offset_weight=0.001,
    boundary_margin=0.90,
):
    """
    Compute a map-constrained minimum-curvature
    racing line.

    The corridor provides:

        -right <= offset <= left
    """

    centerline = np.asarray(
        path,
        dtype=float,
    )

    normals = compute_normals(
        centerline,
    )

    left = np.asarray(
        corridor["left_clearance"],
        dtype=float,
    )

    right = np.asarray(
        corridor["right_clearance"],
        dtype=float,
    )

    n = len(centerline)

    # Don't use the absolute edge of the corridor.
    left *= boundary_margin
    right *= boundary_margin

    bounds = [
        (
            -right[i],
             left[i],
        )
        for i in range(n)
    ]

    def objective(offsets):

        racing_path = build_racing_path(
            centerline,
            normals,
            offsets,
        )

        kappa = curvature(
            racing_path,
        )

        # 1. Penalise curvature.
        curvature_cost = np.sum(
            kappa ** 2
        )

        # 2. Penalise changes in curvature /
        #    lateral offset.
        second_difference = (
            np.roll(offsets, -1)
            - 2.0 * offsets
            + np.roll(offsets, 1)
        )

        smoothness_cost = np.sum(
            second_difference ** 2
        )

        # 3. Very small regularisation term.
        offset_cost = np.sum(
            offsets ** 2
        )

        return (
            curvature_weight
            * curvature_cost
            +
            smoothness_weight
            * smoothness_cost
            +
            offset_weight
            * offset_cost
        )

    # Start from the centreline.
    initial = np.zeros(n)

    result = minimize(
        objective,
        initial,
        method="L-BFGS-B",
        bounds=bounds,
        options={
            "maxiter": 500,
            "ftol": 1e-9,
        },
    )

    if not result.success:
        print(
            "RACING: optimizer warning:",
            result.message,
        )

    racing_path = build_racing_path(
        centerline,
        normals,
        result.x,
    )

    return racing_path.tolist()
