import math

from .lqr import LQRController
from .speed import SpeedController


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


class LQRTracker:

    def __init__(
        self,
        wheelbase,
        max_steering,
        max_speed=1.0,
        min_speed=0.2,
        lateral_weight=1.0,
        heading_weight=1.0,
        steering_weight=0.1,
        max_lateral_acceleration=2.0,
        lookahead_distance=0.5,
    ):
        self.wheelbase = wheelbase
        self.max_steering = max_steering
        self.lookahead_distance = lookahead_distance

        self.lqr = LQRController(
            wheelbase=wheelbase,
            lateral_weight=lateral_weight,
            heading_weight=heading_weight,
            steering_weight=steering_weight,
        )

        self.speed = SpeedController(
            max_speed=max_speed,
            min_speed=min_speed,
            max_lateral_acceleration=max_lateral_acceleration,
        )

    def control(self, state, path, corridor):
        if state.position is None:
            return 0.0, 0.0

        if path is None or len(path) < 2:
            return 0.0, 0.0

        x, y = state.position

        closest_index = self.find_nearest_index(
            x,
            y,
            path,
        )

        if closest_index is None:
            return 0.0, 0.0

        reference_index = self.find_lookahead_index(
            x,
            y,
            path,
            closest_index,
        )

        reference = self.get_reference(
            path,
            reference_index,
        )

        if reference is None:
            return 0.0, 0.0

        lateral_error = self.lateral_error(
            x,
            y,
            reference,
        )

        heading_error = self.heading_error(
            state.yaw,
            reference,
        )

        steering = self.lqr.control(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed=state.speed,
        )

        steering = max(
            -self.max_steering,
            min(self.max_steering, steering),
        )

        target_speed = self.speed.from_curvature(
            reference["curvature"],
        )

        return steering, target_speed

    def find_nearest_index(self, x, y, path):
        best_index = None
        best_distance = float("inf")

        for i, point in enumerate(path):
            px = point[0]
            py = point[1]

            distance = (px - x) ** 2 + (py - y) ** 2

            if distance < best_distance:
                best_distance = distance
                best_index = i

        return best_index

    def find_lookahead_index(self, x, y, path, closest_index):
        distance = 0.0

        for i in range(closest_index, len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            distance += math.hypot(
                x2 - x1,
                y2 - y1,
            )

            if distance >= self.lookahead_distance:
                return i + 1

        return len(path) - 1

    def get_reference(self, path, index):
        if index < 0 or index >= len(path):
            return None

        x, y = path[index]

        if index == 0:
            x0, y0 = path[0]
            x1, y1 = path[1]
        elif index == len(path) - 1:
            x0, y0 = path[index - 1]
            x1, y1 = path[index]
        else:
            x0, y0 = path[index - 1]
            x1, y1 = path[index + 1]

        yaw = math.atan2(
            y1 - y0,
            x1 - x0,
        )

        curvature = self.calculate_curvature(
            path,
            index,
        )

        return {
            "x": x,
            "y": y,
            "yaw": yaw,
            "curvature": curvature,
        }

    def calculate_curvature(self, path, index):
        if index <= 0 or index >= len(path) - 1:
            return 0.0

        x1, y1 = path[index - 1]
        x2, y2 = path[index]
        x3, y3 = path[index + 1]

        a = math.hypot(
            x2 - x1,
            y2 - y1,
        )

        b = math.hypot(
            x3 - x2,
            y3 - y2,
        )

        c = math.hypot(
            x3 - x1,
            y3 - y1,
        )

        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            return 0.0

        area_twice = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)

        return 2.0 * area_twice / (a * b * c)

    def lateral_error(self, x, y, reference):
        dx = x - reference["x"]
        dy = y - reference["y"]

        return -math.sin(reference["yaw"]) * dx + math.cos(reference["yaw"]) * dy

    def heading_error(self, yaw, reference):
        return normalize_angle(yaw - reference["yaw"])
