import math


class SpeedController:

    def __init__(
        self,
        max_speed=1.0,
        min_speed=0.2,
        max_lateral_acceleration=2.0,
    ):
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_lateral_acceleration = max_lateral_acceleration

    def from_curvature(self, curvature):
        curvature = abs(curvature)

        if curvature < 1e-6:
            return self.max_speed

        speed = math.sqrt(self.max_lateral_acceleration / curvature)

        return max(
            self.min_speed,
            min(self.max_speed, speed),
        )
