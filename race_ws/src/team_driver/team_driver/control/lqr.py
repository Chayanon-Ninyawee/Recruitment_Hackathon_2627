import math

import numpy as np


class LQRController:

    def __init__(
        self,
        wheelbase,
        lateral_weight=1.0,
        heading_weight=1.0,
        steering_weight=0.1,
    ):
        self.wheelbase = wheelbase

        self.Q = np.diag(
            [
                lateral_weight,
                heading_weight,
            ]
        )

        self.R = np.array(
            [
                [steering_weight],
            ]
        )

        self.K = None
        self.last_speed = None

    def update(self, speed):
        speed = max(abs(speed), 0.01)

        dt = 0.05

        A = np.array(
            [
                [1.0, speed * dt],
                [0.0, 1.0],
            ]
        )

        B = np.array(
            [
                [0.0],
                [speed * dt / self.wheelbase],
            ]
        )

        P = self.Q.copy()

        for _ in range(100):
            P_next = (
                A.T @ P @ A
                - A.T @ P @ B @ np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A
                + self.Q
            )

            if np.max(np.abs(P_next - P)) < 1e-9:
                P = P_next
                break

            P = P_next

        self.K = np.linalg.inv(self.R + B.T @ P @ B) @ B.T @ P @ A

        self.last_speed = speed

    def control(self, lateral_error, heading_error, speed, curvature):
        speed = max(abs(speed), 0.01)

        if (
            self.K is None
            or self.last_speed is None
            or abs(speed - self.last_speed) > 0.05
        ):
            self.update(speed)

        state = np.array(
            [
                [lateral_error],
                [heading_error],
            ]
        )

        feedback = float(-(self.K @ state)[0, 0])

        feedforward = 0.01 * math.atan(self.wheelbase * curvature)

        steering = feedback + feedforward

        return steering
