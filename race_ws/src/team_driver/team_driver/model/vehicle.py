import numpy as np

from .parameters import VehicleParameters


class Vehicle:
    """
    F1TENTH single-track vehicle model.

    External control:
        steering_angle [rad]
        target_speed    [m/s]

    Internal control:
        steering_velocity [rad/s]
        acceleration      [m/s^2]

    State:
        [X, Y, DELTA, V, PSI, PSI_DOT, BETA]

    This matches the simulator's:
        - STEERING_ANGLE action
        - SPEED action
        - steering_constraint()
        - accl_constraints()
        - vehicle_dynamics_st()
        - RK4 integration
    """

    def __init__(
        self,
        params: VehicleParameters,
        dt: float = 0.01,
        steer_kp: float | None = None,
    ):
        self.params = params
        self.dt = dt

        if steer_kp is None:
            self.steer_kp = 10.0 * params.sv_max / (params.s_max - params.s_min)
        else:
            self.steer_kp = steer_kp

    # ============================================================
    # Steering actuator
    # ============================================================

    def pid_steer(
        self,
        target_steering: float,
        current_steering: float,
    ) -> float:
        """
        Convert target steering angle to steering velocity.

        Matches simulator's pid_steer().
        """

        steer_diff = target_steering - current_steering

        steering_velocity = self.steer_kp * steer_diff

        if steering_velocity > self.params.sv_max:
            steering_velocity = self.params.sv_max
        elif steering_velocity < -self.params.sv_max:
            steering_velocity = -self.params.sv_max

        return steering_velocity

    def steering_constraint(
        self,
        steering_angle: float,
        steering_velocity: float,
    ) -> float:
        """
        Apply steering angle and steering velocity limits.

        Matches simulator's steering_constraint().
        """

        p = self.params

        if (steering_angle <= p.s_min and steering_velocity <= 0.0) or (
            steering_angle >= p.s_max and steering_velocity >= 0.0
        ):
            return 0.0

        if steering_velocity <= p.sv_min:
            return p.sv_min

        if steering_velocity >= p.sv_max:
            return p.sv_max

        return steering_velocity

    def steering_action(
        self,
        target_steering: float,
        current_steering: float,
    ) -> float:
        """
        Complete STEERING_ANGLE action conversion.

        target steering angle
            -> P controller
            -> steering velocity
            -> steering constraints
        """

        steering_velocity = self.pid_steer(
            target_steering,
            current_steering,
        )

        return self.steering_constraint(
            current_steering,
            steering_velocity,
        )

    # ============================================================
    # Longitudinal actuator
    # ============================================================

    def pid_accl(
        self,
        target_speed: float,
        current_speed: float,
    ) -> float:
        """
        Convert target speed to desired acceleration.

        Matches simulator's pid_accl().
        """

        p = self.params

        vel_diff = target_speed - current_speed

        if current_speed > 0.0:
            if vel_diff > 0.0:
                kp = 10.0 * p.a_max / p.v_max
            else:
                kp = 10.0 * p.a_max / (-p.v_min)
        else:
            if vel_diff > 0.0:
                kp = 2.0 * p.a_max / p.v_max
            else:
                kp = 2.0 * p.a_max / (-p.v_min)

        return kp * vel_diff

    def upper_accel_limit(
        self,
        velocity: float,
    ) -> float:
        """
        Maximum positive acceleration at the current velocity.

        Matches simulator's upper_accel_limit().
        """

        p = self.params

        if velocity > p.v_switch:
            return p.a_max * (p.v_switch / velocity)

        return p.a_max

    def acceleration_constraint(
        self,
        velocity: float,
        acceleration: float,
    ) -> float:
        """
        Apply acceleration and velocity limits.

        Matches simulator's accl_constraints().
        """

        p = self.params

        positive_limit = self.upper_accel_limit(velocity)

        if velocity <= p.v_min and acceleration <= 0.0:
            return 0.0

        if velocity >= p.v_max and acceleration >= 0.0:
            return 0.0

        if acceleration <= -p.a_max:
            return -p.a_max

        if acceleration >= positive_limit:
            return positive_limit

        return acceleration

    def speed_action(
        self,
        target_speed: float,
        current_speed: float,
    ) -> float:
        """
        Complete SPEED action conversion.

        target speed
            -> P controller
            -> acceleration
            -> acceleration constraints
        """

        acceleration = self.pid_accl(
            target_speed,
            current_speed,
        )

        return self.acceleration_constraint(
            current_speed,
            acceleration,
        )

    # ============================================================
    # Single-track dynamics
    # ============================================================

    def dynamics(
        self,
        state: np.ndarray,
        target_steering: float,
        target_speed: float,
    ) -> np.ndarray:
        """
        F1TENTH single-track dynamics.

        State:
            [X, Y, DELTA, V, PSI, PSI_DOT, BETA]

        Returns:
            State derivative.
        """

        x = np.asarray(state, dtype=float)

        delta = x[2]
        velocity = x[3]
        psi = x[4]
        psi_dot = x[5]
        beta = x[6]

        p = self.params

        steering_velocity = self.steering_action(
            target_steering,
            delta,
        )

        acceleration = self.speed_action(
            target_speed,
            velocity,
        )

        g = 9.81
        wheelbase = p.lf + p.lr

        # --------------------------------------------------------
        # Low-speed kinematic region
        # --------------------------------------------------------

        if velocity < 0.5:
            beta_hat = np.arctan(np.tan(delta) * p.lr / wheelbase)

            beta_dot = (
                (1.0 / (1.0 + (np.tan(delta) * p.lr / wheelbase) ** 2))
                * (p.lr / (wheelbase * np.cos(delta) ** 2))
                * steering_velocity
            )

            return np.array(
                [
                    velocity * np.cos(psi + beta_hat),
                    velocity * np.sin(psi + beta_hat),
                    steering_velocity,
                    acceleration,
                    (velocity * np.cos(beta_hat) * np.tan(delta) / wheelbase),
                    (1.0 / wheelbase)
                    * (
                        acceleration * np.cos(beta) * np.tan(delta)
                        - velocity * np.sin(beta) * np.tan(delta) * beta_dot
                        + velocity
                        * np.cos(beta)
                        * steering_velocity
                        / np.cos(delta) ** 2
                    ),
                    beta_dot,
                ]
            )

        # --------------------------------------------------------
        # Dynamic single-track region
        # --------------------------------------------------------

        glr = g * p.lr - acceleration * p.h

        glf = g * p.lf + acceleration * p.h

        x_dot = velocity * np.cos(psi + beta)

        y_dot = velocity * np.sin(psi + beta)

        delta_dot = steering_velocity

        velocity_dot = acceleration

        yaw_rate_dot = (
            (p.mu * p.m)
            / (p.I * wheelbase)
            * (
                p.lf * p.C_Sf * glr * delta
                + (p.lr * p.C_Sr * glf - p.lf * p.C_Sf * glr) * beta
                - (p.lf**2 * p.C_Sf * glr + p.lr**2 * p.C_Sr * glf)
                * (psi_dot / velocity)
            )
        )

        beta_dot = (p.mu / (velocity * wheelbase)) * (
            p.C_Sf * glr * delta
            - (p.C_Sr * glf + p.C_Sf * glr) * beta
            + (p.C_Sr * glf * p.lr - p.C_Sf * glr * p.lf) * (psi_dot / velocity)
        ) - psi_dot

        return np.array(
            [
                x_dot,
                y_dot,
                delta_dot,
                velocity_dot,
                psi_dot,
                yaw_rate_dot,
                beta_dot,
            ]
        )

    # ============================================================
    # Integration
    # ============================================================

    def step(
        self,
        state: np.ndarray,
        target_steering: float,
        target_speed: float,
    ) -> np.ndarray:
        """
        Advance the vehicle by one timestep using RK4.

        Matches the simulator's rk4_integration().
        """

        x = np.asarray(
            state,
            dtype=float,
        )

        dt = self.dt

        k1 = self.dynamics(
            x,
            target_steering,
            target_speed,
        )

        k2_state = x + dt * (k1 / 2.0)

        k2 = self.dynamics(
            k2_state,
            target_steering,
            target_speed,
        )

        k3_state = x + dt * (k2 / 2.0)

        k3 = self.dynamics(
            k3_state,
            target_steering,
            target_speed,
        )

        k4_state = x + dt * k3

        k4 = self.dynamics(
            k4_state,
            target_steering,
            target_speed,
        )

        next_state = x + dt * (1.0 / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Match simulator heading wrapping.
        next_state[4] = (next_state[4] + np.pi) % (2.0 * np.pi) - np.pi

        return next_state

    # ============================================================
    # Vehicle measurements
    # ============================================================

    def yaw_rate(
        self,
        state: np.ndarray,
    ) -> float:
        """
        Current yaw rate.
        """

        return float(state[5])

    def slip_angle(
        self,
        state: np.ndarray,
    ) -> float:
        """
        Current body slip angle beta.
        """

        return float(state[6])

    def lateral_acceleration(
        self,
        state: np.ndarray,
    ) -> float:
        """
        Lateral acceleration approximation.

            a_y = v * psi_dot
        """

        return float(state[3] * state[5])

    def curvature(
        self,
        state: np.ndarray,
    ) -> float:
        """
        Vehicle path curvature.

            kappa = psi_dot / v

        At zero velocity, returns zero.
        """

        velocity = state[3]

        if abs(velocity) < 1e-6:
            return 0.0

        return float(state[5] / velocity)

    def actual_controls(
        self,
        state: np.ndarray,
        target_steering: float,
        target_speed: float,
    ) -> tuple[float, float]:
        """
        Return the actual internal controls produced
        by the simulator action conversion.

        Returns:
            steering_velocity [rad/s]
            acceleration      [m/s^2]
        """

        steering_velocity = self.steering_action(
            target_steering,
            state[2],
        )

        acceleration = self.speed_action(
            target_speed,
            state[3],
        )

        return (
            steering_velocity,
            acceleration,
        )
