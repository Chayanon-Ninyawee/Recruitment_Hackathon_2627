import math
from dataclasses import dataclass, replace

import numpy as np

PARAMETER_ORDER = (
    "mu",
    "C_Sf",
    "C_Sr",
    "lf",
    "lr",
    "h",
    "m",
    "I",
    "s_min",
    "s_max",
    "sv_min",
    "sv_max",
    "v_switch",
    "a_max",
    "v_min",
    "v_max",
    "width",
    "length",
    "collision_body_center_x",
    "collision_body_center_y",
    "kappa_dot_max",
    "kappa_dot_dot_max",
    "j_max",
    "j_dot_max",
    "m_s",
    "m_uf",
    "m_ur",
    "I_Phi_s",
    "I_y_s",
    "I_z",
    "I_xz_s",
    "K_sf",
    "K_sdf",
    "K_sr",
    "K_sdr",
    "T_f",
    "T_r",
    "K_ras",
    "K_tsf",
    "K_tsr",
    "K_rad",
    "K_zt",
    "h_raf",
    "h_rar",
    "h_s",
    "I_uf",
    "I_ur",
    "I_y_w",
    "K_lt",
    "R_w",
    "T_sb",
    "T_se",
    "D_f",
    "D_r",
    "E_f",
    "E_r",
    "tire_p_cx1",
    "tire_p_dx1",
    "tire_p_dx3",
    "tire_p_ex1",
    "tire_p_kx1",
    "tire_p_hx1",
    "tire_p_vx1",
    "tire_r_bx1",
    "tire_r_bx2",
    "tire_r_cx1",
    "tire_r_ex1",
    "tire_r_hx1",
    "tire_p_cy1",
    "tire_p_dy1",
    "tire_p_dy3",
    "tire_p_ey1",
    "tire_p_ky1",
    "tire_p_hy1",
    "tire_p_hy3",
    "tire_p_vy1",
    "tire_p_vy3",
    "tire_r_by1",
    "tire_r_by2",
    "tire_r_by3",
    "tire_r_cy1",
    "tire_r_ey1",
    "tire_r_hy1",
    "tire_r_vy1",
    "tire_r_vy3",
    "tire_r_vy4",
    "tire_r_vy5",
    "tire_r_vy6",
)


@dataclass(frozen=True)
class VehicleParameters:
    """Vehicle parameters shared by the vehicle and racing models."""

    # Single-track / kinematic model
    mu: float = math.nan
    C_Sf: float = math.nan
    C_Sr: float = math.nan
    lf: float = math.nan
    lr: float = math.nan
    h: float = math.nan
    m: float = math.nan
    I: float = math.nan

    # Steering limits
    s_min: float = math.nan
    s_max: float = math.nan
    sv_min: float = math.nan
    sv_max: float = math.nan

    # Longitudinal limits
    v_switch: float = math.nan
    a_max: float = math.nan
    v_min: float = math.nan
    v_max: float = math.nan

    # Geometry
    width: float = math.nan
    length: float = math.nan

    # Collision body
    collision_body_center_x: float = 0.0
    collision_body_center_y: float = 0.0

    # Multi-body parameters
    kappa_dot_max: float = math.nan
    kappa_dot_dot_max: float = math.nan
    j_max: float = math.nan
    j_dot_max: float = math.nan
    m_s: float = math.nan
    m_uf: float = math.nan
    m_ur: float = math.nan
    I_Phi_s: float = math.nan
    I_y_s: float = math.nan
    I_z: float = math.nan
    I_xz_s: float = math.nan
    K_sf: float = math.nan
    K_sdf: float = math.nan
    K_sr: float = math.nan
    K_sdr: float = math.nan
    T_f: float = math.nan
    T_r: float = math.nan
    K_ras: float = math.nan
    K_tsf: float = math.nan
    K_tsr: float = math.nan
    K_rad: float = math.nan
    K_zt: float = math.nan
    h_raf: float = math.nan
    h_rar: float = math.nan
    h_s: float = math.nan
    I_uf: float = math.nan
    I_ur: float = math.nan
    I_y_w: float = math.nan
    K_lt: float = math.nan
    R_w: float = math.nan
    T_sb: float = math.nan
    T_se: float = math.nan
    D_f: float = math.nan
    D_r: float = math.nan
    E_f: float = math.nan
    E_r: float = math.nan
    tire_p_cx1: float = math.nan
    tire_p_dx1: float = math.nan
    tire_p_dx3: float = math.nan
    tire_p_ex1: float = math.nan
    tire_p_kx1: float = math.nan
    tire_p_hx1: float = math.nan
    tire_p_vx1: float = math.nan
    tire_r_bx1: float = math.nan
    tire_r_bx2: float = math.nan
    tire_r_cx1: float = math.nan
    tire_r_ex1: float = math.nan
    tire_r_hx1: float = math.nan
    tire_p_cy1: float = math.nan
    tire_p_dy1: float = math.nan
    tire_p_dy3: float = math.nan
    tire_p_ey1: float = math.nan
    tire_p_ky1: float = math.nan
    tire_p_hy1: float = math.nan
    tire_p_hy3: float = math.nan
    tire_p_vy1: float = math.nan
    tire_p_vy3: float = math.nan
    tire_r_by1: float = math.nan
    tire_r_by2: float = math.nan
    tire_r_by3: float = math.nan
    tire_r_cy1: float = math.nan
    tire_r_ey1: float = math.nan
    tire_r_hy1: float = math.nan
    tire_r_vy1: float = math.nan
    tire_r_vy3: float = math.nan
    tire_r_vy4: float = math.nan
    tire_r_vy5: float = math.nan
    tire_r_vy6: float = math.nan

    @property
    def wheelbase(self) -> float:
        return self.lf + self.lr

    def with_updates(self, **updates: float) -> "VehicleParameters":
        return replace(self, **updates)

    def missing_mb_parameters(self) -> tuple[str, ...]:
        return tuple(
            name for name in PARAMETER_ORDER if not math.isfinite(getattr(self, name))
        )

    def to_array(self) -> np.ndarray:
        return np.asarray(
            [getattr(self, name) for name in PARAMETER_ORDER],
            dtype=np.float32,
        )


F1TENTH_VEHICLE_PARAMETERS = VehicleParameters(
    mu=1.0489,
    C_Sf=4.718,
    C_Sr=5.4562,
    lf=0.15875,
    lr=0.17145,
    h=0.074,
    m=3.74,
    I=0.04712,
    s_min=-0.4189,
    s_max=0.4189,
    sv_min=-3.2,
    sv_max=3.2,
    v_switch=7.319,
    a_max=9.51,
    v_min=-5.0,
    v_max=20.0,
    width=0.31,
    length=0.58,
    collision_body_center_x=0.1651,
)
