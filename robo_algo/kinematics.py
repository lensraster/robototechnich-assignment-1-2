"""
Planar serial manipulator: forward kinematics, Jacobian, and iterative IK (damped least squares / LM).
"""
from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional

import numpy as np
import numpy.typing as npt

if TYPE_CHECKING:
    from robo_algo.arm import RoboticArm


def forward_kinematics(
    arm: Any, angles: Optional[npt.NDArray[np.floating]] = None
) -> npt.NDArray[np.float64]:
    """End-effector position (x, y) in world coordinates."""
    base = np.asarray(arm.p0, dtype=np.float64).reshape(2)
    lengths = np.asarray(arm.link_lengths, dtype=np.float64)
    if angles is None:
        angles = np.asarray(arm.get_angles(), dtype=np.float64)
    else:
        angles = np.asarray(angles, dtype=np.float64).reshape(-1)
    return fk_from_params(base, lengths, angles)


def fk_from_params(
    base: npt.NDArray[np.floating],
    link_lengths: npt.NDArray[np.floating],
    link_angles: npt.NDArray[np.floating],
) -> npt.NDArray[np.float64]:
    base = np.asarray(base, dtype=np.float64).reshape(2)
    L = np.asarray(link_lengths, dtype=np.float64).reshape(-1)
    q = np.asarray(link_angles, dtype=np.float64).reshape(-1)
    cum = np.cumsum(q)
    disp = np.stack((L * np.cos(cum), L * np.sin(cum)), axis=1)
    return base + np.sum(disp, axis=0)


def jacobian(link_lengths: npt.NDArray[np.floating], link_angles: npt.NDArray[np.floating]) -> npt.NDArray[np.float64]:
    """
    Jacobian J in R^{2 x n}: rows are (dx/dq, dy/dq) for planar revolute chain with absolute cumulative angles.
    """
    L = np.asarray(link_lengths, dtype=np.float64).reshape(-1)
    q = np.asarray(link_angles, dtype=np.float64).reshape(-1)
    n = q.size
    cum = np.cumsum(q)
    J = np.zeros((2, n), dtype=np.float64)
    for k in range(n):
        for i in range(k, n):
            J[0, k] += -L[i] * np.sin(cum[i])
            J[1, k] += L[i] * np.cos(cum[i])
    return J


def _lm_delta(
    J: npt.NDArray[np.float64],
    e: npt.NDArray[np.float64],
    lm_lambda: float,
) -> npt.NDArray[np.float64]:
    """Damped least squares: (J J^T + λ² I) v = e,  Δq = J^T v."""
    jjt = J @ J.T
    a = jjt + (lm_lambda**2) * np.eye(2, dtype=np.float64)
    v = np.linalg.solve(a, e)
    return J.T @ v


def inverse_kinematics(
    target_position: npt.ArrayLike,
    arm: Any,
    *,
    max_iter: int = 150,
    tol: float = 2e-3,
    lm_lambda: float = 0.12,
    step_scale: float = 1.0,
    use_nullspace: bool = True,
    nullspace_gain: float = 0.08,
    angle_limit: Optional[float] = None,
) -> npt.NDArray[np.float64]:
    """
    Iterative IK using Levenberg–Marquardt (damped least squares on the Jacobian transpose).
    Optional nullspace term (I - J⁺J) pushes joints toward 0 to stay near [-π, π] behavior.
    """
    base = np.asarray(arm.p0, dtype=np.float64).reshape(2)
    L = np.asarray(arm.link_lengths, dtype=np.float64)
    q = np.asarray(arm.get_angles(), dtype=np.float64).copy()
    target = np.asarray(target_position, dtype=np.float64).reshape(2)

    for _ in range(max_iter):
        pos = fk_from_params(base, L, q)
        e = target - pos
        if np.linalg.norm(e) < tol:
            break
        J = jacobian(L, q)
        dq = _lm_delta(J, e, lm_lambda) * step_scale

        if use_nullspace and q.size > 2:
            jjt = J @ J.T
            a = jjt + (lm_lambda**2) * np.eye(2, dtype=np.float64)
            J_pinv_left = J.T @ np.linalg.inv(a)
            p = np.eye(q.size) - J_pinv_left @ J
            q_pref = np.zeros_like(q)
            dq = dq + nullspace_gain * (p @ (q_pref - q))

        q = q + dq
        if angle_limit is not None:
            q = np.clip(q, -float(angle_limit), float(angle_limit))

    return q
