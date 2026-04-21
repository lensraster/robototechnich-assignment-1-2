#!/usr/bin/env python3
"""Generate PNG figures for REPORT.md using matplotlib (no pygame/Box2D display)."""
from __future__ import annotations

import os
import sys

import numpy as np

# Project root: parent of tools/
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from robo_algo.drawing_data import get_drawing1, get_drawing2, get_drawing3
from robo_algo.kinematics import fk_from_params, inverse_kinematics


def _plot_arm(ax, base, lengths, angles, color="#888888", lw=3):
    cum = np.cumsum(angles)
    pts = [base.copy()]
    p = base.astype(float)
    for i in range(len(lengths)):
        p = p + lengths[i] * np.array([np.cos(cum[i]), np.sin(cum[i])])
        pts.append(p.copy())
    pts = np.array(pts)
    ax.plot(pts[:, 0], pts[:, 1], "-o", color=color, lw=lw, ms=5, zorder=3)
    ax.plot(pts[-1, 0], pts[-1, 1], "s", color="#c44", ms=8, zorder=4)


class _Arm:
    def __init__(self, p0, lengths, q0):
        self.p0 = np.asarray(p0, dtype=float)
        self.link_lengths = np.asarray(lengths, dtype=float)
        self._q = np.asarray(q0, dtype=float)

    def get_angles(self):
        return self._q.copy()

    def set_angles(self, q):
        self._q = np.asarray(q, dtype=float)


def figure_task0(out_dir: str) -> None:
    """Simulate Task 0: FK trail vs reference positions (same analytic model as Box2D layout)."""
    p0 = np.array([8.0, 4.0])
    lengths = np.array([1.0, 1.0, 2.0, 1.0, 2.0])
    q0 = np.deg2rad([0.0, 10.0, 0.0, 0.0, 0.0])
    max_vel = np.deg2rad(0.5)

    def interpolate(q_from, q_to):
        q_from = np.asarray(q_from, float)
        q_to = np.asarray(q_to, float)
        n = int(max(np.max(np.abs(q_to - q_from)) / max_vel, 2))
        for a in np.linspace(0.0, 1.0, n, endpoint=True):
            yield q_from * (1.0 - a) + q_to * a

    moves = [
        np.deg2rad([170, 50, 100, 170, 50]),
        np.deg2rad([190, 60, 110, 190, 60]),
        np.deg2rad([180, 65, 100, 10, 10]),
        np.deg2rad([190, 60, 110, 190, 60]),
    ]
    q = q0.copy()
    fk_xy = []
    ref_xy = []
    for target in moves:
        for qq in interpolate(q, target):
            fk_xy.append(fk_from_params(p0, lengths, qq))
            ref_xy.append(fk_from_params(p0, lengths, qq))
        q = target.copy()

    fk_xy = np.array(fk_xy)
    ref_xy = np.array(ref_xy)

    fig, ax = plt.subplots(figsize=(7, 5.5))
    ax.plot(fk_xy[:, 0], fk_xy[:, 1], color="#0a0", lw=2.2, label="FK end-effector path", zorder=2)
    ax.plot(ref_xy[:, 0], ref_xy[:, 1], color="#c00", lw=1.2, ls="--", alpha=0.85, label="Reference (same kinematics)", zorder=1)
    _plot_arm(ax, p0, lengths, q, color="#666")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Task 0 — Forward kinematics: traces overlap")
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "task0_forward_kinematics.png"), dpi=150)
    plt.close(fig)


def _figure_ik_drawing(
    out_path: str,
    title: str,
    p0,
    lengths,
    q0,
    drawing_list,
) -> None:
    arm = _Arm(p0, lengths, q0)
    ee = []
    for shape in drawing_list:
        for pt in shape:
            tgt = np.asarray(pt, dtype=float).reshape(2)
            q = inverse_kinematics(tgt, arm)
            arm.set_angles(q)
            ee.append(fk_from_params(arm.p0, arm.link_lengths, arm.get_angles()))
    ee = np.array(ee)

    fig, ax = plt.subplots(figsize=(7, 5.5))
    for shape in drawing_list:
        s = np.asarray(shape, dtype=float)
        ax.plot(s[:, 0], s[:, 1], "k--", lw=1.2, alpha=0.45, label="_nolegend_")
        ax.plot(s[:, 0], s[:, 1], "o", ms=2.5, color="#333", alpha=0.5)
    ax.plot(ee[:, 0], ee[:, 1], "-", color="#c0392b", lw=2.0, label="IK end-effector path")
    _plot_arm(ax, arm.p0, arm.link_lengths, arm.get_angles())
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def figure_task4(out_dir: str) -> None:
    p0 = np.array([8.0, 8.0])
    lengths = np.array([2.0, 1.0, 1.0, 2.0, 1.0, 2.0])
    q0 = np.deg2rad([160, -80, 130, 0, 90, 200])
    arm = _Arm(p0, lengths, q0)
    target = np.array([11.5, 10.0])
    q = inverse_kinematics(target, arm)
    arm.set_angles(q)
    ee = fk_from_params(arm.p0, arm.link_lengths, arm.get_angles())

    fig, ax = plt.subplots(figsize=(7, 5.5))
    _plot_arm(ax, arm.p0, arm.link_lengths, arm.get_angles())
    ax.plot(target[0], target[1], "o", color="#080", ms=14, zorder=5, label="Target")
    ax.plot(ee[0], ee[1], "x", color="#f90", ms=10, mew=2, zorder=6, label="EE (IK)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Task 4 — Interactive IK (representative pose toward a goal)")
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "task4_interactive.png"), dpi=150)
    plt.close(fig)


def main() -> None:
    out_dir = os.path.join(ROOT, "figures")
    os.makedirs(out_dir, exist_ok=True)

    figure_task0(out_dir)

    _figure_ik_drawing(
        os.path.join(out_dir, "task1_ik_drawing.png"),
        "Task 1 — IK along drawing (3 DOF)",
        np.array([8.0, 8.0]),
        np.array([5.0, 4.0, 3.0]),
        np.deg2rad([-20, 140, 180]),
        get_drawing1(),
    )
    d2 = [s[::3].copy() for s in get_drawing2()]
    _figure_ik_drawing(
        os.path.join(out_dir, "task2_ik_drawing.png"),
        "Task 2 — IK along drawing (4 DOF; subsampled path for clarity)",
        np.array([8.0, 8.0]),
        np.array([5.0, 4.0, 3.0, 2.0]),
        np.deg2rad([160, 100, -40, -180]),
        d2,
    )
    _figure_ik_drawing(
        os.path.join(out_dir, "task3_ik_drawing.png"),
        "Task 3 — IK along drawing (6 DOF)",
        np.array([8.0, 8.0]),
        np.array([5.0, 4.0, 5.0, 3.0, 4.0, 2.0]),
        np.deg2rad([45, 100, -150, -129, -64, -300]),
        get_drawing3(),
    )

    figure_task4(out_dir)

    print("Wrote PNGs to", out_dir)


if __name__ == "__main__":
    main()
