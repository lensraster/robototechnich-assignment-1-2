# Assignment report: forward and inverse kinematics

**Source code:** [github.com/lensraster/robototechnich-assignment-1-2](https://github.com/lensraster/robototechnich-assignment-1-2) (public repository)

This report summarizes the implementation required by [README_EN.md](README_EN.md): forward kinematics (Task 0), Jacobian-based inverse kinematics for planar arms (Tasks 1–3), and an interactive IK demo (Task 4).

## Environment

Follow the README:

```bash
python3 -m venv venv
source ./venv/bin/activate
pip install -r requirements.txt
pip install -e .
```

On some systems `box2d-py` needs build tools (for example SWIG) or a prebuilt wheel; install any missing system packages if the install step fails.

## Task 0 — Forward kinematics

**Goal:** Compute the end-effector position from the base `p0`, link lengths `L_i`, and relative joint angles `q_i`, so that the green trace (FK) matches the red trace (simulation).

**Model:** Cumulative angles `φ_i = Σ_{j=0}^{i} q_j`. Each link contributes a displacement `(L_i cos φ_i, L_i sin φ_i)`. The end effector is:

`p_ee = p_0 + Σ_i (L_i cos φ_i, L_i sin φ_i)`.

**Code:** `forward_kinematics` in `robo_algo/kinematics.py` (used from `task0.py`).

**Check:** With `TEST_RUN = True` in `task0.py`, the script compares FK to the Box2D end-effector position (`atol=0.1`).

## Tasks 1–3 — Jacobian and inverse kinematics

**Jacobian:** For a planar revolute chain, with `φ_i` as above,

- `∂x/∂q_k = Σ_{i≥k} (-L_i sin φ_i)`
- `∂y/∂q_k = Σ_{i≥k} (L_i cos φ_i)`

This yields `J ∈ ℝ^{2×n}` (rows: x and y partials).

**IK method:** Levenberg–Marquardt / damped least squares. At each iteration, with error `e = p_goal - p_ee(q)`:

`(J J^T + λ^2 I) v = e`, `Δq = J^T v`

with damping `λ` (implementation uses `lm_lambda` in `inverse_kinematics`). The step is scaled by `step_scale` (default 1). Iteration stops when `‖e‖ < tol` or `max_iter` is reached.

**Nullspace (redundant arms, n > 2):** An extra term `(I - J^+ J) (q_pref - q)` with `q_pref = 0` biases the solution toward a neutral posture and reduces drift when many joint configurations solve the same end-effector pose. This is optional (`use_nullspace=True` by default) and only applied when there are more than two joints.

**Drawing logic:** For each list of polylines (`get_drawing1()`, `get_drawing2()`, `get_drawing3()`), the controller visits waypoints in order. While `ArmController` is idle, the next target is taken, `inverse_kinematics` computes joint angles, and `move_to_angles` interpolates motion subject to `MAX_SPEED`. Each frame: `controller.step()`, then `arm.draw()`.

| Task | Links / DOF | Data |
|------|-------------|------|
| 1 | 3 | `get_drawing1()` — one closed polyline |
| 2 | 4 | `get_drawing2()` — two sine/cosine paths |
| 3 | 6 | `get_drawing3()` — multiple separate shapes |

## Task 4 — Interactive IK

**Behavior:** The mouse sets a target point in world coordinates (green circle). Whenever the controller is idle, joint targets are recomputed with `inverse_kinematics` toward the current mouse position, then `move_to_angles` runs; `step()` and `arm.draw()` run every frame.

**What to try (as in the README):** Move the target slowly vs quickly, choose points clearly outside the reachable workspace, and compare how the arm behaves when the goal jumps (tracking lag vs smooth motion). Adjust `MAX_SPEED` in `task4.py` to see how interpolation limits affect following.

## File map

| File | Role |
|------|------|
| `robo_algo/kinematics.py` | FK, Jacobian, iterative IK (LM + optional nullspace) |
| `task0.py` | Task 0 visualization |
| `task1.py` … `task3.py` | IK drawing demos |
| `task4.py` | Interactive demo |

## How to run

```bash
source ./venv/bin/activate
python task0.py
python task1.py
python task2.py
python task3.py
python task4.py
```

Close any window with Escape or the window close button.

## Summary

Forward kinematics follows the same cumulative-angle convention as `RoboticArm` in `robo_algo/arm.py`. Inverse kinematics uses a damped Jacobian transpose (Levenberg–Marquardt) update with an optional nullspace projection for redundant manipulators, integrated with the provided `ArmController` for smooth motion along polylines and toward a moving target.
