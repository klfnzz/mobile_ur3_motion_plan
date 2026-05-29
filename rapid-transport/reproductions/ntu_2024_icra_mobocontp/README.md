# NTU 2024 ICRA MoboConTP Reproduction

This folder is a self-contained reproduction scaffold for:

`Planning Optimal Trajectories for Mobile Manipulators under End-effector Trajectory Continuity Constraint`

It reuses this project's OpenRAVE UR3 + Ranger-style mobile base model and
implements the paper's core planning idea for a mobile printing/spraying task:

1. sample a fixed, time-parametrized end-effector trajectory;
2. build admissible base configuration spacetime stages;
3. apply MoboConTP-style backward dynamic programming with base velocity
   controls and a minimum-control-effort cost;
4. optionally recover UR3 arm IK samples along the optimal base trajectory.

## Files

- `models/ur3_ranger_printing.env.xml`: OpenRAVE scene with UR3+base,
  printing surface, and a base obstacle.
- `models/open_workspace.env.xml`: obstacle-free OpenRAVE scene for the joint
  visual simulation.
- `config/printing_line.yaml`: default continuous printing task and planner
  discretization.
- `config/arm_motion_demo.yaml`: compact visual simulation where both the
  mobile base and UR3 arm move.
- `run_mobocontp_ur3.py`: runnable reproduction script.
- `outputs/`: generated CSV/SVG/JSON results from the latest run.

## Run

From the repository root:

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml
```

The default config sets `ik.mode: off`, so the script reproduces the base
spacetime planner without requiring an IKFast cache. It exports:

- `outputs/printing_line_trajectory.csv`
- `outputs/printing_line_dense_base_ee.csv`
- `outputs/printing_line_top_view.svg`
- `outputs/printing_line_summary.json`

To ask OpenRAVE to solve UR3 IK along the optimal base trajectory:

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml \
  --ik generate
```

`--ik generate` may take several minutes because OpenRAVE generates IKFast
for the active UR3 manipulator if no cache exists.

## Verification Experiment

The verification scene uses a U-shaped continuous end-effector path and two
base obstacles to check that the admissible B-spacetime and backward dynamic
programming steps are doing real work:

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/exp_verify.yaml
```

This writes:

- `outputs/exp_verify_trajectory.csv`
- `outputs/exp_verify_dense_base_ee.csv`
- `outputs/exp_verify_top_view.svg`
- `outputs/exp_verify_summary.json`

## Larger Visual Scene

For a larger and more target-like mobile printing/spraying task, use the open
workspace scene with no printing table, several wall/column obstacles, and a
long serpentine continuous end-effector trajectory:

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/large_printing_world.yaml
```

This writes:

- `outputs/large_printing_world_trajectory.csv`
- `outputs/large_printing_world_dense_base_ee.csv`
- `outputs/large_printing_world_top_view.svg`
- `outputs/large_printing_world_summary.json`

## Joint Visual Simulation

Use this command for the obstacle-free visual simulation. The compact demo now
follows the paper's geometry-first pipeline more closely: it precomputes a
valid voxel cloud around the arm base, maps each task-space target point back
into that cloud for every candidate base state, builds admissible B-spacetime,
then runs backward DP and recovers a smooth UR3 joint sequence:

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/arm_motion_demo.yaml \
  --ik load \
  --view
```

Expected key lines:

```text
Loaded valid voxel cloud: 5585 / 14157 valid voxels
Solved IK for 13/13 trajectory samples.
Optimal base trajectory cost: 0.032500
```

On the first run, the cache may instead print `Building valid voxel cloud`.
Later runs reuse `outputs/arm_motion_valid_voxel_cloud.npz`. The current task
asks the base to move from around `(0.35, 0.00)` to around `(0.85, -0.25)`
while the end effector follows a continuous line from
`(0.30, -0.20, 0.70)` to `(1.10, -0.20, 0.70)`. It writes:

- `outputs/arm_motion_demo_trajectory.csv`
- `outputs/arm_motion_demo_dense_base_ee.csv`
- `outputs/arm_motion_demo_top_view.svg`
- `outputs/arm_motion_demo_summary.json`

The OpenRAVE warnings about `OPENRAVE_PLUGINS` and transparent Collada
materials are non-fatal in this environment.

To animate the default planner-only scene instead:

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/printing_line.yaml \
  --view
```

## Paper U-Shape Task

The paper's hardware demo reports a mobile 3D-printing U shape of
`0.9 x 0.675 x 0.05 m`, `5` layers, total printing path `19.85 m`, and constant
nozzle speed `10 cm/s`. The exact CAD waypoints are not listed in the PDF, so
`paper_u_shape.yaml` generates a matching U-shaped multilayer contour with
length `19.844 m` and `dt = 3 s`, as reported in the evaluation section.

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/paper_u_shape.yaml \
  --ik load \
  --view
```

Verified key lines:

```text
Loaded task samples: 67
Built valid voxel cloud: 5587 / 15925 valid voxels
Solved IK for 67/67 trajectory samples.
Optimal base trajectory cost: 0.037500
```

## Paper NTU Task

The paper's Fig. 1 reports an NTU shape with `10` layers, size
`3.0 x 0.75 x 0.15 m`, total printing path `112.9 m`, and constant nozzle
speed `10 cm/s`. The exact CAD waypoints are not included in the PDF, so
`paper_ntu_shape.yaml` reconstructs a paper-scale grouped-letter NTU stroke
path and matches the reported total length exactly.

The suction direction is treated as the manipulator TCP local `Z` axis. The
task target pose sets TCP `Z` along world `-Z`; the final arm trajectory is
solved as a 5D IK task, matching position plus TCP `Z` direction while leaving
rotation about the suction axis free, as in the paper's printing task.

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/paper_ntu_shape.yaml \
  --ik load \
  --view \
  --view-delay 0.05 \
  --view-substeps 8
```

Verified key lines:

```text
Loaded task samples: 95
Loaded valid voxel cloud: 573 / 2527 valid voxels
Solved TCP z-axis numeric IK for 95/95 trajectory samples.
Optimal base trajectory cost: 2.490000
```

The generated trajectory has also been checked in OpenRAVE: maximum TCP
`Z`-down angular error is about `0.087 deg`, and maximum TCP position error
is about `5.1e-4 m`.

## Single-Layer NTU Table Scene

For a clearer visualization of one NTU pass on a plane, use
`paper_ntu_table_single.yaml`. It uses the same `3.0 x 0.75 m` NTU footprint,
adds a flat printing plane, and prints only one layer. The target path follows
an equal-width CAD-style NTU outer contour as one continuous closed loop:
straight line segments form the N and T contours, the T top bar connects into
both N and U, a smooth circular arc forms the bottom of the U, and the path
returns to the start without a tool lift. With the current equal-spacing
outline sampler the loop length is `10.4559 m`. The task uses
`duration = 115.0 s` and `dt = 2.5 s`, giving `47` spacetime samples.

```bash
python reproductions/ntu_2024_icra_mobocontp/run_mobocontp_ur3.py \
  --config reproductions/ntu_2024_icra_mobocontp/config/paper_ntu_table_single.yaml \
  --view \
  --view-delay 0.06 \
  --view-substeps 8
```

Verified key lines:

```text
Loaded task samples: 47
OpenRAVE whole-robot collision checks: 82796, rejected 20028
Solved TCP z-axis numeric IK for 29/47 trajectory samples.
Keeping previous arm posture for failed samples: [...]
Optimal base trajectory cost: 254.336568
Vehicle +X/work heading error: max 15.03 deg, mean 4.72 deg
Whole-robot OpenRAVE collisions: env [], self []
```

The base is constrained outside the table footprint and guided around the
table-side band. Current output checks:

```text
base yaw is the zhijia/arm-support side front direction
OpenRAVE model yaw = base_yaw - 180 deg
continuous CAD NTU outer contour: 521 waypoints, 10.4559 m, closed distance 0
duration/dt/samples: 115.0 s / 2.5 s / 47
vehicle +X/work heading error max/mean: 15.03 / 4.72 deg
base yaw rotation: 13 nonzero steps, 240 deg total
whole-robot OpenRAVE collision samples: env [], self []
base_inside_table_forbidden: []
end-effector footprint: 3.0 x 0.75 m at z=0.56 m
```

This scene is meant as a visual reproduction. It records partial numeric IK
failures in `outputs/paper_ntu_table_single_summary.json`; failed arm samples
hold the previous solved posture so the OpenRAVE animation can still show the
base, table, and NTU task trajectory continuously.

## Mapping To The Paper

- Paper base state `qb = (x, y, phi)` is represented by each grid node in
  `base_grid`.
- Paper spacetime stages `Xi_a` are represented by the per-time-step
  admissible dictionaries printed as `stage 00`, `stage 01`, etc.
- Paper admissible controls `Ua` are generated from `controls.dvx`,
  `controls.dvy`, `controls.domega_deg`, `controls.vmax`, and
  `controls.omega_max_deg`.
- Paper backward value iteration is implemented by `mobocontp_backward_dp`.
- Paper cost `integral qdot^T I qdot dt` is implemented by `control_cost`
  with `cost.yaw_weight`.

The simple starter demos can still use a fixed 6D TCP pose, but the paper-style
U-shape/NTU configs use the geometry-first path: valid voxel cloud for
admissible base stages, then 5D numeric IK for the final arm trajectory. The
central continuity constraint is preserved: every base stage corresponds to a
fixed timestamp on the continuous end-effector trajectory.

## Current Notes

For configs without `valid_voxel_cloud.enabled`, the geometric reachable region
falls back to lightweight distance/heading filters in `reachability`. The
paper-style configs should be preferred when comparing against the algorithm in
the PDF.
