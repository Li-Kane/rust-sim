# Spot ONNX Locomotion Policy Guide

This document summarizes the core specifications for porting the Spot locomotion ONNX policy (`spot_policy.onnx`) into `rust-sim`.

---

## 1. Provenance & Citations

The policy was developed and trained in **NVIDIA Isaac Lab** (formerly Isaac Orbit) in collaboration with **The Boston Dynamics AI Institute**:

- **Repository**: [isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab)
- **Task Config**: [`source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/spot/flat_env_cfg.py`](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/spot/flat_env_cfg.py)
- **Robot Asset Config**: [`source/isaaclab_assets/isaaclab_assets/robots/spot.py`](https://github.com/isaac-sim/IsaacLab/blob/main/source/isaaclab_assets/isaaclab_assets/robots/spot.py)
- **Tutorial & Deployment Guide**: [Isaac Lab Policy Deployment Tutorial](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/03_envs/policy_deployment.html)
- **Hardware Integration**: Boston Dynamics & AI Institute [Spot RL Researcher Kit](https://bostondynamics.com/reinforcement-learning-researcher-kit/)

---

## 2. Update Rate (50 Hz) and Action Scaling (0.2)

The **50 Hz update rate** and the **0.2 action scale** are **directly specified by Isaac Lab's official training configuration.

---

## 3. Joint Ordering

The simulation URDF and the ONNX policy use different joint index orderings.

### URDF Ordering (Leg-by-Leg)
Joints defined sequentially in `spot_simple.urdf`:
```
[0..2]   FL: fl_hx, fl_hy, fl_kn  (Front-Left: hip roll, hip pitch, knee)
[3..5]   FR: fr_hx, fr_hy, fr_kn  (Front-Right: hip roll, hip pitch, knee)
[6..8]   HL: hl_hx, hl_hy, hl_kn  (Hind-Left: hip roll, hip pitch, knee)
[9..11]  HR: hr_hx, hr_hy, hr_kn  (Hind-Right: hip roll, hip pitch, knee)
```

### Policy Ordering (Grouped by Joint Type)
The ONNX policy groups all 4 legs by joint function (`hx`, `hy`, `kn`):
```
[0..3]   Hip Roll (hx):   fl_hx, fr_hx, hl_hx, hr_hx
[4..7]   Hip Pitch (hy):  fl_hy, fr_hy, hl_hy, hr_hy
[8..11]  Knee Pitch (kn): fl_kn, fr_kn, hl_kn, hr_kn
```

### Remapping Indices
```rust
pub const URDF_TO_POLICY: [usize; 12] = [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11];
pub const POLICY_TO_URDF: [usize; 12] = [0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11];
```

---

## 4. Observation Vector (48 floats)

Specified directly by `SpotObservationsCfg.PolicyCfg` in `flat_env_cfg.py`. All spatial quantities are expressed in the **robot's local base body frame** ($B$).

| Indices | Size | Term | Description |
| :--- | :--- | :--- | :--- |
| `0..3` | 3 | `base_lin_vel` | Base linear velocity in robot body frame: $[v_x, v_y, v_z]$ ($v_x$=forward, $v_y$=left, $v_z$=up). |
| `3..6` | 3 | `base_ang_vel` | Base angular velocity in robot body frame: $[\omega_x, \omega_y, \omega_z]$ (roll, pitch, yaw rates). |
| `6..9` | 3 | `projected_gravity` | Unit gravity vector in robot body frame ($R_{BW} \hat{g}_W$). Upright on flat ground = `[0.0, 0.0, -1.0]`. |
| `9..12` | 3 | `velocity_commands` | Commanded velocities: $[v_{x,\text{cmd}}, v_{y,\text{cmd}}, \omega_{z,\text{cmd}}]$ (forward m/s, lateral m/s, yaw rad/s). |
| `12..24` | 12 | `joint_pos_rel` | Relative joint positions: $(q_{\text{current}} - q_{\text{default}})$ in **Policy order** (rad). |
| `24..36` | 12 | `joint_vel` | Joint velocities $\dot{q}$ in **Policy order** (rad/s). |
| `36..48` | 12 | `previous_action` | Raw unscaled output from the previous policy step (zeros at reset). |

---

## 5. Coordinate System Differences

### Robot Base Frame (URDF / Policy)
- Standard ROS robotics frame (**Z-up**):
  - +X: Forward
  - +Y: Left
  - +Z: Up

### Bevy World Frame
- OpenGL convention (**Y-up**):
  - +Y: Up
  - -Z: Forward
  - -X: Left
