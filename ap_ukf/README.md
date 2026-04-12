# ap_ukf

Unscented Kalman Filter fusion for **horizontal** state in **local meters** .

## Behavior

- **State (4D):** `x`, `y`, `vx`, `vy` : local frame, meters and m/s. No IMU, attitude, or Z in the filter.
- **Initialization:** First **`/odometry/vps`** message triggers a **bootstrap** with position from VPS and velocity from the latest VIO twist (or zero if none yet). No IMU required.
- **VIO** (`/odometry/vio`): full 4D observation (pose xy + twist xy). Queue time-ordered.
- **VPS** (`/odometry/vps`): absolute **xy**; optional **Mahalanobis gate** (`vps_chi2_threshold`). **Soft correction:** each fix is applied as **N** intermediate measurements along a line from the current estimate to the target (`vps_soft_frames`).
- **Output:** `/topic/sensor/odom_state` : horizontal pose + twist; `z` and orientation are placeholders (identity quaternion); fuse heading/altitude outside this node per plan.

## Parameters (`params/estimator_config.yaml`)

- `frequency` : timer rate for publishing and draining the measurement queue.
- `vps_soft_frames` : N for soft VPS (default 10).
- `vps_chi2_threshold` : chi^2 on innovation; set **0** to disable.
- `process_noise_covariance` / `initial_estimate_covariance` : **4×4** matrices.
- `base_link_frame`, `odom_frame`

## Reference

Original 15D fork: https://github.com/rummanwaqar