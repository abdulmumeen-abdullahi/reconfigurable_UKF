# Reconfigurable N-DOF UKF State Estimation Architecture

[![ROS 2](https://img.shields.io/badge/ROS_2-Compatible-22314E?logo=ros)](https://docs.ros.org/)
[![Python 3.12+](https://img.shields.io/badge/Python-3.12+-3776AB?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-All_Rights_Reserved-red.svg)](LICENSE)

> **⚠️ Academic Embargo Notice:**

> The core mathematical formulations, adaptive covariance mechanisms, and quantitative benchmark results associated with this architecture are currently under peer review. Full algorithmic details and evaluation metrics will be updated upon publication.
>
> The companion UFACTORY xArm7 simulation testbed and raw benchmark logs are maintained in [`abdulmumeen-abdullahi/xarm7_generalisedUKF_experiment`](https://github.com/abdulmumeen-abdullahi/xarm7_generalisedUKF_experiment) *(currently **private** during peer review; will be made public upon publication)*.


## Overview
`ukf_dt_sync` is a generalized, hardware-agnostic ROS 2 package providing a reconfigurable Unscented Kalman Filter (UKF) framework for robotic joint state estimation. Designed to handle severe observation latency in distributed or computationally bound systems, the architecture dynamically scales to any $N$-Degree of Freedom ($N$-DOF) robotic manipulator without requiring source code modifications.

By abstracting kinematic parameters, controller topics, and noise settings into a single YAML configuration file, this package serves as a pure state-estimation backend capable of interfacing with diverse simulation testbeds or `/clock`-synchronized setups.

## Core Features
- **$N$-DOF Generalization:** Dynamically scales the state vector, sigma points, and covariance matrices from configuration parameters, eliminating hardcoded joint constraints.
- **Latency Compensation:** Fuses delayed sensory feedback with buffered controller reference commands and predictive forward modeling to reconstruct real-time joint positions and velocities.
- **Statistical Consistency Monitoring:** Computes and publishes Normalized Innovation Squared (NIS) metrics alongside $\chi^2$ consistency validation.
- **Hardware-Agnostic Abstraction:** Completely decoupled from specific robot URDFs or hardware drivers.
- **Decoupled Data Logging:** Automatically exports benchmark metrics and time-series logs outside the active Git workspace to preserve repository integrity.

## Node Architecture
The package is composed of four decoupled ROS 2 nodes orchestrating the estimation and benchmarking pipeline:

1. **`trajectory_executor`**: A `FollowJointTrajectory` action client that reads configurable $N$-DOF joint-space waypoints from YAML and cyclically commands the manipulator through continuous reference motions.
2. **`injector`**: A synthetic channel-degradation node that buffers incoming `/joint_states` to introduce configurable network latency (`delay_ms`) and injects zero-mean Gaussian measurement noise, publishing degraded feedback to `/delayed_joint_states`.
3. **`ukf_predictor`**: The primary ROS 2 estimation node wrapping the core `ukf.py` engine. It fuses delayed joint-position measurements with buffered controller reference velocities (`JointTrajectoryControllerState`), compensates for observation latency via forward prediction to publish real-time position and velocity estimates (`/predicted_joint_states`), and outputs Normalized Innovation Squared (NIS) consistency metrics (`/ukf_nis`).
4. **`data_logger`**: An automated benchmarking node that samples ground-truth, delayed baseline, and UKF-predicted joint states at 10 Hz over a 60-second trial window. It computes position error metrics (RMSE, MAE, percentage improvement) alongside $\chi^2$ NIS filter-consistency statistics, exports CSV logs (`UKF_pred_results.csv`, `UKF_nis_results.csv`, and `<delay_ms>ms.csv`) to a decoupled results directory, and signals pipeline shutdown upon trial completion.


## Companion Testbed & Experimental Results
Because `ukf_dt_sync` is decoupled from robot-specific URDFs and controllers, full closed-loop simulation benchmarks are executed alongside the companion testbed repository:

- **Repository:** [`abdulmumeen-abdullahi/xarm7_generalisedUKF_experiment`](https://github.com/abdulmumeen-abdullahi/xarm7_generalisedUKF_experiment) *(Private — under peer review)*
- **Included Packages & Artifacts:**
  - `ufactory_xarm7_description`: Robot URDF/Xacro kinematic and visual descriptions.
  - `ufactory_xarm7_gazebo`: Gazebo (`gz sim`) simulation environment and `/clock` bridge configuration.
  - `ufactory_xarm7_moveit_config`: MoveIt 2 and `ros2_control` joint trajectory controller configurations.
  - `experiment_results`: Recorded RMSE, MAE, $\chi^2$ NIS consistency CSVs, and time-series trajectories across evaluated latency scenarios.

## Installation

### Prerequisites
- ROS 2 (Jazzy)
- Python 3.x
- `numpy`, `scipy`

### Build Instructions
1. **Create the workspace and clone the repository:**
   ```bash
   mkdir -p reconfigurable_ukf_ws/src
   cd reconfigurable_ukf_ws/src
   git clone https://github.com/abdulmumeen-abdullahi/reconfigurable_UKF.git
   cd ..
   ```

Ensure this repository is cloned into the `src/` directory of your ROS 2 workspace.

2. **Build the ROS 2 package**

```bash
colcon build --symlink-install
source install/setup.bash
```

## Configuration
Before launching, update `src/config/default_config.yaml` to match your target manipulator and local environment:

| Parameter | Description |
| :--- | :--- |
| `dof` | Number of manipulator joints ($N$). Automatically sizes the $2N$ state vector $[q, \dot{q}]^T$ and covariance matrices. |
| `joint_names` | Ordered list of $N$ joint names matching those published on `/joint_states`. |
| `controller_action_topic` | Action server topic for trajectory execution (e.g., `/xarm7_controller/follow_joint_trajectory`). |
| `controller_state_topic` | Topic publishing `JointTrajectoryControllerState` reference velocities (defaults to `/xarm7_controller/controller_state`). |
| `waypoints` | Flattened 1D list of target joint positions in radians (automatically chunked into $N$-DOF waypoints). |
| `waypoint_duration_sec` | Execution duration (in seconds) allocated per waypoint transition. |
| `results_output_dir` | Local directory path (supports `~`) where evaluation CSVs and time-series logs are saved. |

## Usage
Launch the robot testbed (with its URDF, controllers, and `/clock` active) in one terminal, then bring up the estimation pipeline:

```bash
ros2 launch ukf_dt_sync ukf_pipeline.launch.py
```

To benchmark a specific synthetic communication delay (in milliseconds), pass the `delay_ms` launch argument:

```bash
ros2 launch ukf_dt_sync ukf_pipeline.launch.py delay_ms:=100
```

> **Important Notes on Execution & Clock Synchronization:**
> - **Simulation Time (`/clock`):** Both `injector` and `ukf_predictor` operate with `use_sim_time` enabled internally so latency buffering and timestamp alignment stay synchronized with simulation time. Ensure a `/clock` topic is actively bridged or published (e.g., from Gazebo `gz sim`) before launching; otherwise, timestamped buffer propagation will remain at `0.0`.
> - **Automated Trial Shutdown:** Each benchmark run executes for a fixed **60-second** sampling window in `data_logger`. Once the 60s window elapses and all CSV artifacts are written to `results_output_dir`, `data_logger` automatically shuts down the pipeline.