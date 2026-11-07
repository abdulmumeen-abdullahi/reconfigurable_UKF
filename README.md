# Reconfigurable N-DOF UKF State Estimation Architecture

[![ROS 2](https://img.shields.io/badge/ROS_2-Compatible-22314E?logo=ros)](https://docs.ros.org/)
[![Python 3](https://img.shields.io/badge/Python-3.x-3776AB?logo=python)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> **⚠️ Academic Embargo Notice**
> The core mathematical formulations, adaptive covariance mechanisms, and quantitative benchmark results associated with this architecture are currently under peer review. Full algorithmic details and evaluation metrics will be updated upon publication.

## Overview
`ukf_dt_sync` is a generalized, hardware-agnostic ROS 2 package providing a reconfigurable Unscented Kalman Filter (UKF) framework for robotic joint state estimation. Designed to handle severe observation latency in distributed or computationally bound systems, the architecture dynamically scales to any $N$-Degree of Freedom (N-DOF) robotic manipulator without requiring source code modifications.

By abstracting the kinematic and dynamic properties into configurable parameters, this package serves as a pure state-estimation backend capable of interfacing with diverse physical hardware or simulation testbeds.

## Core Features
* **$N$-DOF Generalization:** Dynamically scales the state vector and covariance matrices based on a single configuration parameter, eliminating hardcoded joint constraints.
* **Latency Compensation:** Integrates delayed sensory feedback with predictive forward-modeling to estimate real-time joint states.
* **Hardware-Agnostic Abstraction:** Completely decoupled from specific robot URDFs or hardware drivers.
* **Decoupled Data Logging:** Automatically manages experimental data storage outside the active Git workspace to maintain repository integrity.

## Node Architecture
The package is composed of several decoupled ROS 2 nodes, orchestrating the estimation pipeline:

1. **`trajectory_executor`**: Generates and commands reference joint trajectories for the $N$-DOF system.
2. **`injector`**: A synthetic delay induction node utilized for benchmarking filter robustness against network latency.
3. **`ukf_predictor`**: The primary ROS 2 wrapper interfacing with the core `ukf.py` mathematical engine, publishing corrected state estimates.
4. **`data_logger`**: An automated, isolated logging node that writes time-series metrics and estimation comparisons to a decoupled directory.

## Installation

### Prerequisites
* ROS 2 (Jazzy)
* Python 3.x
* `numpy`, `scipy`, `pandas`

### Build Instructions
#### - Create the workspace and Clone the repository
```bash
mkdir -p reconfigurable_ukf_ws/src
cd reconfigurable_ukf_ws/src
git clone https://github.com/abdulmumeen-abdullahi/reconfigurable_UKF.git
cd ..
```

Ensure this repository is cloned into the `src/` directory of your ROS 2 workspace.

#### - Build the ROS 2 package

```bash
colcon build --symlink-install
source install/setup.bash
```

#### - Usage
The entire estimation pipeline is brought up using the provided composition launch file.

```bash
ros2 launch ukf_dt_sync ukf_pipeline.launch.py
```

(Note: To run full physical or simulated experiments, this package must be launched in tandem with a robot-specific testbed repository containing the URDF and hardware controllers).
