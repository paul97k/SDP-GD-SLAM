# SDP-GD-SLAM

An underwater simulation framework for evaluating SLAM algorithms using synthetic sonar, visual, and model-based approaches. The project enables reproducible experiments in complex underwater environments with tunable terrain and current conditions.
---
note: the development has slowed down and stopped completely
## Table of Contents

* [Description](#description)
* [Installation](#installation)
* [Usage](#usage)
* [Simulation Architecture](#simulation-architecture)
* [Data Flow](#data-flow)
* [Customizing the Simulation](#customizing-the-simulation)
* [Data Processing](#data-processing)
* [Support](#support)
* [Contributing](#contributing)
* [Authors and Acknowledgment](#authors-and-acknowledgment)
* [License](#license)
* [Project Status](#project-status)

---

## Description

SDP-GD-SLAM enables the testing and evaluation of multiple SLAM algorithms—Sonar-based SLAM, ORB-SLAM2 (visual), and SDP-GD (model-based)—under various simulated underwater environments.

Key components:

* **Sonar-Based SLAM**: Implemented and run on **Ubuntu 16.04** with ROS Melodic, using the Desistek SAGA robot and DVL, IMU, and sonar sensors.
* **ORB-SLAM2 (Visual SLAM)**: Executed on **Ubuntu 22.04** using the BlueROV2 model in a ROS2/Gazebo environment.
* **SDP-GD Algorithm**: A MATLAB-based method used to compute an output map from control inputs and measurements. This algorithm runs entirely on **Windows**.
* **Evaluation & Comparison**: All output maps (from Sonar SLAM, ORB-SLAM2, and SDP-GD) are compared in MATLAB on **Windows** using `.mat` files derived from ROS bag recordings.

A **Blender** file is included to generate seabed maps with varying complexity levels, which are used to test algorithm robustness under different environmental conditions.

---

## Installation

### Requirements

* ROS Melodic (Ubuntu 16.04) – for sonar-based SLAM
* ROS2 Humble or compatible (Ubuntu 22.04) – for ORB-SLAM2 simulation
* MATLAB (Windows) – for SDP-GD and output map evaluation
* Blender (any platform) – for generating seabed meshes
* Python 3.x
* Gazebo 9+

### Clone Required Repositories

First, clone the base simulation repositories in the appropriate Ubuntu environments:

```bash
# Ubuntu 16.04 (ROS Melodic)
cd ~/catkin_ws/src
git clone https://github.com/Tim-HW/HW-BlueRov2-Sonar-based-SLAM.git

# Ubuntu 22.04 (ROS2)
cd ~/colcon_ws/src
git clone https://github.com/clydemcqueen/orca4.git
```

### Replace with Modified Files

> **Important**: After cloning the repositories above, replace the corresponding files and folders with the edited versions provided in **this SDP-GD-SLAM repository**.
> These include modified launch files, model definitions, trajectory scripts, evaluation scripts, and recorded datasets required for correct simulation and analysis.
> Some files are newly generated or corrected versions that do not exist in the original repositories.

After replacing the files, build each workspace:

```bash
# For ROS Melodic
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# For ROS2
cd ~/colcon_ws
colcon build --symlink-install
source install/setup.bash
```


Build the workspaces accordingly.

---

## Usage

### Sonar-Based SLAM (Ubuntu 16.04)

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch sonar_mapping sim.launch
```

### ORB-SLAM2 Simulation (Ubuntu 22.04)

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
ros2 launch orca_bringup sim_launch.py
```

---

## Simulation Architecture

| Platform      | Purpose                            | OS            | Tools                        |
| ------------- | ---------------------------------- | ------------- | ---------------------------- |
| **Windows**   | Output map calculation, plotting   | Windows 10/11 | MATLAB (SDP-GD, comparison)  |
| **Ubuntu 16** | Sonar-based SLAM simulation        | Ubuntu 16.04  | ROS Melodic, Gazebo, DVL/IMU |
| **Ubuntu 22** | Visual SLAM simulation (ORB-SLAM2) | Ubuntu 22.04  | ROS2 Humble, ORCA4, Gazebo   |

---

## Data Flow

1. **Simulate trajectories** in either the sonar-based (Ubuntu 16) or visual SLAM (Ubuntu 22) environments.
2. **Record data** using ROS bag with relevant topics (e.g., `/pose`, `/sonar`, `/odom`).
3. **Convert `.bag` files to `.mat`** format using the provided Python script on Ubuntu 16.
4. **Transfer `.mat` files to Windows**, where the SDP-GD algorithm processes the input data and generates output maps.
5. **Evaluate and compare** results between SDP-GD, ORB-SLAM2, and sonar-based SLAM using MATLAB.

---

## Customizing the Simulation

* **Trajectory:**
  `/catkin_ws/src/uuv_simulator/uuv_teleop/scripts/go_straight.py`

* **Controller:**
  `/catkin_ws/src/desistek_saga/desistek_saga_control/launch/my_start_nmb_sm_control.launch`

* **Seabed Map:**
  `/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/models/herkules_seabed/model.sdf`

* **Generate terrain in Blender:**
  Use the included `.blend` file to create custom terrain meshes. Export them as `.dae` for Gazebo integration.

---

## Data Processing

### Convert `.bag` to `.mat` (Ubuntu 16)

1. Edit the file:

   ```
   /catkin_ws/src/data_bags/class_storedata.py
   ```
2. Update `processFilenames` with the bag filenames.
3. Run:

   ```bash
   python3 class_storedata.py
   ```

---

## Support

paulkarto@live.com

---

## Contributing

You're welcome to contribute. Please fork the project, make changes on a feature branch, and submit a merge request. Contributions should include:

* A clear description
* Code comments
* Tests or demonstration scripts if applicable

---

## Authors and Acknowledgment

* **Paul Kartoidjojo**, TU Delft
  Thesis: *Comparative Evaluation of SLAM Algorithms in Simulated Underwater Environments*
* Supervised by: **Dr. M. (Manuel) Mazo Espinosa
 and Sasan Vakili**
This project builds on the following open-source repositories:

- [orca4 (MIUT fork)](https://github.com/miut-orca/orca4) – adapted for ROS2 and BlueROV2 simulation  
- [HW-BlueRov2-Sonar-based-SLAM](https://github.com/Tim-HW/HW-BlueRov2-Sonar-based-SLAM) – sonar SLAM implementation for Desistek SAGA  
- [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) – for visual SLAM localization

---

Currently in use for academic experimentation. Planned additions include performance evaluation under varying current profiles and real-time loop closure analysis.
