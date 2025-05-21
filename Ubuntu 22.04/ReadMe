# ORCA4 Simulation Setup and Usage Guide

This guide provides installation instructions, simulation usage, and information on modifying key components of the ORCA4 underwater robot simulator.

---

## Installation

Clone the repository and install dependencies:

```bash
git clone https://github.com/clydemcqueen/orca4.git
cd orca4
```

Install all necessary dependencies using `rosdep`:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Then build the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running the Simulator

To launch the full simulation environment:

```bash
ros2 launch orca_bringup sim_launch.py
```

---

## Recording a ROS Bag

To record the trajectory and mapping topics:

```bash
ros2 bag record \
  /mavros/local_position/pose \
  /model/orca4/odometry \
  /orb_slam2_stereo_node/pose \
  /orb_slam2_stereo_node/map_points \
  /lidar \
  /odom \
  -o Traj_50_50_-6_Ca_0.0_0.0_Li_0_0_v1
```

---

## Customizing the Simulation

### Change the Map

To modify the simulated seabed or terrain:

Edit the following file:

```bash
colcon_ws/src/bluerov2_gz/models/sand_heightmap/model.sdf
```

---

### Change the AUV Trajectory

To update or customize the AUV's path:

Edit the mission script:

```bash
colcon_ws/src/orca4/orca_bringup/scripts/mission_runner.py
```

---

### Change Sensor Specifications

To adjust sensor configurations (e.g., LIDAR, camera):

Edit the model definition:

```bash
colcon_ws/src/orca4/orca_description/models/orca4/model.sdf
```

---

## Run ICP Algorithm

Run custom ICP-based localization or mapping algorithm:

```bash
python3 orca4/orca_bringup/scripts/my_icp.py
```

---

## Notes

* Ensure that Gazebo, MAVROS, and ORB-SLAM2 are correctly installed and sourced.
* Use `rviz2` for visualizing the trajectory, SLAM output, and point clouds.


