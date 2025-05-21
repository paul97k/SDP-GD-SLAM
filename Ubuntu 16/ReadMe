---

# Desistek SAGA + BlueROV2 Sonar SLAM Simulation Guide

This guide explains how to set up and run simulations using the Desistek SAGA AUV and BlueROV2 with Sonar-based SLAM. It includes instructions for launching the simulator, recording data, converting it for analysis, and customizing key components.

---

## 1. Setup Instructions

### Clone and Install Required Repositories

#### a. Desistek SAGA Simulation

Follow the instructions in:

[https://github.com/clydemcqueen/orca4/tree/main](https://github.com/clydemcqueen/orca4/tree/main)

#### b. Sonar-Based SLAM (BlueROV2)

Follow and install dependencies from:

[https://github.com/Tim-HW/HW-BlueRov2-Sonar-based-SLAM](https://github.com/Tim-HW/HW-BlueRov2-Sonar-based-SLAM)

Make sure both repositories are cloned and installed in your ROS workspace (`~/catkin_ws/src`), and build the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 2. Running the Simulation

### Source the Environment

In a new terminal:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Launch the Simulator

```bash
roslaunch sonar_mapping sim.launch
```

---

## 3. Recording Simulation Data

Use the following command to store the relevant topics in a ROS bag:

```bash
rosbag record -o my_rosbag \
  /desistek_saga/cmd_vel \
  /desistek_saga/dp_controller/trajectory \
  /desistek_saga/dp_controller/waypoints \
  /desistek_saga/dvl \
  /desistek_saga/ground_truth_to_tf_desistek_saga/euler \
  /desistek_saga/ground_truth_to_tf_desistek_saga/pose \
  /desistek_saga/imu \
  /desistek_saga/pose_gt \
  /desistek_saga/sonar \
  /desistek_saga/dvl_sonar0 \
  /desistek_saga/dvl_sonar1 \
  /desistek_saga/dvl_sonar2 \
  /desistek_saga/dvl_sonar3 \
  /desistek_saga/thrusters/0/thrust/ \
  /desistek_saga/thrusters/1/thrust/ \
  /desistek_saga/thrusters/2/thrust/ \
  /desistek_saga/thrusters/0/input \
  /desistek_saga/thrusters/1/input \
  /desistek_saga/thrusters/2/input \
  /odom \
  /sonar/PC2 \
  /hydrodynamics/current_velocity
```

---

## 4. Data Conversion (ROS Bag to `.mat` for MATLAB)

To convert `.bag` files to `.mat`:

1. Open:
   `/catkin_ws/src/data_bags/class_storedata.py`

2. Modify the `processFilenames` list with the names of your recorded bag files.

3. Run the script in a terminal or from within a Python environment.

---

## 5. Customizing the Simulation

### Change Trajectory

Edit:

```bash
/catkin_ws/src/uuv_simulator/uuv_teleop/scripts/go_straight.py
```

---

### Change Controller

Edit:

```bash
/catkin_ws/src/desistek_saga/desistek_saga_control/launch/my_start_nmb_sm_control.launch
```

---

### Change the Map

Edit the seabed model file:

```bash
/catkin_ws/src/uuv_simulator/uuv_gazebo_worlds/models/herkules_seabed/model.sdf
```

You can choose different seabed complexity levels by modifying the `<uri>`:

```xml
<uri>model://mine/S0.22_D10_R0.5_D12.dae</uri>
<!-- Other options:
     model://mine/S0.22_D10_R0.5_D0.dae
     model://mine/S0.22_D10_R0.5_D1.dae
     ...
     model://mine/S0.22_D10_R0.5_D10.dae
-->
```

---

### Simulate Water Currents

While the simulator is running, execute the following services in a terminal:

```bash
rosservice call /hydrodynamics/set_current_velocity_model \
  "{mean: 0.2, min: 0.1, max: 0.3, noise: 0.1, mu: 0.0}"

rosservice call /hydrodynamics/set_current_horz_angle_model \
  "{mean: 0.0, min: -0.174533, max: 0.174533, noise: 0.1, mu: 0.0}"

rosservice call /hydrodynamics/set_current_vert_angle_model \
  "{mean: 0.0, min: -0.174533, max: 0.174533, noise: 0.1, mu: 0.0}"
```

---
