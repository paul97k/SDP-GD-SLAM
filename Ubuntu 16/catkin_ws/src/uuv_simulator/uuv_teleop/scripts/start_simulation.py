#!/usr/bin/env python
import subprocess
import time
# Start the ROS launch file
roslaunch_process = subprocess.Popen(["roslaunch", "sonar_mapping", "sim.launch"])

# Start rosbag recording
rosbag_process = subprocess.Popen([
    "rosbag", "record", "-o", "my_rosbag",
    "/desistek_saga/cmd_vel",
    "/desistek_saga/dp_controller/trajectory",
    "/desistek_saga/dp_controller/waypoints",
    "/desistek_saga/dvl",
    "/desistek_saga/ground_truth_to_tf_desistek_saga/euler",
    "/desistek_saga/ground_truth_to_tf_desistek_saga/pose",
    "/desistek_saga/imu",
    "/desistek_saga/pose_gt",
    "/desistek_saga/sonar",
    "/desistek_saga/dvl_sonar0",
    "/desistek_saga/dvl_sonar1",
    "/desistek_saga/dvl_sonar2",
    "/desistek_saga/dvl_sonar3",
    "/desistek_saga/thrusters/0/thrust/",
    "/desistek_saga/thrusters/1/thrust/",
    "/desistek_saga/thrusters/2/thrust/",
    "/desistek_saga/thrusters/0/input",
    "/desistek_saga/thrusters/1/input",
    "/desistek_saga/thrusters/2/input",
    "/odom",
    "/sonar/PC2",
    "/hydrodynamics/current_velocity"
])
# Run for 40 seconds
time.sleep(40)

# Stop rosbag and the ROS launch process
rosbag_process.terminate()
roslaunch_process.terminate()
