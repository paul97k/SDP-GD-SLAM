import rosbag
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tf_trans
from sensor_msgs.msg import Imu

# Define the bag file path
file_name = "data_bags/my_rosbag_2025-02-09-12-48-58.bag"

# Lists to store extracted data
timestamps = []
rolls, pitches, yaws = [], [], []
angular_velocities = []
linear_accelerations = []

# Read the bag file
with rosbag.Bag(file_name, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=["/desistek_saga/imu"]):
        timestamps.append(t.to_sec())  # Convert ROS time to seconds
        
        # Convert quaternion to Euler angles
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        euler = tf_trans.euler_from_quaternion(quaternion)  # Roll, pitch, yaw
        rolls.append(euler[0])
        pitches.append(euler[1])
        yaws.append(euler[2])
        
        # Store angular velocity
        angular_velocities.append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        # Store linear acceleration
        linear_accelerations.append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

# Convert lists to numpy arrays
timestamps = np.array(timestamps)
angular_velocities = np.array(angular_velocities)
linear_accelerations = np.array(linear_accelerations)

# Plot orientation
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(timestamps, rolls, label="Roll", color="r")
plt.plot(timestamps, pitches, label="Pitch", color="g")
plt.plot(timestamps, yaws, label="Yaw", color="b")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Orientation (rad)")
plt.title("IMU Orientation (Euler Angles)")

# Plot angular velocity
plt.subplot(3, 1, 2)
plt.plot(timestamps, angular_velocities[:, 0], label="wx", color="r")
plt.plot(timestamps, angular_velocities[:, 1], label="wy", color="g")
plt.plot(timestamps, angular_velocities[:, 2], label="wz", color="b")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("IMU Angular Velocity")

# Plot linear acceleration
plt.subplot(3, 1, 3)
plt.plot(timestamps, linear_accelerations[:, 0], label="ax", color="r")
plt.plot(timestamps, linear_accelerations[:, 1], label="ay", color="g")
plt.plot(timestamps, linear_accelerations[:, 2], label="az", color="b")
plt.legend()
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s2)")
plt.title("IMU Linear Acceleration")

plt.tight_layout()
plt.show()
