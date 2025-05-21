import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Explicit import for 3D plotting

file_name = "data_bags/my_rosbag_2025-02-10-13-43-40.bag"

# Topics to extract
pose_topic = "/desistek_saga/ground_truth_to_tf_desistek_saga/pose"
euler_topic = "/desistek_saga/ground_truth_to_tf_desistek_saga/euler"

# Lists to store extracted position data
timestamps_pose = []
x_positions = []
y_positions = []
z_positions = []

# Lists to store extracted Euler angles
timestamps_euler = []
roll_angles = []
pitch_angles = []
yaw_angles = []

# Open the bag file and extract data
with rosbag.Bag(file_name, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[pose_topic, euler_topic]):
        if topic == pose_topic:
            timestamps_pose.append(t.to_sec())
            x_positions.append(msg.pose.position.x)
            y_positions.append(msg.pose.position.y)
            z_positions.append(msg.pose.position.z)
        elif topic == euler_topic:
            timestamps_euler.append(t.to_sec())
            roll_angles.append(msg.vector.x)   # Roll
            pitch_angles.append(msg.vector.y)  # Pitch
            yaw_angles.append(msg.vector.z)    # Yaw

# Convert to NumPy arrays for easier plotting
timestamps_pose = np.array(timestamps_pose)
x_positions = np.array(x_positions)
y_positions = np.array(y_positions)
z_positions = np.array(z_positions)

timestamps_euler = np.array(timestamps_euler)
roll_angles = np.array(roll_angles)
pitch_angles = np.array(pitch_angles)
yaw_angles = np.array(yaw_angles)




# Plot Euler angles over time
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax[0].plot(timestamps_pose, x_positions, label=" (X)", color="r")
ax[0].set_ylabel("Roll (rad)")
ax[0].legend()

ax[1].plot(timestamps_pose, y_positions, label=" (Y)", color="g")
ax[1].set_ylabel("Pitch (rad)")
ax[1].legend()

ax[2].plot(timestamps_pose, z_positions, label=" (Z)", color="b")
ax[2].set_xlabel("Time (s)")
ax[2].set_ylabel("Yaw (rad)")
ax[2].legend()

plt.suptitle("position Over Time")




# Plot Euler angles over time
fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

ax[0].plot(timestamps_euler, roll_angles, label="Roll (X)", color="r")
ax[0].set_ylabel("Roll (rad)")
ax[0].legend()

ax[1].plot(timestamps_euler, pitch_angles, label="Pitch (Y)", color="g")
ax[1].set_ylabel("Pitch (rad)")
ax[1].legend()

ax[2].plot(timestamps_euler, yaw_angles, label="Yaw (Z)", color="b")
ax[2].set_xlabel("Time (s)")
ax[2].set_ylabel("Yaw (rad)")
ax[2].legend()

plt.suptitle("Euler Angles Over Time")
plt.show()
