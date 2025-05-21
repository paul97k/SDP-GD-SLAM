import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Range
from uuv_sensor_ros_plugins_msgs.msg import DVL
from geometry_msgs.msg import Twist

# Define the bag file path
file_name = "data_bags/my_rosbag_2025-02-28-14-06-37.bag"


cmd_vel_topic = "/desistek_saga/cmd_vel"  # New topic

# Data storage
cmd_vel_data = {"linear_x": [], "linear_y": [], "linear_z": [], "angular_x": [], "angular_y": [], "angular_z": []}
cmd_vel_time_stamps = []

# Read the bag file
with rosbag.Bag(file_name, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[cmd_vel_topic]):
        cmd_vel_time_stamps.append(t.to_sec())

        # Extract linear velocities
        cmd_vel_data["linear_x"].append(msg.linear.x)
        cmd_vel_data["linear_y"].append(msg.linear.y)
        cmd_vel_data["linear_z"].append(msg.linear.z)

        # Extract angular velocities
        cmd_vel_data["angular_x"].append(msg.angular.x)
        cmd_vel_data["angular_y"].append(msg.angular.y)
        cmd_vel_data["angular_z"].append(msg.angular.z)

# Convert lists to NumPy arrays
cmd_vel_time_np = np.array(cmd_vel_time_stamps)
cmd_vel_linear_x = np.array(["linear_x"])
cmd_vel_linear_y = np.array(cmd_vel_data["linear_y"])
cmd_vel_linear_z = np.array(cmd_vel_data["linear_z"])
cmd_vel_angular_x = np.array(cmd_vel_data["angular_x"])
cmd_vel_angular_y = np.array(cmd_vel_data["angular_y"])
cmd_vel_angular_z = np.array(cmd_vel_data["angular_z"])
print(np.shape(cmd_vel_linear_x))
# Plot linear velocities
plt.figure(figsize=(10, 5))
plt.plot(cmd_vel_time_np, cmd_vel_linear_x, label="Linear X")
plt.plot(cmd_vel_time_np, cmd_vel_linear_y, label="Linear Y")
plt.plot(cmd_vel_time_np, cmd_vel_linear_z, label="Linear Z")
plt.xlabel("Time (s)")
plt.ylabel("Linear Velocity (m/s)")
plt.legend()
plt.title("Linear Velocities from /cmd_vel")
plt.grid()
plt.show()


# # Plot angular velocities
# plt.figure(figsize=(10, 5))
# plt.plot(cmd_vel_time_np, cmd_vel_angular_x, label="Angular X")
# plt.plot(cmd_vel_time_np, cmd_vel_angular_y, label="Angular Y")
# plt.plot(cmd_vel_time_np, cmd_vel_angular_z, label="Angular Z")
# plt.xlabel("Time (s)")
# plt.ylabel("Angular Velocity (rad/s)")
# plt.legend()
# plt.title("Angular Velocities from /cmd_vel")
# plt.grid()

# # Save data
# np.savez("cmd_vel_data.npz", time_stamps=cmd_vel_time_np, linear_x=cmd_vel_linear_x, linear_y=cmd_vel_linear_y, linear_z=cmd_vel_linear_z,
#          angular_x=cmd_vel_angular_x, angular_y=cmd_vel_angular_y, angular_z=cmd_vel_angular_z)

# print("Saved cmd_vel data")


