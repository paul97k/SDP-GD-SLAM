# import rosbag
# import numpy as np
# import matplotlib.pyplot as plt
# from sensor_msgs.msg import LaserScan

# # Define the bag file path
# file_name = "data_bags/my_rosbag_2025-02-07-16-31-40.bag"
# file_name = "data_bags/my_rosbag_2025-02-07-16-42-15.bag"

# # Topic to extract
# topic = "/desistek_saga/sonar"

# # Store sonar data
# scan_data = {"angles": [], "ranges": [], "timestamps": []}

# # Open the bag file and extract data
# with rosbag.Bag(file_name, "r") as bag:
#     for _, msg, t in bag.read_messages(topics=[topic]):
#         scan_data["timestamps"].append(t.to_sec())  # Convert ROS time to seconds

#         # Extract range and angle data
#         angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
#         scan_data["angles"].extend(angles)
#         scan_data["ranges"].extend(msg.ranges)

# # Convert to NumPy arrays
# angles = np.array(scan_data["angles"])
# ranges = np.array(scan_data["ranges"])

# # Convert polar to Cartesian
# x = ranges * np.cos(angles)
# y = ranges * np.sin(angles)

# # Plot the 2D sonar scan
# plt.figure(figsize=(8, 6))
# plt.scatter(x, y, c=ranges, cmap="viridis", s=5)
# plt.colorbar(label="Range (m)")

# # Customize the plot
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.title("2D Sonar Scan from LaserScan Data")
# plt.axis("equal")  # Keep aspect ratio
# plt.grid()

# plt.show()
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import time

# Define the bag file path
file_name = "data_bags/my_rosbag_2025-02-07-16-42-15.bag"
file_name = "data_bags/my_rosbag_2025-02-09-12-48-58.bag"
file_name = "data_bags/S0.22_D10_R0.5_D0_withoutAnything.bag"

# Topic to extract
topic = "/desistek_saga/sonar"

# Open the bag file and extract data
with rosbag.Bag(file_name, "r") as bag:
    plt.figure(figsize=(8, 6))  # Initialize the plot
    for _, msg, t in bag.read_messages(topics=[topic]):
        plt.clf()  # Clear the previous plot

        # Convert range data to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        z = np.array(msg.ranges) * np.cos(angles)
        x = np.array(msg.ranges) * np.sin(angles)

        # Scatter plot
        plt.scatter(-x, z, c=msg.ranges, cmap="viridis", s=5)
        plt.colorbar(label="Range (m)")

        # Customize the plot
        plt.xlabel("X (m)")
        plt.ylabel("Z (m)")
        plt.title("2D Sonar Scan at t=sec")
        plt.axis("equal")  # Keep aspect ratio
        plt.grid()

        plt.pause(0.01)  # Pause for 1 second before the next scan

    plt.show()  # Show the final plot after all scans are displayed
# Define storage
scan_data = {"angles": [], "ranges": [], "timestamps": []}

# Open the bag file and extract data
with rosbag.Bag(file_name, "r") as bag:
    for _, msg, t in bag.read_messages(topics=[topic]):
        scan_data["timestamps"].append(t.to_sec())  # Convert ROS time to seconds

        # Extract range and angle data
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        scan_data["angles"].append(angles)
        scan_data["ranges"].append(msg.ranges)

# Convert lists to NumPy arrays
angles_np = np.array(scan_data["angles"])
ranges_np = np.array(scan_data["ranges"])
timestamps_np = np.array(scan_data["timestamps"])

# # Save to an `.npz` file
# output_file = "sonar_scan_data.npz"
# np.savez(output_file, angles=angles_np, ranges=ranges_np, timestamps=timestamps_np)

# print("Saved sonar scan data to")
