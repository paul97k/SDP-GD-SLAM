import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from class_icp import Align2D  # Import your ICP implementation

# Define the bag file path
file_name = "data_bags/my_rosbag_2025-02-09-12-48-58.bag"
topic = "/sonar/PC2"

# List to store scans for ICP
scan_list = []

# Open the bag file and extract data
with rosbag.Bag(file_name, "r") as bag:
    prev_time = None
    for _, msg, t in bag.read_messages(topics=[topic]):
        # Convert ROS time to seconds
        curr_time = t.to_sec()

        # Only update every 2 seconds
        if prev_time is None or (curr_time - prev_time) >= 2:
            prev_time = curr_time

            # Extract point cloud data
            scan = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x = point[0]  # Swap X and Y for correct sonar frame
                y = point[1]
                z = point[2]
                scan.append([x, y, z, 1])  # Homogeneous coordinates

            # Convert scan to NumPy array
            scan_np = np.array(scan)

            if scan_np.shape[0] == 0:
                print("Warning: Empty scan at timestamp {} - Skipping.".format(t.to_sec()))

                continue  # Skip empty scans

            # Store only valid scans
            scan_list.append(scan_np)


# Define the output file path
output_file = "scans_data.npz"

# Save scan_list as an npz file
np.savez(output_file, *scan_list)

print("Saved ")


# Convert scans to a NumPy array list
print("Extracted {} scans from ROS bag.".format(len(  scan_list  )))
if len(scan_list) < 2:
    print("Not enough scans for ICP alignment.")
else:
    # Convert all scans to NumPy arrays (if not already)
    scan_list = [np.array(scan) for scan in scan_list]

    # Initialize the first scan as the reference
    aligned_scans = [scan_list[0][:, :3]]  # Store only (x, y, z)

    # Initial transformation guess (identity matrix)
    init_T = np.eye(3)

    for i in range(1, len(scan_list)):
        print("Aligning scan {} with scan {} using ICP...".format(i, i-1))


        # Run ICP
        icp = Align2D(scan_list[i][:, :3], aligned_scans[-1], init_T)
        transformed_scan, error = icp.transform  # Output transformed scan

        # Store the aligned scan
        aligned_scans.append(transformed_scan[:, :3])

    print("ICP alignment completed.")



fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")

for scan in aligned_scans:
    ax.scatter(scan[:, 0], scan[:, 1], scan[:, 2], s=2)

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Aligned Sonar Point Clouds (ICP)")




import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

# Initialize figure for real-time animation
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")
plt.ion()  # Enable interactive mode

# Set fixed axis limits
ax.set_xlim([0, 20])  
ax.set_ylim([-20, 20])  
ax.set_zlim([-5, 5])
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Aligned Sonar Point Clouds (ICP)")

# Lists to store cumulative points
all_x, all_y, all_z = [], [], []

# Iterate over aligned scans and show points one-by-one
for scan in aligned_scans:
    for point in scan:
        # Extract x, y, z coordinates
        x, y, z = point[0], point[1], point[2]

        # Append new point to the cumulative list
        all_x.append(x)
        all_y.append(y)
        all_z.append(z)

        # Clear the plot and redraw with new and previous points
        ax.cla()
        # ax.set_xlim([0, 20])  
        # ax.set_ylim([-20, 20])  
        # ax.set_zlim([-5, 5])
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("Aligned Sonar Point Clouds (ICP)")

        # Plot all points (old + new)
        ax.scatter(all_x, all_y, all_z, c=all_z, cmap="viridis", s=2)

        plt.pause(0.01)  # Pause to create real-time effect

# Keep final frame displayed
plt.ioff()  # Disable interactive mode
plt.show()
