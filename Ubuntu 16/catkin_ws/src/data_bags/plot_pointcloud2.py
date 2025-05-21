import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2  # Helper function to read PointCloud2 data
import time

# Define the bag file path
file_name = "data_bags/my_rosbag_2025-02-09-12-48-58.bag"

file_name = "data_bags/S0.22_D10_R0.5_D0_withoutAnything.bag"
# Topic to extract
topic = "/sonar/PC2"

# Initialize the figure
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection="3d")

# Initialize empty lists to store cumulative points
all_x, all_y, all_z = [], [], []

# Set fixed axis limits
ax.set_xlim([0, 20])  # Adjust based on expected range
ax.set_ylim([-20, 20])  
ax.set_zlim([-5, 5])
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3D Sonar PointCloud2")

# Open the bag file and extract data
with rosbag.Bag(file_name, "r") as bag:
    prev_time = None
    for _, msg, t in bag.read_messages(topics=[topic]):
        # Convert ROS time to seconds
        curr_time = t.to_sec()
        
        # Only update every 0.01 seconds
        if prev_time is None or (curr_time - prev_time) >= 2:
            prev_time = curr_time
            
            # Extract point cloud data
            new_x, new_y, new_z = [], [], []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x= -point[1]
                y= point[0]
                new_x.append(x)
                new_y.append(y)
                new_z.append(point[2])

            # Append new points to the cumulative lists
            all_x.extend(new_x)
            all_y.extend(new_y)
            all_z.extend(new_z)

            # Convert lists to NumPy arrays
            all_x_np = np.array(all_x)
            all_y_np = np.array(all_y)
            all_z_np = np.array(all_z)

            # Plot the updated scatter points
            ax.scatter(all_x_np, all_y_np, all_z_np, c=all_z_np, cmap="viridis", s=2)

            # Update title with timestamp
            ax.set_title("3D Sonar PointCloud2 at t={:.2f} sec".format(curr_time))

            plt.pause(0.0001)  # Pause for animation

plt.show()  # Show final frame
# Define the output file path
output_file = "sonar_pointcloud.npz"

# Convert lists to NumPy arrays
all_x_np = np.array(all_x)
all_y_np = np.array(all_y)
all_z_np = np.array(all_z)

# # Save the point cloud data
# np.savez(output_file, x=all_x_np, y=all_y_np, z=all_z_np)

# print("Saved point cloud data to ")
