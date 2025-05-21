import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate
import numpy as np
from scipy.ndimage import uniform_filter1d  # Alternative: np.convolve
import os
os.chdir("/home/paul/colcon_ws/src")
print("Current working directory:", os.getcwd())  # Verify


# Load the saved .npz file
loaded_data = np.load("orca4/orca_bringup/scripts/data npz/sonar_scan_data1.npz")
angles = loaded_data["angles"]
ranges = loaded_data["ranges"]
timestamps = loaded_data["timestamps"]

skip_i = 5  # how many indices to skip between scans
window_size = 20  # Adjust based on noise level
optimal_lag_it = 0
stored_signals = {}
scan_list = []

# Iterate over consecutive scans
for i in range(1, len(timestamps) - 1 - skip_i, skip_i):
# for i in range(1, 20- 1 - skip_i, skip_i):
    # plt.clf()  # Clear the previous plot
    # Create a fixed x-range for plotting (assumes both scans have the same length)
    z1 = ranges[i] * np.cos(angles[i])
    z2 = ranges[i + 1 + skip_i] * np.cos(angles[i + 1 + skip_i])
    z1 = moving_average(z1, window_size)
    z2 = moving_average(z2, window_size)

    t1 = np.linspace(-3, 3, len(z1))
    t2 = np.linspace(-3, 3, len(z2))

    optimal_lag, shifted_t = compute_optimal_lag(z1, z2, t1, t2, False)
    shifted_t = t2+optimal_lag

    scan = np.column_stack((np.array(t1+optimal_lag_it) , 
                            np.array(z1) , 
                            np.ones_like(z1) , 
                            np.ones_like(z1)))  # Homogeneous coordinates
    scan_list.append(scan)

    scan = np.column_stack((np.array(shifted_t+optimal_lag_it) , 
                            np.array(z2) , 
                            np.ones_like(z2) , 
                            np.ones_like(z2)))  # Homogeneous coordinates
    scan_list.append(scan)

    stored_signals[i] = {
        "t1": t1 + optimal_lag_it,
        "t2": t2 + optimal_lag_it,
        "shifted_t": shifted_t + optimal_lag_it,
        "z1": z1,
        "z2": z2,
        "optimal_lag": optimal_lag
    }

    optimal_lag_it = optimal_lag_it + optimal_lag

import sys
sys.path.append("/home/paul/colcon_ws/src/orca4/orca_bringup/scripts/")  # Add path where class_icp.py is located

from class_icp import Align2D



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

    for i in range(1, len(scan_list)-1):
        print("Aligning scan {} with scan {} using ICP...".format(i, i-1))

        
        first_scan = scan_list[i][:, :3]
        second_scan = scan_list[i+1][:, :3]

        cutPerc = 0

        cut_ids = round(cutPerc*len(first_scan[:,1]))
        first_scan = first_scan[cut_ids:,:]

        cut_ide= round((1-cutPerc)*len(second_scan[:,1]))
        second_scan = second_scan[:cut_ide,:]

        
        # Run ICP
        icp = Align2D(first_scan, second_scan, init_T)
        transformed_scan, error = icp.transform  # Output transformed scan

        # Store the aligned scan
        aligned_scans.append(transformed_scan[:, :3])

    print("ICP alignment completed.")



import matplotlib.pyplot as plt
import numpy as np

# Plot all scans
plt.figure(figsize=(8, 6))

for i, scan in enumerate(scan_list):
    plt.scatter(-scan[:, 0], -scan[:, 1], s=1, label=f"Scan {i}")  # s=1 makes points small

plt.xlabel("X")
plt.ylabel("Y")
plt.title("All Scans Overlay")
plt.xlim([-3,120])
plt.ylim([-5,-2])

import matplotlib.pyplot as plt

# Initialize figure for 2D scatter plot
fig, ax = plt.subplots(figsize=(10, 7))

# Plot only the X and Y axes
# plot 2
for scan in aligned_scans:
    ax.scatter(-scan[:, 0], -scan[:, 1], s=2)  # Only X and Y

# Set axis labels
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_title("Aligned Sonar Point Clouds (ICP) - X-Y Projection")
# ax.set_xlim([-100,100])
# ax.set_ylim([-100,100])
ax.set_xlim([-3,120])
ax.set_ylim([-5,-2])



import numpy as np
import matplotlib.pyplot as plt

# Extract X, Y, and Z values from original scans
original_x = np.concatenate([-scan[:, 0] for scan in scan_list])
original_y = np.concatenate([-scan[:, 1] for scan in scan_list])
original_z = np.concatenate([scan[:, 2] for scan in scan_list])

# Extract X, Y, and Z values from aligned scans (ICP result)
aligned_x = np.concatenate([-scan[:, 0] for scan in aligned_scans])
aligned_y = np.concatenate([-scan[:, 1] for scan in aligned_scans])
aligned_z = np.concatenate([scan[:, 2] for scan in aligned_scans])



merged_x, merged_y, merged_z = merge_scans(original_x, original_y, original_z, num_bins=500)
merged_x_a, merged_y_a, merged_z_a = merge_scans(aligned_x, aligned_y, aligned_z, num_bins=500)

# Plot the merged X-Y projection
import matplotlib.pyplot as plt

plt.figure(figsize=(8, 6))
# plt.scatter(original_x, original_y, s=1, alpha=0.2, label="Original Scans")
# plt.scatter(aligned_x, aligned_y, s=1, alpha=0.2, label="ICP Scans")
plt.plot(merged_x, merged_y, color='red', linewidth=2, label="Merged Signal (Average)")
plt.plot(merged_x_a, merged_y_a, color='blue', linewidth=2, label="Merged ICP Signal (Average)")

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Merged Sonar Scans (X-Y Projection)")
plt.legend()
plt.grid()
plt.show()




# # plot 5
# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection="3d")

# for scan in aligned_scans:
#     ax.scatter(-scan[:, 0], -scan[:, 1], scan[:, 2], s=2)

# ax.set_xlabel("X (m)")
# ax.set_ylabel("Y (m)")
# ax.set_zlabel("Z (m)")
# ax.set_title("Aligned Sonar Point Clouds (ICP)")
# # ax.set_xlim([-100,100])
# # ax.set_ylim([-100,100])
# ax.set_xlim([-3,120])
# ax.set_ylim([-5,-2])

# ax.set_zlim([-5,5])


# # Show plot
plt.show()



