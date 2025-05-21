import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# Load the saved .npz file
loaded_data = np.load("orca4/orca_bringup/scripts/data npz/sonar_scan_data1.npz")

angles = loaded_data["angles"]
ranges = loaded_data["ranges"]
timestamps = loaded_data["timestamps"]
plt.figure(figsize=(8, 6))


# Iterate over consecutive scans
skip_i = 2
for i in range(1, len(timestamps) - 1- skip_i, skip_i):
    # Convert both scans to Cartesian coordinates
    plt.clf()  # Clears the previous plot

    x1 = ranges[i] * np.sin(angles[i])
    z1 = ranges[i] * np.cos(angles[i])
    scan1 = np.vstack((x1, z1)).T  # (N, 2) shape

    x2 = ranges[i + 1+ skip_i] * np.sin(angles[i + 1+ skip_i])
    z2 = ranges[i + 1 +skip_i] * np.cos(angles[i + 1+ skip_i])
    scan2 = np.vstack((x2, z2)).T  # (N, 2) shape

    scan1 = np.nan_to_num(scan1, nan=0.0, posinf=0.0, neginf=0.0)
    scan2 = np.nan_to_num(scan2, nan=0.0, posinf=0.0, neginf=0.0)


    # Create a KDTree for the first scan
    tree = KDTree(scan1)

    # Find nearest neighbors in the second scan
    distances, indices = tree.query(scan2, k=1)  # k=1 for nearest neighbor

    # Plot matching points
    plt.scatter(scan1[:, 0], scan1[:, 1], c='blue', label="Scan t")
    plt.scatter(scan2[:, 0], scan2[:, 1], c='red', label="Scan t+1")

    # Draw matches
    for idx, matched_idx in enumerate(indices):
        plt.plot([scan1[matched_idx, 0], scan2[idx, 0]], 
                 [scan1[matched_idx, 1], scan2[idx, 1]], 
                 'g-', alpha=0.5)

    plt.xlabel("X (m)")
    plt.ylabel("Z (m)")
    plt.title(f"Feature Matching between Scans {i} and {i+1}")
    plt.legend()
    plt.grid()
    plt.xlim([-4,4])
    plt.ylim([2,5])

    plt.pause(0.1)  # Pause to view the plot
plt.show()
