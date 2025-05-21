import numpy as np

# Load the .npz file with pickle enabled
data = np.load('/home/paul/colcon_ws/src/orca4/orca_bringup/scripts/ros_data.npz', allow_pickle=True)

# Print available keys
print("Keys in .npz file:", data.files)

# Access the specific array
array_name = 'desistek_saga/sonar'  # Replace with actual key from data.files
array_data = data[array_name]

print("hell")