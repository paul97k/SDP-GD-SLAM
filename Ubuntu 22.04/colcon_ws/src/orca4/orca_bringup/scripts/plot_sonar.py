



# source myenv/bin/activate
import os
import numpy as np
import open3d as o3d
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go


# Define file name
file_name = "rosbag2_2025_01_30-15_15_16"
file_name = "rosbag2_2025_01_30-15_30_40"

file_name = "my_rosbag_2025-02-05-14-23-38"

folder_path = f"/home/paul/Documents/BAGS/{file_name_ros1}"


# folder_path = f"/home/paul/colcon_ws/{file_name}"

# ROS 2 Bag Reader
storage_options = rosbag2_py.StorageOptions(uri=folder_path, storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")
reader = rosbag2_py.SequentialReader()
reader.open(storage_options, converter_options) 

# Get topics
topics_info = reader.get_all_topics_and_types()
topic_name = "/multi_beam_lidar"  # Ensure this topic exists

# Read messages
pc_all = []
i = 0
i_max = 0
while reader.has_next():
    (topic, data, t) = reader.read_next()
    if topic == topic_name:
        if i % 100 == 0:
            msg = deserialize_message(data, LaserScan)

            # Extract ranges and angles
            ranges = np.array(msg._ranges)
            
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            angles = np.arange(angle_min, angle_min + len(ranges) * angle_increment, angle_increment)

            # Convert to Cartesian coordinates
            x = ranges * np.cos(angles)
            x = x+abs(min(x))+i_max
            y = ranges * np.sin(angles)
            # z = np.arange(ranges.size)+i*ranges.size
            z = np.zeros_like(x) 
            # z = np.ones(ranges.size)*i

            # Append points
            points = np.vstack((x, y, z)).T
            pc_all.append(points)
            
            if max(x) <= 10000:
                i_max = max(x)
                # print(i_max)
        i=i+1

        
        

# Concatenate point cloud
pc_all = np.vstack(pc_all)

# Create figure
import plotly.graph_objects as go
pc_sub = pc_all
# Create a 3D scatter plot
fig = go.Figure(data=[go.Scatter3d(
    x=pc_sub[:, 0],
    y=pc_sub[:, 1],
    z=pc_sub[:, 2],
    mode='markers',
    marker=dict(
        size=2,
        color=pc_all[:, 2],  # set color to the Z coordinate
        colorscale='Viridis',  # choose a colorscale
        opacity=0.8
    )
)])

# Update plot layout
fig.update_layout(
    title="LiDAR Point Cloud",
    scene=dict(
        xaxis_title='X [meters]',
        yaxis_title='Y [meters]',
        zaxis_title='Z [meters]',
        aspectmode='manual',  # Allows manual aspect ratio adjustment
        aspectratio=dict(x=10, y=1, z=0.1)  # Make X direction longer
    )
)
fig.show()
