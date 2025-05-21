#! /home/paul/python2_env/bin/python
import rosbag
import pandas as pd
import numpy as np
import json
import h5py
import sensor_msgs.point_cloud2 as pc2
import sensor_msgs.msg

# Define input bag file
file_name = "data_bags/my_rosbag_2025-02-05-14-23-38.bag"

# Define output files
csv_output = "ros_data.csv"
json_output = "ros_data.json"
h5_output = "ros_data.h5"

# Topics you want to process
topics = {
    "/sonar/PC2": "sensor_msgs/PointCloud2",
    "/tf": "tf2_msgs/TFMessage",
    "/odom": "nav_msgs/Odometry",
    "/desistek_saga/thrusters/0/thrust/": "uuv_gazebo_ros_plugins_msgs/FloatStamped",
    "/desistek_saga/thrusters/2/input": "uuv_gazebo_ros_plugins_msgs/FloatStamped",
    "/desistek_saga/thrusters/1/input": "uuv_gazebo_ros_plugins_msgs/FloatStamped",
    "/tf_static": "tf2_msgs/TFMessage",
    "/desistek_saga/ground_truth_to_tf_desistek_saga/pose": "geometry_msgs/PoseStamped",
    "/desistek_saga/dvl_sonar0": "sensor_msgs/Range",
    "/desistek_saga/dvl_sonar1": "sensor_msgs/Range",
    "/desistek_saga/dvl_sonar2": "sensor_msgs/Range",
    "/desistek_saga/dvl_sonar3": "sensor_msgs/Range",
    "/desistek_saga/sonar": "sensor_msgs/LaserScan",
    "/sonar/LS": "sensor_msgs/LaserScan",
    "/desistek_saga/dvl": "uuv_sensor_ros_plugins_msgs/DVL",
    "/desistek_saga/thrusters/0/input": "uuv_gazebo_ros_plugins_msgs/FloatStamped",
    "/sonar/PC": "sensor_msgs/PointCloud",
    "/desistek_saga/thrusters/2/thrust/": "uuv_gazebo_ros_plugins_msgs/FloatStamped",
    "/desistek_saga/cmd_vel": "geometry_msgs/Twist",
    "/desistek_saga/ground_truth_to_tf_desistek_saga/euler": "geometry_msgs/Vector3Stamped",
    "/desistek_saga/pose_gt": "nav_msgs/Odometry",
    "/desistek_saga/thrusters/1/thrust/": "uuv_gazebo_ros_plugins_msgs/FloatStamped",
    "/desistek_saga/imu": "sensor_msgs/Imu"
}

# Dictionary to store all extracted data
data_store = {}

# Open the bag file
with rosbag.Bag(file_name, "r") as bag:
    for topic, msg, t in bag.read_messages():
        timestamp = t.to_sec()

        if topic not in topics:
            continue  # Skip topics not in the list




        # Handling PointCloud2
        if topic.startswith("/sonar/PC2"):
            # print("Message Type:", type(msg))  # Debug the type of the message

            # Ensure the message is of type PointCloud2
            if 'PointCloud2' in msg._type:
                points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
                data_store.setdefault(topic, []).append({"timestamp": timestamp, "points": points})
            else:
                print("Message Type:", type(msg)) 
                print("Skipping message from topic {} - incorrect type: {}".format(topic, type(msg)))


        # Handling PointCloud (sensor_msgs/PointCloud)
        elif topic == "/sonar/PC":

            # Check if it's PointCloud (you can process it similarly)
            if 'PointCloud' in msg._type:
                points = [(p.x, p.y, p.z) for p in msg.points]  # Manually extracting points
                data_store.setdefault(topic, []).append({"timestamp": timestamp, "points": points})
            else:
                print("Message Type:", type(msg))  # Debug the type of the message

                print("Skipping message from topic {} - incorrect type: {}".format(topic, type(msg)))




        elif topic.startswith("/tf"):
            transforms = [{"frame_id": tf.child_frame_id, "transform": tf.transform} for tf in msg.transforms]
            data_store.setdefault(topic, []).append({"timestamp": timestamp, "transforms": transforms})

        elif topic.startswith("/odom") or topic.startswith("/desistek_saga/pose_gt"):
            odom_data = {
                "timestamp": timestamp,
                "position": [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                "orientation": [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            }
            data_store.setdefault(topic, []).append(odom_data)

        elif topic.startswith("/desistek_saga/thrusters"):
            thrust_value = msg.data
            data_store.setdefault(topic, []).append({"timestamp": timestamp, "thrust": thrust_value})

        elif topic == "/desistek_saga/dvl":
            dvl_data = {
                "timestamp": timestamp,
                "velocity": [msg.velocity.x, msg.velocity.y, msg.velocity.z],
                "altitude": msg.altitude
            }
            # print("hell")
            data_store.setdefault(topic, []).append(dvl_data)

        elif topic.startswith("/desistek_saga/imu"):
            # print(msg)
            imu_data = {
                "timestamp": timestamp,
                "orientation": [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                "angular_velocity": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "linear_acceleration": [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
            }
            data_store.setdefault(topic, []).append(imu_data)

        elif topic.startswith("/desistek_saga/sonar") or topic.startswith("/sonar/LS"):
            ranges = list(msg.ranges)
            data_store.setdefault(topic, []).append({"timestamp": timestamp, "ranges": ranges})

# # Convert data to DataFrame (for CSV export)
# df_list = []
# for topic, messages in data_store.items():
#     for msg in messages:
#         msg["topic"] = topic  # Add topic as a column
#         df_list.append(msg)

# df = pd.DataFrame(df_list)

# # Save to CSV
# df.to_csv(csv_output, index=False)
# print("Saved CSV:", csv_output)

# # Save to JSON
# # with open(json_output, "w") as json_file:
# #     json.dump(data_store, json_file, indent=4)
# # print("Saved JSON:", json_output)

# # Save to HDF5 (for large datasets)
# with h5py.File(h5_output, "w") as h5_file:
#     for topic, messages in data_store.items():
#         h5_file.create_dataset(topic, data=np.array(messages, dtype="S"))
# print("Saved HDF5:", h5_output)


# Create a dictionary to hold the data arrays
npz_data = {}

# Convert data to numpy arrays and store them in npz_data
for topic, messages in data_store.items():
    # For each topic, convert the list of messages into a numpy array
    npz_data[topic] = np.array(messages, dtype="object")  # Use 'object' dtype for heterogeneous data

# Save to NPZ file
np.savez_compressed("ros_data.npz", **npz_data)
print("Saved NPZ:", "ros_data.npz")