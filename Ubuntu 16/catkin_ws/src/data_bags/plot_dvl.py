import rosbag
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Range
from uuv_sensor_ros_plugins_msgs.msg import DVL
import rosbag
import numpy as np

def process_dvl_sonar4(bag_file, sonar_topics):
    """
    Extracts sonar range values and timestamps for each sonar topic
    and returns them as NumPy arrays.

    Parameters:
    - bag_file: path to the .bag file
    - sonar_topics: list of 4 sonar topic names (in correct order)

    Returns:
    - range_0, range_1, range_2, range_3: np arrays of ranges
    - time_0, time_1, time_2, time_3: np arrays of timestamps
    """
    # Initialize storage
    sonar_ranges = {topic: [] for topic in sonar_topics}
    sonar_timestamps = {topic: [] for topic in sonar_topics}

    # Read bag file
    with rosbag.Bag(bag_file, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=sonar_topics):
            sonar_ranges[topic].append(msg.range)
            sonar_timestamps[topic].append(t.to_sec())

    # Convert to NumPy arrays
    ranges_np = {topic: np.array(sonar_ranges[topic]) for topic in sonar_topics}
    times_np = {topic: np.array(sonar_timestamps[topic]) for topic in sonar_topics}

    # Extract data
    range_0 = ranges_np[sonar_topics[0]]
    range_1 = ranges_np[sonar_topics[1]]
    range_2 = ranges_np[sonar_topics[2]]
    range_3 = ranges_np[sonar_topics[3]]

    time_0 = times_np[sonar_topics[0]]
    time_1 = times_np[sonar_topics[1]]
    time_2 = times_np[sonar_topics[2]]
    time_3 = times_np[sonar_topics[3]]

    return (
        range_0, range_1, range_2, range_3,
        time_0, time_1, time_2, time_3
    )


# Define the bag file path
file_name = "data_bags/SR3.bag"
file_name = "data_bags/S0.22_D10_R0.5_D6.bag"

# Topics to extract
sonar_topics = [
    "/desistek_saga/dvl_sonar0",
    "/desistek_saga/dvl_sonar1",
    "/desistek_saga/dvl_sonar2",
    "/desistek_saga/dvl_sonar3",
]
dvl_topic = "/desistek_saga/dvl"
thruster_topics = [
    "/desistek_saga/thrusters/0/thrust/",
    "/desistek_saga/thrusters/1/thrust/",
    "/desistek_saga/thrusters/2/thrust/",
    "/desistek_saga/thrusters/0/input",
    "/desistek_saga/thrusters/1/input",
    "/desistek_saga/thrusters/2/input",
]

thruster_input_topics = [
    "/desistek_saga/thrusters/0/input",
    "/desistek_saga/thrusters/1/input",
    "/desistek_saga/thrusters/2/input",
]
thruster_output_topics = [
    "/desistek_saga/thrusters/0/thrust/",
    "/desistek_saga/thrusters/1/thrust/",
    "/desistek_saga/thrusters/2/thrust/",
]

# Data storage
time_stamps = []
sonar_ranges = {topic: [] for topic in sonar_topics}
dvl_data = {"vx": [], "vy": [], "vz": []}
thruster_data = {topic: [] for topic in thruster_topics}
time_stamps_per_topic = {topic: [] for topic in sonar_topics + [dvl_topic] + thruster_topics}

# Store timestamps for DVL separately
dvl_time_stamps = []



# Read the bag file
with rosbag.Bag(file_name, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=sonar_topics + [dvl_topic] + thruster_topics):
        time_stamps.append(t.to_sec())


        time_stamps_per_topic[topic].append(t.to_sec())

        if topic in sonar_topics:
            sonar_ranges[topic].append(msg.range)  # Store sonar range data

        if topic == dvl_topic:
            dvl_time_stamps.append(t.to_sec())  # Store DVL timestamps
            # print(msg)
            dvl_data["vx"].append(msg.velocity.x)
            dvl_data["vy"].append(msg.velocity.y)
            dvl_data["vz"].append(msg.velocity.z)




        if topic in thruster_topics:
            thruster_data[topic].append(msg.data)
            
for topic, times in time_stamps_per_topic.items():
    if len(times) > 1:
        duration = times[-1] - times[0]
        frequency = len(times) / duration if duration > 0 else 0
        print("Topic: {} | Frequency: {:.2f} Hz".format(topic, frequency))
        print(duration)


    else:
        print("Topic: {} | Not enough data to compute frequency.".format(topic))
            

# Convert timestamps to numpy array
time_stamps = np.array(time_stamps)

# Plot sonar range readings
# plt.figure(figsize=(10, 5))
# for topic, data in sonar_ranges.items():
#     plt.plot(time_stamps[:len(data)], data, label=topic)
# plt.xlabel("Time (s)")
# plt.ylabel("Sonar Range (m)")
# plt.legend()
# plt.title("DVL Sonar Readings")
# plt.grid()

# # Plot DVL velocities
# plt.figure(figsize=(10, 5))
# plt.plot(dvl_time_stamps, dvl_data["vx"], label="Vx")
# plt.plot(dvl_time_stamps, dvl_data["vy"], label="Vy")
# plt.plot(dvl_time_stamps, dvl_data["vz"], label="Vz")
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (m/s)")
# plt.legend()
# plt.title("DVL Velocities")
# plt.grid()
# # Plot Thruster Inputs
# plt.figure(figsize=(10, 5))
# for topic in thruster_input_topics:
#     if topic in thruster_data:
#         plt.plot(time_stamps_per_topic[topic], thruster_data[topic], label=topic)

# plt.xlabel("Time (s)")
# plt.ylabel("Thruster Inputs")
# plt.legend()
# plt.title("Thruster Inputs Over Time")
# plt.grid()

# # Plot Thruster Outputs (Thrust)
# plt.figure(figsize=(10, 5))
# for topic in thruster_output_topics:
#     if topic in thruster_data:
#         plt.plot(time_stamps_per_topic[topic], thruster_data[topic], label=topic)

# plt.xlabel("Time (s)")
# plt.ylabel("Thrust Output")
# plt.legend()
# plt.title("Thruster Outputs Over Time")
# plt.grid()



# # Convert lists to NumPy arrays
# vx_np = np.array(dvl_data["vx"])
# vy_np = np.array(dvl_data["vy"])
# vz_np = np.array(dvl_data["vz"])
# dvl_time_np = np.array(dvl_time_stamps) 
# # Save to an `.npz` file
# output_file = "vdl_data.npz"
# np.savez(output_file, vx=vx_np, vy=vy_np, vz=vz_np, time_stamps=dvl_time_np)


ranges_0, ranges_1, ranges_2, ranges_3, t_0, t_1, t_2, t_3 = process_dvl_sonar4(file_name, sonar_topics)



ranges_list = [ranges_0, ranges_1, ranges_2, ranges_3]
times_list = [t_0, t_1, t_2, t_3]


for i, topic in enumerate(sonar_topics):
    ranges = ranges_list[i]
    times = times_list[i]

    plt.figure()
    plt.plot(times, ranges)
    plt.title("Sonar Range Data - {}".format(topic))
    plt.xlabel("Time [s]")
    plt.ylabel("Range [m]")
    plt.grid(True)
    plt.tight_layout()



plt.show()


