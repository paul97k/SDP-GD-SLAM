import rosbag


file_name = "data_bags/my_rosbag_2025-02-05-14-23-38.bag"

with rosbag.Bag(file_name, "r") as bag:
    topic_info = bag.get_type_and_topic_info().topics

    print("Topics and Message Types in the Bag File:")
    for topic, info in topic_info.items():
            print(" - {}: {}".format(topic, info)) 
