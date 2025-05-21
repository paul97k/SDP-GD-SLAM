import rosbag
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf.transformations as tf_trans

class DataProcessor:
    def __init__(self, bag_file):

        self.bag_file = bag_file
        self.topicSonarPoint2Cloud = "/sonar/PC2"
        self.topicSonarLaserscan = "/desistek_saga/sonar"
        self.topicGroundTruthPose = "/desistek_saga/ground_truth_to_tf_desistek_saga/pose"
        self.sonar_topics = [
            "/desistek_saga/dvl_sonar0",
            "/desistek_saga/dvl_sonar1",
            "/desistek_saga/dvl_sonar2",
            "/desistek_saga/dvl_sonar3",
        ]
        self.dvl_topic = "/desistek_saga/dvl"
        self.thruster_topics = [
            "/desistek_saga/thrusters/0/thrust/",
            "/desistek_saga/thrusters/1/thrust/",
            "/desistek_saga/thrusters/2/thrust/",
            "/desistek_saga/thrusters/0/input",
            "/desistek_saga/thrusters/1/input",
            "/desistek_saga/thrusters/2/input",
        ]
        self.imu_topic = "/desistek_saga/imu",
        self.cmd_vel_topic = "/desistek_saga/cmd_vel"
        

        self.topicGroundTruthEuler = "/desistek_saga/ground_truth_to_tf_desistek_saga/euler"
        self.hyrdro_vel_topic = "/hydrodynamics/current_velocity"


    def process_SonarPoint2Cloud(self, time_interval=2):
        """
        Extracts and processes PointCloud2 data from the ROS bag file.
        :param time_interval: Time interval (seconds) between data updates
        """
        file_name = self.bag_file
        topic = self.topicSonarPoint2Cloud
        all_x = []
        all_y = []
        all_z = []
        time_stamps = []

        with rosbag.Bag(file_name, "r") as bag:
            prev_time = None
            for _, msg, t in bag.read_messages(topics=[topic]):
                curr_time = t.to_sec()
                
                if prev_time is None or (curr_time - prev_time) >= time_interval:
                    prev_time = curr_time
                    new_x, new_y, new_z = [], [], []
                    
                    for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                        x = -point[1]
                        y = point[0]
                        new_x.append(x)
                        new_y.append(y)
                        new_z.append(point[2])

                    time_stamps.append(t.to_sec())  # Store DVL timestamps
                    
                    all_x.extend(new_x)
                    all_y.extend(new_y)
                    all_z.extend(new_z)
        return all_x, all_y, all_z, time_stamps
        
    def process_SonarLaserscan(self):
        file_name = self.bag_file
        topic = self.topicSonarLaserscan
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
        return angles_np, ranges_np, timestamps_np


    def process_GroundTruthPose(self):

        file_name = self.bag_file
        topic = self.topicGroundTruthPose
        # Lists to store extracted data
        timestamps = []
        x_positions = []
        y_positions = []
        z_positions = []

        # Open the bag file and extract data
        with rosbag.Bag(file_name, "r") as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                # Extract timestamp
                timestamps.append(t.to_sec())

                # Extract position
                x_positions.append(msg.pose.position.x)

                
                y_positions.append(msg.pose.position.y)
                z_positions.append(msg.pose.position.z)

        # Convert to NumPy arrays for easier plotting
        timestamps = np.array(timestamps)
        x_positions = np.array(x_positions)
        y_positions = np.array(y_positions)
        z_positions = np.array(z_positions)
        return x_positions, y_positions, z_positions, timestamps
        


    def process_GroundTruthAngle(self):

        file_name = self.bag_file
        topic = self.topicGroundTruthEuler
        # Lists to store extracted data
        # Lists to store extracted Euler angles
        timestamps_euler = []
        roll_angles = []
        pitch_angles = []
        yaw_angles = []
        # Open the bag file and extract data
        with rosbag.Bag(file_name, "r") as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                timestamps_euler.append(t.to_sec())
                roll_angles.append(msg.vector.x)   # Roll
                pitch_angles.append(msg.vector.y)  # Pitch
                yaw_angles.append(msg.vector.z)    # Yaw

        timestamps_euler = np.array(timestamps_euler)
        roll_angles = np.array(roll_angles)
        pitch_angles = np.array(pitch_angles)
        yaw_angles = np.array(yaw_angles)
        return roll_angles, pitch_angles, yaw_angles, timestamps_euler


    # def process_dvl_sonar4(self):

    #     file_name = self.bag_file
    #     sonar_topics = self.sonar_topics
    #     sonar_ranges = {topic: [] for topic in sonar_topics}
    #     time_stamps = []

    #     with rosbag.Bag(file_name, "r") as bag:
    #         for topic, msg, t in bag.read_messages(topics=sonar_topics):
    #             time_stamps.append(t.to_sec())  # Store DVL timestamps
    #             sonar_ranges[topic].append(msg.range)  # Store sonar range data
    #     return sonar_ranges, time_stamps

    def process_dvl_sonar4(self):
        """
        Extracts sonar range values and timestamps for each sonar topic
        and returns them as NumPy arrays.
        """
        file_name = self.bag_file
        sonar_topics = self.sonar_topics  # e.g., ["/desistek_saga/dvl_sonar0", ..., "sonar3"]

        # Initialize storage
        sonar_ranges = {topic: [] for topic in sonar_topics}
        sonar_timestamps = {topic: [] for topic in sonar_topics}

        # Read bag file
        with rosbag.Bag(file_name, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=sonar_topics):
                sonar_ranges[topic].append(msg.range)
                sonar_timestamps[topic].append(t.to_sec())

        # Convert lists to NumPy arrays
        ranges_np = {topic: np.array(sonar_ranges[topic]) for topic in sonar_topics}
        times_np = {topic: np.array(sonar_timestamps[topic]) for topic in sonar_topics}

        # Extract data per topic
        range_0 = ranges_np["/desistek_saga/dvl_sonar0"]
        range_1 = ranges_np["/desistek_saga/dvl_sonar1"]
        range_2 = ranges_np["/desistek_saga/dvl_sonar2"]
        range_3 = ranges_np["/desistek_saga/dvl_sonar3"]

        time_0 = times_np["/desistek_saga/dvl_sonar0"]
        time_1 = times_np["/desistek_saga/dvl_sonar1"]
        time_2 = times_np["/desistek_saga/dvl_sonar2"]
        time_3 = times_np["/desistek_saga/dvl_sonar3"]

        return (
            range_0, range_1, range_2, range_3,
            time_0, time_1, time_2, time_3
        )



    def process_dvl(self):

        file_name = self.bag_file
        topic = self.dvl_topic
        dvl_data = {"vx": [], "vy": [], "vz": []}
        dvl_time_stamps = []

        with rosbag.Bag(file_name, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=topic):
                dvl_time_stamps.append(t.to_sec())  # Store DVL timestamps
                dvl_data["vx"].append(msg.velocity.x)
                dvl_data["vy"].append(msg.velocity.y)
                dvl_data["vz"].append(msg.velocity.z)
        vx = np.array(dvl_data["vx"])
        vy = np.array(dvl_data["vy"])
        vz = np.array(dvl_data["vz"])                
        dvl_time_stamps = np.array(dvl_time_stamps)     

        return vx, vy, vz, dvl_time_stamps


           
    import rosbag
    import numpy as np

    def process_thruster_topics(self):
        """
        Extracts thrust/input values and timestamps for each thruster topic
        and returns them as NumPy arrays.
        """
        file_name = self.bag_file
        thruster_topics = self.thruster_topics

        # Initialize storage for signals and timestamps
        thruster_signals = {topic: [] for topic in thruster_topics}
        thruster_timestamps = {topic: [] for topic in thruster_topics}

        # Read bag file
        with rosbag.Bag(file_name, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=thruster_topics):
                thruster_timestamps[topic].append(t.to_sec())  # Convert ROS time to seconds
                thruster_signals[topic].append(msg.data)  # Store thrust/input data

        # Convert lists to NumPy arrays
        thruster_data_np = {topic: np.array(thruster_signals[topic]) for topic in thruster_topics}
        thruster_time_np = {topic: np.array(thruster_timestamps[topic]) for topic in thruster_topics}

        # Extract specific arrays
        thruster0_out = thruster_data_np["/desistek_saga/thrusters/0/thrust/"]
        thruster1_out = thruster_data_np["/desistek_saga/thrusters/1/thrust/"]
        thruster2_out = thruster_data_np["/desistek_saga/thrusters/2/thrust/"]
        thruster0_in = thruster_data_np["/desistek_saga/thrusters/0/input"]
        thruster1_in = thruster_data_np["/desistek_saga/thrusters/1/input"]
        thruster2_in = thruster_data_np["/desistek_saga/thrusters/2/input"]

        # Extract timestamps
        thruster0_out_t = thruster_time_np["/desistek_saga/thrusters/0/thrust/"]
        thruster1_out_t = thruster_time_np["/desistek_saga/thrusters/1/thrust/"]
        thruster2_out_t = thruster_time_np["/desistek_saga/thrusters/2/thrust/"]
        thruster0_in_t = thruster_time_np["/desistek_saga/thrusters/0/input"]
        thruster1_in_t = thruster_time_np["/desistek_saga/thrusters/1/input"]
        thruster2_in_t = thruster_time_np["/desistek_saga/thrusters/2/input"]

        return (
            thruster0_out, thruster1_out, thruster2_out, 
            thruster0_in, thruster1_in, thruster2_in, 
            thruster0_out_t, thruster1_out_t, thruster2_out_t, 
            thruster0_in_t, thruster1_in_t, thruster2_in_t
        )

    def process_IMU_topics(self):
        # Lists to store extracted data
        timestamps = []
        rolls, pitches, yaws = [], [], []
        angular_velocities = []
        linear_accelerations = []
        file_name = self.bag_file


        # Read the bag file
        with rosbag.Bag(file_name, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=self.imu_topic):
                timestamps.append(t.to_sec())  # Convert ROS time to seconds
                
                # Convert quaternion to Euler angles
                quaternion = (
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w
                )
                euler = tf_trans.euler_from_quaternion(quaternion)  # Roll, pitch, yaw
                rolls.append(euler[0])
                pitches.append(euler[1])
                yaws.append(euler[2])
                
                # Store angular velocity
                angular_velocities.append([
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z
                ])
                
                # Store linear acceleration
                linear_accelerations.append([
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z
                ])

        # Convert lists to numpy arrays
        timestamps = np.array(timestamps)
        angular_velocities = np.array(angular_velocities)
        linear_accelerations = np.array(linear_accelerations)
        return timestamps, rolls, pitches, yaws, angular_velocities, linear_accelerations
    



    def process_cmd_topics(self):
        # Lists to store extracted data
        cmd_vel_data = {"linear_x": [], "linear_y": [], "linear_z": [], "angular_x": [], "angular_y": [], "angular_z": []}
        cmd_vel_time_stamps = []

        # Read the bag file
        with rosbag.Bag(file_name, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=self.cmd_vel_topic):
                cmd_vel_time_stamps.append(t.to_sec())

                # Extract linear velocities
                cmd_vel_data["linear_x"].append(msg.linear.x)
                cmd_vel_data["linear_y"].append(msg.linear.y)
                cmd_vel_data["linear_z"].append(msg.linear.z)

                # Extract angular velocities
                cmd_vel_data["angular_x"].append(msg.angular.x)
                cmd_vel_data["angular_y"].append(msg.angular.y)
                cmd_vel_data["angular_z"].append(msg.angular.z)


        timestamps = np.array(cmd_vel_time_stamps)
        cmd_linear_x = np.array(cmd_vel_data["linear_x"])
        cmd_linear_y = np.array(cmd_vel_data["linear_y"])
        cmd_linear_z = np.array(cmd_vel_data["linear_z"])
        cmd_angular_x = np.array(cmd_vel_data["angular_x"])
        cmd_angular_y = np.array(cmd_vel_data["angular_y"])
        cmd_angular_z = np.array(cmd_vel_data["angular_z"])
        return timestamps, cmd_linear_x ,cmd_linear_y ,cmd_linear_z ,cmd_angular_x ,cmd_angular_y ,cmd_angular_z


    
    def print_rosbag_topics(self):
        """
        Prints all topics available in a ROS bag file.
        :param bag_file: Path to the ROS bag file.
        """
        file_name = self.bag_file
        with rosbag.Bag(file_name, "r") as bag:
            topics = bag.get_type_and_topic_info()[1].keys()
            print("Topics in the ROS bag file:")
            for topic in topics:
                print(topic)


    def process_hyrdro_vel(self):

        file_name = self.bag_file
        topic = self.hyrdro_vel_topic
        hyrdro_vel_data = {"linear_x": [], "linear_y": [], "linear_z": [], "angular_x": [], "angular_y": [], "angular_z": []}
        hyrdro_vel_time_stamps = []

        with rosbag.Bag(file_name, "r") as bag:
            for topic, msg, t in bag.read_messages(topics=topic):
                hyrdro_vel_time_stamps.append(t.to_sec())  # Store DVL timestamps
                # Extract linear velocities
                hyrdro_vel_data["linear_x"].append(msg.twist.linear.x)
                hyrdro_vel_data["linear_y"].append(msg.twist.linear.y)
                hyrdro_vel_data["linear_z"].append(msg.twist.linear.z)

                # Extract angular velocities
                hyrdro_vel_data["angular_x"].append(msg.twist.angular.x)
                hyrdro_vel_data["angular_y"].append(msg.twist.angular.y)
                hyrdro_vel_data["angular_z"].append(msg.twist.angular.z)


        timestamps = np.array(hyrdro_vel_time_stamps)
        hyrdro_linear_x = np.array(hyrdro_vel_data["linear_x"])
        hyrdro_linear_y = np.array(hyrdro_vel_data["linear_y"])
        hyrdro_linear_z = np.array(hyrdro_vel_data["linear_z"])
        hyrdro_angular_x = np.array(hyrdro_vel_data["angular_x"])
        hyrdro_angular_y = np.array(hyrdro_vel_data["angular_y"])
        hyrdro_angular_z = np.array(hyrdro_vel_data["angular_z"])
        return timestamps, hyrdro_linear_x ,hyrdro_linear_y ,hyrdro_linear_z ,hyrdro_angular_x ,hyrdro_angular_y ,hyrdro_angular_z   

        # return vx, vy, vz, hyrdo_vel_time_stamps
import scipy.io
# Example usage:
if __name__ == "__main__":

    processFilenames = ["SR0.01", "SR0.1", "SR0.2","SR0.5","SR0.7", "SR1.0", "SR1.2",
                        "u1_0.5", "u1_0.05","u6_0.5", "u7_0.5", 
                        "SC11", "SC12", "SC13", 
                        "SC22", "SC23", 
                        "NC0", "NC01", "NC02", "NC03", "NC04", "NC05", 
                        "C01", "C02", "C03", "C04", "C05",
                        ]

    processFilenames = ["chirp0.001-1x", "chirp0.0001-1xyz", "chirp0.0001-1xyz+1"]
    processFilenames = ["S0.22_D10_R0.5_D0_withoutAnything", "S0.22_D10_R0.5_D0",  
    "S0.22_D10_R0.5_D0", 
    "S0.22_D10_R0.5_D1", 
    "S0.22_D10_R0.5_D2", 
    "S0.22_D10_R0.5_D3", 
    "S0.22_D10_R0.5_D4", 
    # "S0.22_D10_R0.5_D5", 
    "S0.22_D10_R0.5_D6", 
    "S0.22_D10_R0.5_D8", 
    "S0.22_D10_R0.5_D10",
    "S0.22_D10_R0.5_D12"]

    processFilenames = [
    "S0.22_D10_R0.5_D0_Test1",    "S0.22_D10_R0.5_D0_Test2",
    "S0.22_D10_R0.5_D1_Test1",    "S0.22_D10_R0.5_D1_Test2",
    "S0.22_D10_R0.5_D2_Test1",    "S0.22_D10_R0.5_D2_Test2",
    "S0.22_D10_R0.5_D3_Test1",    "S0.22_D10_R0.5_D3_Test2",
    "S0.22_D10_R0.5_D4_Test1",    "S0.22_D10_R0.5_D4_Test2",
    "S0.22_D10_R0.5_D6_Test1",    "S0.22_D10_R0.5_D6_Test2",
    "S0.22_D10_R0.5_D8_Test1",    "S0.22_D10_R0.5_D8_Test2",
    "S0.22_D10_R0.5_D10_Test1",    "S0.22_D10_R0.5_D10_Test2",
    "S0.22_D10_R0.5_D12_Test1",    "S0.22_D10_R0.5_D12_Test2"]


    processFilenames = [
    "S0.22_D10_R0.5_D0_Noise1","S0.22_D10_R0.5_D0_Noise2",
    "S0.22_D10_R0.5_D6_Noise1","S0.22_D10_R0.5_D6_Noise2",
    "S0.22_D10_R0.5_D12_Noise1","S0.22_D10_R0.5_D12_Noise2"]
    
    ned_velocities_all = {}

    
    for processFilename in processFilenames:

        file_name = "data_bags/" + processFilename + ".bag"
        saved_filename = processFilename + ".mat"
            
        processor = DataProcessor(file_name)

        # processor.print_rosbag_topics()


        angles_np, ranges_np, timestamps_sonar = processor.process_SonarLaserscan()

        # Process DVL data
        vx, vy, vz, timestamps_dvl = processor.process_dvl()
        [range_0, range_1, range_2, range_3,
            time_0, time_1, time_2, time_3 ]= processor.process_dvl_sonar4()

        [thruster0_out, thruster1_out, 
        thruster2_out, thruster0_in, 
        thruster1_in, thruster2_in,
        thruster0_out_t,
        thruster1_out_t,
        thruster2_out_t,
        thruster0_in_t,
        thruster1_in_t,
        thruster2_in_t,
        ] = processor.process_thruster_topics()


        
        [timestamps_imu, 
        rolls, pitches, yaws, 
        angular_velocities, linear_accelerations] = processor.process_IMU_topics()


        [x_positionsGT, 
        y_positionsGT, 
        z_positionsGT, 
        timestamps_poseGT] = processor.process_GroundTruthPose()
        
        [roll_anglesGT, 
        pitch_anglesGT, 
        yaw_anglesGT, 
        timestamps_eulerGT] = processor.process_GroundTruthAngle()

        [cmd_timestamps, 
        cmd_linear_x ,
        cmd_linear_y ,
        cmd_linear_z ,
        cmd_angular_x ,
        cmd_angular_y ,
        cmd_angular_z] = processor.process_cmd_topics()


        [hyrdro_timestamps, 
        hyrdro_linear_x ,
        hyrdro_linear_y ,
        hyrdro_linear_z ,
        hyrdro_angular_x ,
        hyrdro_angular_y ,
        hyrdro_angular_z ] = processor.process_hyrdro_vel()


        # np.savez("processed_data.npz", 
        #         angles=angles_np, 
        #         ranges=ranges_np, 
        #         timestamps_sonar=timestamps_sonar,

        #         vx=vx, 
        #         vy=vy, 
        #         vz=vz, 
        #         timestamps_dvl=timestamps_dvl,

        #         thruster0_out=thruster0_out, 
        #         thruster1_out=thruster1_out, 
        #         thruster2_out=thruster2_out, 
        #         thruster0_in=thruster0_in, 
        #         thruster1_in=thruster1_in, 
        #         thruster2_in=thruster2_in,
        #         thruster0_out_t = thruster0_out_t,
        #         thruster1_out_t = thruster1_out_t,
        #         thruster2_out_t = thruster2_out_t,
        #         thruster0_in_t = thruster0_in_t,
        #         thruster1_in_t = thruster1_in_t,
        #         thruster2_in_t = thruster2_in_t,

        #         rolls = rolls,
        #         pitches = pitches,
        #         yaws = yaws,
        #         angular_velocities = angular_velocities,
        #         linear_accelerations = linear_accelerations,
        #         timestamps_imu = timestamps_imu,

        #         x_positionsGT = x_positionsGT, 
        #         y_positionsGT = y_positionsGT, 
        #         z_positionsGT = z_positionsGT, 
        #         timestamps_pose = timestamps_poseGT,

        #         roll_anglesGT = roll_anglesGT, 
        #         pitch_anglesGT = pitch_anglesGT, 
        #         yaw_anglesGT = yaw_anglesGT, 
        #         timestamps_eulerGT =timestamps_eulerGT,

        #         cmd_timestamps=cmd_timestamps,
        #         cmd_linear_x=cmd_linear_x,
        #         cmd_linear_y=cmd_linear_y,
        #         cmd_linear_z=cmd_linear_z,
        #         cmd_angular_x=cmd_angular_x,
        #         cmd_angular_y=cmd_angular_y,
        #         cmd_angular_z=cmd_angular_z,

        #         hyrdro_timestamps = hyrdro_timestamps,
        #         hyrdro_linear_x = hyrdro_linear_x,
        #         hyrdro_linear_y = hyrdro_linear_y,
        #         hyrdro_linear_z = hyrdro_linear_z,
        #         hyrdro_angular_x = hyrdro_angular_x,
        #         hyrdro_angular_y = hyrdro_angular_y,
        #         hyrdro_angular_z = hyrdro_angular_z,

        #         range_0 = range_0,
        #         range_1 = range_1,
        #         range_2 = range_2,
        #         range_3 = range_3,

        #         time_0 = time_0,
        #         time_1 = time_1,
        #         time_2 = time_2,
        #         time_3 = time_3,
        
            
        #         allow_pickle=True)

        data_dict = {
            "angles": angles_np,
            "ranges": ranges_np,
            "timestamps_sonar": timestamps_sonar,

            "vx": vx,
            "vy": vy,
            "vz": vz,
            "timestamps_dvl": timestamps_dvl,


            'thruster0_out': thruster0_out, 
            'thruster1_out': thruster1_out, 
            'thruster2_out': thruster2_out, 
            'thruster0_in': thruster0_in, 
            'thruster1_in': thruster1_in, 
            'thruster2_in': thruster2_in,
            'thruster0_out_t' : thruster0_out_t,
            'thruster1_out_t' : thruster1_out_t,
            'thruster2_out_t' : thruster2_out_t,
            'thruster0_in_t' : thruster0_in_t,
            'thruster1_in_t' : thruster1_in_t,
            'thruster2_in_t' : thruster2_in_t,

            'rolls' : rolls,
            'pitches' : pitches,
            'yaws' : yaws,
            'angular_velocities' : angular_velocities,
            'linear_accelerations' : linear_accelerations,

            'x_positionsGT' : x_positionsGT, 
            'y_positionsGT' : y_positionsGT, 
            'z_positionsGT' : z_positionsGT, 
            'timestamps_pose' : timestamps_poseGT, 

            'roll_anglesGT' : roll_anglesGT, 
            'pitch_anglesGT' : pitch_anglesGT, 
            'yaw_anglesGT' : yaw_anglesGT, 
            'timestamps_eulerG' : timestamps_eulerGT, 

            'timestamps_imu' : timestamps_imu,

            'cmd_timestamps' : cmd_timestamps,
            'cmd_linear_x' : cmd_linear_x,
            'cmd_linear_y' : cmd_linear_y,
            'cmd_linear_z' : cmd_linear_z,
            'cmd_angular_x' : cmd_angular_x,
            'cmd_angular_y' : cmd_angular_y,
            'cmd_angular_z' : cmd_angular_z,

            'hyrdro_timestamps' : hyrdro_timestamps,
            'hyrdro_linear_x' : hyrdro_linear_x,
            'hyrdro_linear_y' : hyrdro_linear_y,
            'hyrdro_linear_z' : hyrdro_linear_z,
            'hyrdro_angular_x' : hyrdro_angular_x,
            'hyrdro_angular_y' : hyrdro_angular_y,
            'hyrdro_angular_z' : hyrdro_angular_z,

            'range_0'  : range_0,
            'range_1'  : range_1,
            'range_2'  : range_2,
            'range_3'  : range_3,
            'time_0'  : time_0,
            'time_1'  : time_1,
            'time_2'  : time_2,
            'time_3'  : time_3

        }
        print("Data successfully saved "+saved_filename)
        
        scipy.io.savemat("data_bags/mat files/"+saved_filename, data_dict)


        # sonar_topics = [
        #     "/desistek_saga/dvl_sonar0",
        #     "/desistek_saga/dvl_sonar1",
        #     "/desistek_saga/dvl_sonar2",
        #     "/desistek_saga/dvl_sonar3",
        # ]
        # ranges_list = [range_0, range_1, range_2, range_3]
        # times_list = [time_0, time_1, time_2, time_3]
        # plt.figure()  # One figure for all lines

        # for i, topic in enumerate(sonar_topics):
        #     ranges = ranges_list[i]
        #     times = times_list[i]

        #     plt.plot(times, ranges, label=topic)

        # plt.title("Sonar Range Data (All)")
        # plt.xlabel("Time [s]")
        # plt.ylabel("Range [m]")
        # plt.grid(True)
        # plt.legend()  # Show labels for each line
        # plt.tight_layout()
        # plt.show()

#         # Compute NED velocity
#         ned_velocities = []
#         for i in range(len(timestamps_dvl)):
#             imu_idx = np.argmin(np.abs(timestamps_imu - timestamps_dvl[i]))
#             roll, pitch, yaw = rolls[imu_idx], pitches[imu_idx], yaws[imu_idx]

#             # Rotation matrix
#             R = np.array([
#                 [np.cos(yaw) * np.cos(pitch), np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll), np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)],
#                 [np.sin(yaw) * np.cos(pitch), np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll), np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)],
#                 [-np.sin(pitch), np.cos(pitch) * np.sin(roll), np.cos(pitch) * np.cos(roll)]
#             ])

#             body_velocity = np.array([vx[i], vy[i], vz[i]])
#             ned_velocity = np.dot(R, body_velocity)

#             ned_velocities.append(ned_velocity)

#         ned_velocities = np.array(ned_velocities)

#         # Store results
#         ned_velocities_all[processFilename] = {
#             "timestamps_dvl": timestamps_dvl,
#             "vx_ned": ned_velocities[:, 0],
#             "vy_ned": ned_velocities[:, 1],
#             "vz_ned": ned_velocities[:, 2],
#         }

# import matplotlib.pyplot as plt

# # Plot the velocities
# plt.figure(figsize=(12, 6))

# for processFilename in processFilenames:
#     data = ned_velocities_all[processFilename]
#     # plt.plot(data["timestamps_dvl"], data["vx_ned"], label=processFilename + " vx_ned")
#     # plt.plot(data["timestamps_dvl"], data["vy_ned"], label=processFilename + " vy_ned")
#     plt.plot(data["timestamps_dvl"], data["vz_ned"], label=processFilename + " vz_ned")

# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (m/s)")
# plt.title("NED Velocity for Different Datasets")
# plt.legend()
# plt.grid()
# plt.show()