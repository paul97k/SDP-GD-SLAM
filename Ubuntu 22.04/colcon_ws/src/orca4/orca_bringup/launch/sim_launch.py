#!/usr/bin/env python3

"""
Launch a simulation.

Includes Gazebo, ArduSub, RViz, mavros, all ROS nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    orca_bringup_dir = get_package_share_directory('orca_bringup')
    orca_description_dir = get_package_share_directory('orca_description')

    ardusub_params_file = os.path.join(orca_bringup_dir, 'cfg', 'sub.parm')
    mavros_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_mavros_params.yaml')
    orca_params_file = os.path.join(orca_bringup_dir, 'params', 'sim_orca_params.yaml')
    rosbag2_record_qos_file = os.path.join(orca_bringup_dir, 'params', 'rosbag2_record_qos.yaml')
    rviz_file = os.path.join(orca_bringup_dir, 'cfg', 'sim_launch.rviz')
    world_file = os.path.join(orca_description_dir, 'worlds', 'sand.world')

    sim_left_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_left.ini')
    sim_right_ini = os.path.join(orca_bringup_dir, 'cfg', 'sim_right.ini')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'ardusub',
            default_value='True',
            description='Launch ArduSUB with SIM_JSON?'
        ),

        DeclareLaunchArgument(
            'bag',
            default_value='True',
            description='Bag interesting topics?',
        ),

        DeclareLaunchArgument(
            'base',
            default_value='True',
            description='Launch base controller?',
        ),

        DeclareLaunchArgument(
            'gzclient',
            default_value='True',
            description='Launch Gazebo UI?'
        ),

        DeclareLaunchArgument(
            'mavros',
            default_value='True',
            description='Launch mavros?',
        ),

        DeclareLaunchArgument(
            'nav',
            default_value='True',
            description='Launch navigation?',
        ),

        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Launch rviz?',
        ),

        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Launch SLAM?',
        ),

        # Bag useful topics
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--qos-profile-overrides-path', rosbag2_record_qos_file,
                '--include-hidden-topics',
                '/cmd_vel',
                '/mavros/local_position/pose',
                # '/mavros/rc/override',
                # '/mavros/setpoint_position/global',
                # '/mavros/state',
                # '/motion',
                # '/orb_slam2_stereo_node/status',
                '/mavros/vision_pose/pose',
                '/model/orca4/odometry',
                '/mavros/imu/data',
                '/odom',
                '/orb_slam2_stereo_node/pose',
                '/pid_z',
                '/rosout',
                '/tf',
                '/tf_static',
                '/mavros/local_position/pose',  # Add topics you want to record
                '/model/orca4/odometry',
                '/orb_slam2_stereo_node/pose',
                '/orb_slam2_stereo_node/map_points',
                '/lidar',
                '/multi_beam_lidar',
                '/model/orca4/joint/thruster1_joint/cmd_thrust',
                '/model/orca4/joint/thruster2_joint/cmd_thrust',
                '/model/orca4/joint/thruster3_joint/cmd_thrust',
                '/model/orca4/joint/thruster4_joint/cmd_thrust',
                '/model/orca4/joint/thruster5_joint/cmd_thrust',
                '/model/orca4/joint/thruster6_joint/cmd_thrust',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('bag')),
        ),

        # Launch rviz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # Launch ArduSub w/ SIM_JSON
        # -w: wipe eeprom
        # --home: start location (lat,lon,alt,yaw). Yaw is provided by Gazebo, so the start yaw value is ignored.
        # ardusub must be on the $PATH, see src/orca4/setup.bash
        ExecuteProcess(
            cmd=['ardusub', '-S', '-w', '-M', 'JSON', '--defaults', ardusub_params_file,
                 '-I0', '--home', '33.810313,-118.39386700000001,0.0,0'],
            output='screen',
            condition=IfCondition(LaunchConfiguration('ardusub')),
        ),

        # Launch Gazebo Sim
        # gz must be on the $PATH
        # libArduPilotPlugin.so must be on the GZ_SIM_SYSTEM_PLUGIN_PATH
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', world_file],
            output='screen',
            condition=IfCondition(LaunchConfiguration('gzclient')),
        ),

        # Launch Gazebo Sim server-only
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '3', '-r', '-s', world_file],
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gzclient')),
        ),

        # Get images from Gazebo Sim to ROS
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['stereo_left', 'stereo_right'],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Gazebo Sim doesn't publish camera info, so do that here
        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='left_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_left_ini,
                'camera_name': 'stereo_left',
                'frame_id': 'stereo_left_frame',
                'timer_period_ms': 50,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('/camera_info', '/stereo_left/camera_info'),
            ],
        ),

        Node(
            package='orca_base',
            executable='camera_info_publisher',
            name='right_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': 'file://' + sim_right_ini,
                'camera_name': 'stereo_right',
                'frame_id': 'stereo_right_frame',
                'timer_period_ms': 50,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('/camera_info', '/stereo_right/camera_info'),
            ],
        ),

        # Publish ground truth pose from Ignition Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/orca4/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/multi_beam_lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/pc2@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloud2'],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        

                # Bridge for /ocean_current topic
        # Bridge for /ocean_current topic
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/ocean_current@geometry_msgs/msg/Vector3@gz.msgs.Vector3d'
            ],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # # Launch the sinusoidal current publisher node
        # Node(
        #     package='sinusoidal_current',
        #     executable='sinusoidal_current',
        #     name='sinusoidal_current',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        # ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/orca4/joint/thruster1_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster2_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster3_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster4_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster5_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/orca4/joint/thruster6_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),



        # Node(
        #     package='laser_geometry',
        #     executable='laser_geometry_node',  # Adjust with actual executable that performs the conversion
        #     name='lidar_to_pointcloud',
        #     remappings=[
        #         ('/scan', '/multi_beam_lidar'),  # Topic for incoming LaserScan data
        #         ('/cloud_out', '/multi_beam_lidar_pointcloud')  # Topic for outgoing PointCloud2 data
        #     ],
        #     parameters=[
        #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
        #     ],
        #     output='screen'
        # ),
        # Node(
        #     package='pointcloud_to_laserscan',
        #     executable='laserscan_to_pointcloud_node',
        #     name='laserscan_to_pointcloud',
        #     remappings=[
        #         ('/scan', '/multi_beam_lidar'),  # Topic for incoming LaserScan data
        #         ('/cloud_out', '/multi_beam_lidar_pointcloud')  # Topic for outgoing PointCloud2 data
        #     ],
        #     parameters=[{'target_frame': 'scan', 'transform_tolerance': 0.01}]
        # ),


        # Bring up Orca and Nav2 nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(orca_bringup_dir, 'launch', 'bringup.py')),
            launch_arguments={
                'base': LaunchConfiguration('base'),
                'mavros': LaunchConfiguration('mavros'),
                'mavros_params_file': mavros_params_file,
                'nav': LaunchConfiguration('nav'),
                'orca_params_file': orca_params_file,
                'slam': LaunchConfiguration('slam'),
            }.items(),
        ),
        # Run mission_runner.py
        ExecuteProcess(
            cmd=['ros2', 'run', 'orca_bringup', 'mission_runner.py'],  # Command to run the script
            output='screen'  # Output to screen so you can see the logs in the terminal
        ),
    ])
