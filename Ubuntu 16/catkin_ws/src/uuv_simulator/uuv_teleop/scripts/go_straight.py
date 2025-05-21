#!/usr/bin/env python
from __future__ import print_function
import os
import time
import sys, termios, tty
import rospy
from geometry_msgs.msg import Twist, Vector3
import math
import subprocess
from scipy.signal import chirp


class KeyBoardVehicleTeleop:
    def __init__(self):
        # ROS Publisher
        self._output_pub = rospy.Publisher('output', Twist, queue_size=1)

        # Default message remains as twist
        self._msg_type = 'twist'
        if rospy.has_param('~type'):
            self._msg_type = rospy.get_param('~type')
            if self._msg_type not in ['twist', 'accel']:
                raise rospy.ROSException('Teleoperation output must be either twist or accel')

        rospy.loginfo("Starting ROV forward motion...")

        # Initialize constant forward movement
        cmd = Twist()
        forward_speed = 1
        cmd.linear = Vector3(forward_speed, 0, 0)  # Move forward with constant speed
        cmd.angular = Vector3(0, 0, 0)  # No rotation
        # start_time = rospy.get_time()
        # wave_freq = 0.05# Frequency: 0.1 Hz

        # wave_freq_start = 0.0001  # Start frequency in Hz
        # wave_freq_end = 1.0  # End frequency in Hz
        # T = 200.0  # Duration over which chirp happens

        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
        #     linear_x = math.sin(2 * math.pi * wave_freq* rospy.get_time() )
        #     linear_y = math.cos(2 * math.pi * wave_freq* rospy.get_time() )

        #     t = rospy.get_time()  # Get current time
        #     chrip_x = chirp(t, f0=wave_freq_start, f1=wave_freq_end, t1=T, method='linear')
        #     linear_y = math.cos(2 * math.pi * wave_freq_end * t)

            # linear_x = 0.5*linear_x+0.5 #

            cmd = Twist()
            cmd.linear = Vector3(forward_speed, 0, 0)  # Sine wave motion
            # cmd.angular = Vector3(0, 0, 0)  # No rotatio
            if rospy.get_time()>5:
                # cmd.linear = Vector3(chrip_x*0.5+1, chrip_x, chrip_x*0.5)  # MAX = 1/ms
                # cmd.linear = Vector3(0, linear_y, 0)  # MAX = 1/ms
                cmd.linear = Vector3(forward_speed, 0, 0)  # MAX = 1/ms
                cmd.angular = Vector3(0, 0, 0)  # No rotatio
            else:
                cmd.linear = Vector3(0, 0, 0)  # Sine wave motion
                cmd.angular = Vector3(0, 0, 0)  # No rotatio
            

                
        
            self._output_pub.publish(cmd)
            rate.sleep()

if __name__ == '__main__':
    time.sleep(5)  # Wait before printing instructions

    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    teleop = KeyBoardVehicleTeleop()
