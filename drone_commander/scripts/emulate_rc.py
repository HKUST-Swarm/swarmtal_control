#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import pygame
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
import numpy as np
from transformations import euler_from_quaternion, quaternion_from_euler, euler_matrix, quaternion_matrix, quaternion_from_matrix

# This is a simple script that emulates a remote control for testing purposes by using a DJI C5 joystick.
# It publishes a constant stream of joystick messages to the /joy topic.
# The joystick has 8 channels

# Read the values from the joystick using pygame
def to_pwm(value):
    return int((value + 1.0) * 500.0 + 1000.0)

odom = Odometry()
received_odom = False
def odom_callback(msg):
    # Read the odom values
    global odom
    odom = msg
    # Rotate odom by 90 deg on z
    R_90 = euler_matrix(0, 0, -np.pi/2)[:3,:3]
    R = quaternion_matrix([odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z])[:3,:3]

    pos = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
    vel = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z])
    pos = R_90 @ pos
    vel = R_90 @ R @ vel
    odom.pose.pose.position.x = pos[0]
    odom.pose.pose.position.y = pos[1]
    odom.pose.pose.position.z = pos[2]
    odom.twist.twist.linear.x = vel[0]
    odom.twist.twist.linear.y = vel[1]
    odom.twist.twist.linear.z = vel[2]

    R = R_90 @ R
    # Extened R to 4x4
    R = np.vstack((np.hstack((R, np.zeros((3,1)))), np.array([0,0,0,1])))
    q = quaternion_from_matrix(R)
    odom.pose.pose.orientation.w = q[0]
    odom.pose.pose.orientation.x = q[1]
    odom.pose.pose.orientation.y = q[2]
    odom.pose.pose.orientation.z = q[3]
    ang_vel = np.array([odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z])
    ang_vel = R_90 @ ang_vel
    odom.twist.twist.angular.x = ang_vel[0]
    odom.twist.twist.angular.y = ang_vel[1]
    odom.twist.twist.angular.z = ang_vel[2]
    global received_odom
    received_odom = True
    odom.header.stamp = rospy.get_rostime()


def main():
    # Initialize the node
    rospy.init_node('emulate_joy', anonymous=True)
    # Initialize the publisher
    pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
    pub_odom = rospy.Publisher('/sim_odometry', Odometry, queue_size=10)
    # Subscribe mavros odom
    sub = rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback)
    # Initialize the joystick
    pygame.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    # Initialize the message
    msg = OverrideRCIn()
    # Loop
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        # Read the joystick values
        pygame.event.pump()
        # In case of joystick is DJI C5
        if joystick.get_name() == 'DJI C5':
            msg.channels[0] = to_pwm(joystick.get_axis(0))
            msg.channels[1] = to_pwm(joystick.get_axis(1))
            msg.channels[2] = to_pwm(joystick.get_axis(2))
            msg.channels[3] = to_pwm(joystick.get_axis(3))
            # Use B6 and B7 as to simulate axis 4
            # B6=0 B7=1 axis = 1100
            # B6=1 B7=0 axis = 1500
            # B6=0 B7=0 axis = 1900
            if joystick.get_button(6) == 0 and joystick.get_button(7) == 1:
                msg.channels[6] = 1100
            elif joystick.get_button(6) == 1 and joystick.get_button(7) == 0:
                msg.channels[6] = 1500
            elif joystick.get_button(6) == 0 and joystick.get_button(7) == 0:
                msg.channels[6] = 1900
            # Use B4 and B5 as to simulate axis 5
            # B4=0 B5=1 axis = 1100
            # B4=1 B5=0 axis = 1500
            # B4=0 B5=0 axis = 1900
            if joystick.get_button(4) == 0 and joystick.get_button(5) == 1:
                msg.channels[7] = 1100
            elif joystick.get_button(4) == 1 and joystick.get_button(5) == 0:
                msg.channels[7] = 1500
            elif joystick.get_button(4) == 0 and joystick.get_button(5) == 0:
                msg.channels[7] = 1900
        # Publish the message
        pub.publish(msg)
        if received_odom:
            pub_odom.publish(odom)
        # Sleep
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass