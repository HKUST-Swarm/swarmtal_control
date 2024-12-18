#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import numpy as np

def odometry_callback(msg):
    global odom_publisher

    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    pos_z = msg.pose.pose.position.z

    modified_position = [pos_x, pos_y, pos_z]

    vel_x = msg.twist.twist.linear.x
    vel_y = msg.twist.twist.linear.y
    vel_z = msg.twist.twist.linear.z

    modified_velocity = [-vel_y, vel_x, vel_z]

    modified_odom = Odometry()
    modified_odom.header = msg.header
    modified_odom.header.frame_id = "world"
    
    modified_odom.pose.pose.position.x = modified_position[0]
    modified_odom.pose.pose.position.y = modified_position[1]
    modified_odom.pose.pose.position.z = modified_position[2]
    modified_odom.pose.pose.orientation = msg.pose.pose.orientation  # 保持姿态不变

    modified_odom.twist.twist.linear.x = modified_velocity[0]
    modified_odom.twist.twist.linear.y = modified_velocity[1]
    modified_odom.twist.twist.linear.z = modified_velocity[2]
    modified_odom.twist.twist.angular = msg.twist.twist.angular  # 保持角速度不变

    odom_publisher.publish(modified_odom)

if __name__ == "__main__":
    rospy.init_node("odom_transformer", anonymous=True)

    odom_publisher = rospy.Publisher("~odom_out", Odometry, queue_size=10)
    rospy.Subscriber("~odom_in", Odometry, odometry_callback)
    rospy.loginfo(f"Listening to odom_in and publishing transformed odometry to odom_out")
    rospy.spin()

