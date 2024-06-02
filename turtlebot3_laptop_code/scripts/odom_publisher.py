#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

def odom_callback(data):
    odom_str = data.data
    odom_data = odom_str.split(',')
    
    encoder_angular_A = float(odom_data[0].split(':')[1])
    encoder_angular_B = float(odom_data[1].split(':')[1])
    encoder_linearVelocity = float(odom_data[2].split(':')[1])
    gyro_z = float(odom_data[3].split(':')[1])

    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"
    odom_msg.twist.twist.linear.x = encoder_linearVelocity
    odom_msg.twist.twist.angular.z = gyro_z

    odom_pub.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('odom_processor', anonymous=True)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    rospy.Subscriber('raw_odom', String, odom_callback)
    rospy.spin()
