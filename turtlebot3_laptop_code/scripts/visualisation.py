#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion
import time
import matplotlib.pyplot as plt

class DifferentialDriveRobot:
    def __init__(self, wheel_radius, wheel_base):
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.x = 0.0 # initial x position
        self.y = 0.0 # initial y position
        self.theta = 0.0 # initial orientation
        self.history = {'x': [], 'y': [], 'theta': []}

    def odom_callback(self, msg):
        # Extract pose information from Odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Append current pose to history
        self.history['x'].append(self.x)
        self.history['y'].append(self.y)
        self.history['theta'].append(self.theta)

    def plot_robot(self):
        plt.plot(self.history['x'], self.history['y'], 'ro')
        arrow_direction_x = -np.sin(self.theta)
        arrow_direction_y = np.cos(self.theta)
        plt.quiver(self.history['x'], self.history['y'], arrow_direction_x, arrow_direction_y)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Differential Drive Robot Simulation')
        plt.axis('equal')
        plt.grid(True)

        # Increase graph sensitivity
        #plt.xlim(self.x - 0.1, self.x + 0.1)
        #plt.ylim(self.y - 0.1, self.y + 0.1)
             
            
        # plt.xlim(0.01, None)
        # plt.ylim(0.01, None)
        # plt.plot(self.x, self.y, 'ro')
        # plt.quiver(self.x, self.y, np.cos(self.theta), np.sin(self.theta))
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.title('Differential Drive Robot Simulation')
        # plt.axis('equal')
        # plt.grid(True)

def main():
    # Initialize ROS node
    rospy.init_node('robot_visualization', anonymous=True)

    # Create instance of DifferentialDriveRobot class
    robot = DifferentialDriveRobot(wheel_radius=0.04, wheel_base=0.13)

    # Subscribe to the Odometry topic published by robot state publisher
    rospy.Subscriber('/odom', Odometry, robot.odom_callback)

    # Simulation loop
    while not rospy.is_shutdown():
        # time.sleep(1)  # Wait for 1 second
        robot.plot_robot()
        plt.pause(0.01)

    plt.show()

if __name__ == '__main__':
    main()
