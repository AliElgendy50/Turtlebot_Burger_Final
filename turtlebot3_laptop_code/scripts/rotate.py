#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math

class RotateRobot:
    def __init__(self):
        rospy.init_node('rotate_robot_node', anonymous=True)
        self.subscriber = rospy.Subscriber('/description/finished', String, self.rotate_callback)
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.move_pub = rospy.Publisher('/move_forward', String, queue_size=10)  # Publisher for move signal
        self.current_angle = 0.0
        self.angular_velocity = 0.0
        self.rate = rospy.Rate(10)  # Control loop frequency
        self.last_time = rospy.Time.now()
        self.rotating = False

        # PID control parameters
        self.Kp = 1.7  # Increased Proportional gain
        self.Ki = 1.1  # Integral gain
        self.Kd = 0.2  # Increased Derivative gain
        self.set_point = math.pi / 3  # Target angle in radians (90 degrees)

        # PID control variables
        self.error_prior = 0.0
        self.integral = 0.0

    def imu_callback(self, msg):
        # Assuming IMU message contains angular velocity in rad/s
        self.angular_velocity = msg.angular_velocity.z

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def pid_control(self, current_angle):
        # Calculate the error
        error = self.set_point - current_angle
        error = self.normalize_angle(error)

        # Update integral term with anti-windup
        self.integral += error
        self.integral = max(min(self.integral, 10), -10)  # Limiting the integral term

        # Compute PID control output
        output = self.Kp * error + self.Ki * self.integral + self.Kd * (error - self.error_prior)

        # Update error for next iteration
        self.error_prior = error

        # Ensure the output is within the robot's acceptable range
        min_output = 47
        max_output = 50  # Increased max_output
        output = max(min(output, max_output), min_output)

        return output

    def rotate_callback(self, data):
        if not self.rotating:
            self.rotating = True
            rospy.loginfo("Starting rotation...")
            cmd_vel_msg = Twist()
            self.current_angle = 0.0  # Reset current angle

            while not rospy.is_shutdown():
                current_time = rospy.Time.now()
                time_delta = (current_time - self.last_time).to_sec()
                self.current_angle += self.angular_velocity * time_delta
                self.current_angle = self.normalize_angle(self.current_angle)
                self.last_time = current_time

                # Apply PID control to adjust angular velocity
                pid_output = self.pid_control(self.current_angle)
                cmd_vel_msg.angular.z = pid_output
                self.publisher.publish(cmd_vel_msg)

                # Convert angle from radians to degrees
                current_angle_degrees = math.degrees(self.current_angle)

                # Print the current angle in degrees
                rospy.loginfo(f"Current angle: {current_angle_degrees} degrees")

                # Check if robot has rotated 90 degrees
                if abs(self.current_angle) >= (math.pi / 3):  # 90 degrees in radians
                    # Publish zero velocity to stop the robot
                    cmd_vel_msg.angular.z = 0
                    self.publisher.publish(cmd_vel_msg)
                    rospy.loginfo("Robot rotated 90 degrees. Stopping.")
                    self.move_pub.publish("start")  # Signal to start moving forward
                    break

                self.rate.sleep()

            self.rotating = False

if __name__ == '__main__':
    try:
        robot = RotateRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
