#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from ultralytics import YOLO

import requests
import imutils


class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detection_publisher')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Publisher for annotated image
        self.image_pub = rospy.Publisher('/object_detection/image', Image, queue_size=10)

        # Publisher for object position
        self.object_position_pub = rospy.Publisher('/object_position', Point, queue_size=10)

        # Publisher for detected class
        self.class_pub = rospy.Publisher('/object_detection/class', String, queue_size=10)

        # Publisher for robot velocity
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Subscriber to start moving forward
        self.move_subscriber = rospy.Subscriber('/move_forward', String, self.move_forward_callback)

        # Load the YOLOv8 model
        self.model = YOLO('/home/ali/catkin_ws/src/Turtlebot_Burger/turtlebot3_laptop_code/scripts/statues_A.pt')
        
        self.url = "http://192.168.100.7:8080/shot.jpg"

        # Loop for processing frames
        self.rate = rospy.Rate(30)  # Adjust as needed

        # Start the robot moving forward at the beginning
        self.move_forward("start")

    def move_forward_callback(self, msg):
        self.move_forward(msg.data)

    def move_forward(self, command):
        twist = Twist()
        if command == "start":
            rospy.loginfo("Starting forward movement...")
            twist.linear.x = 3.6  # Adjust the forward velocity as needed
        elif command == "stop":
            rospy.loginfo("Stopping forward movement...")
            twist.linear.x = 0
        self.cmd_vel_pub.publish(twist)

    def detect_object(self):
        # Fetch image from the URL
        img_resp = requests.get(self.url)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
        img = cv2.imdecode(img_arr, -1)
        img = imutils.resize(img, width=1000, height=1800)

        # Run YOLOv8 tracking on the frame, persisting tracks between frames
        results = self.model.track(img, persist=True, conf=0.6)
        object_position = Point()
        object_position.x = 0
        object_position.y = 0

        detected_class = None

        if len(results[0].boxes) > 0:
            # Loop through all detected objects
            for cls, _, xyxy in zip(results[0].boxes.cls, results[0].boxes.conf, results[0].boxes.xyxy):
                class_label = None
                # Check the detected class and set the appropriate label
                if cls == 0:
                    class_label = "Chinese Fu Dog"
                elif cls == 1:
                    class_label = "Chopin"
                elif cls == 2:
                    class_label = "King-Tut"
                elif cls == 3:
                    class_label = "Nefertiti"

                if class_label:
                    detected_class = class_label
                    # Print the detected class
                    print(f"Detected class: {class_label}")

                    # Publish the class label
                    self.class_pub.publish(class_label)

                    # Extract bounding box coordinates
                    object_bbox = [int(coord) for coord in xyxy.tolist()]

                    # Calculate the center of the bounding box
                    object_position.x = (object_bbox[0] + object_bbox[2]) / 2
                    object_position.y = (object_bbox[1] + object_bbox[3]) / 2
                    self.object_position_pub.publish(object_position)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()

        # Convert the annotated frame from BGR to RGB
        annotated_frame_rgb = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)

        # Convert the frame to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(annotated_frame_rgb, encoding="rgb8")

        # Publish the annotated frame
        self.image_pub.publish(image_msg)

        # Gradually stop the robot if a specific object is detected
        if detected_class:
            self.gradual_stop()

    def gradual_stop(self):
        # Gradually reduce the robot's velocity to zero
        twist = Twist()
        rate = rospy.Rate(10)  # 10 Hz

        # Ensure the robot has stopped completely
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Robot stopped gradually.")

    def run(self):
        while not rospy.is_shutdown():
            self.detect_object()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ObjectDetector().run()
    except rospy.ROSInterruptException:
        pass
