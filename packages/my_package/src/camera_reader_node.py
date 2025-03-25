#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage

import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
        # construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert image to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Define HSV color ranges for white color
        lower_white = np.array([0, 0, 150])  # low saturation, high value for white
        upper_white = np.array([220, 30, 255])

        # Create mask for white color
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Find contours in the mask (even though we don't need the actual mask for further operations)
        contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw square contours around the detected white regions
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # You can adjust this area threshold for sensitivity
                # Get the bounding rectangle for each contour (square or rectangular bounding box)
                x, y, w, h = cv2.boundingRect(contour)
                
                # Draw a square/rectangle contour (green in this case)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green rectangle, 2px thickness

        # Resize the result for better display
        image_resized = cv2.resize(image, (600, 600))

        # Display the final image with square contours
        cv2.imshow('Square Contours Detection', image_resized)

        cv2.waitKey(1)
if __name__ == '__main__':
    # create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # keep spinning
    rospy.spin()
