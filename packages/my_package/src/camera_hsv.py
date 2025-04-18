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
        # Initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Create window
        self._window = "Adjust HSV"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        # Create trackbars for adjusting HSV values
        cv2.createTrackbar('Lower H', self._window, 0, 255, self.nothing)
        cv2.createTrackbar('Lower S', self._window, 0, 255, self.nothing)
        cv2.createTrackbar('Lower V', self._window, 150, 255, self.nothing)
        cv2.createTrackbar('Upper H', self._window, 220, 255, self.nothing)
        cv2.createTrackbar('Upper S', self._window, 30, 255, self.nothing)
        cv2.createTrackbar('Upper V', self._window, 255, 255, self.nothing)

        # Construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

    def nothing(self, x):
        pass

    def callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Get trackbar values
        lower_h = cv2.getTrackbarPos('Lower H', self._window)
        lower_s = cv2.getTrackbarPos('Lower S', self._window)
        lower_v = cv2.getTrackbarPos('Lower V', self._window)
        upper_h = cv2.getTrackbarPos('Upper H', self._window)
        upper_s = cv2.getTrackbarPos('Upper S', self._window)
        upper_v = cv2.getTrackbarPos('Upper V', self._window)

        # Define HSV color range
        lower_white = np.array([lower_h, lower_s, lower_v])
        upper_white = np.array([upper_h, upper_s, upper_v])

        # Create mask for white color
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # Apply mask on original frame
        result = cv2.bitwise_and(frame, frame, mask=mask_white)

        # Show result
        cv2.imshow(self._window, result)

        cv2.waitKey(1)

if __name__ == '__main__':
    # Create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # Keep spinning
    rospy.spin()
