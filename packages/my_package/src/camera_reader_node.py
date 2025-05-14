#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Bool  # Import Bool message type

class CameraReaderNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(CameraReaderNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # Static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/apriltag_detector_node/detections/image/compressed"

        # Bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # Create window
        self._window = "Detect ducks"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        # Construct subscriber
        self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)
        # Construct publisher
        self.pub = rospy.Publisher(f"/{self._vehicle_name}/duck_detected", Bool, queue_size=10)

    def nothing(self, x):
        pass

    def callback(self, msg):
        # Convert ROS image to OpenCV format
        frame = self._bridge.compressed_imgmsg_to_cv2(msg)

        # Convert image to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Get trackbar values
        lower_h =97
        lower_s = 100
        lower_v = 126
        upper_h = 109
        upper_s = 255
        upper_v = 255

        # Define HSV color range
        lower_white = np.array([lower_h, lower_s, lower_v])
        upper_white = np.array([upper_h, upper_s, upper_v])

        # Create mask for white color
        mask = cv2.inRange(hsv, lower_white, upper_white)

        height, width = frame.shape[:2]

        # Define middle-bottom ROI
        roi_top = int(height * 2 / 3)  # Start at 2/3 of the height
        roi_bottom = height  # Bottom of the frame
        roi_left = int(width * 1 / 3)  # Start at 1/3 of the width
        roi_right = int(width * 2 / 3)  # End at 2/3 of the width

        # Extract only the middle-bottom part of the mask
        roi_mask = mask[roi_top:roi_bottom, roi_left:roi_right]

        # Check if there are any detected pixels in the ROI
        object_detected = np.any(roi_mask > 0)

        # Apply mask on original frame
        result = frame

        # If object is detected, print text on the image
        if object_detected:
            # Define text parameters
            text = "Object detected"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            color = (0, 255, 0)  # Green text
            thickness = 2

            # Get text size to center it
            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
            text_x = int((width - text_width) / 2)
            text_y = int(height - 30)  # Near the bottom

            # Put the text on the image
            cv2.putText(result, text, (text_x, text_y), font, font_scale, color, thickness)

        # Draw a rectangle around the ROI (for debugging)
        cv2.rectangle(result, (roi_left, roi_top), (roi_right, roi_bottom), (0, 255, 0), 2)
        
        cv2.imshow(self._window, result)
        self.pub.publish(object_detected)
        cv2.waitKey(1)

if __name__ == '__main__':
    # Create the node
    node = CameraReaderNode(node_name='camera_reader_node')
    # Keep spinning
    rospy.spin()
