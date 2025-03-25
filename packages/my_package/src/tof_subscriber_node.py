#!/usr/bin/env python3

import rospy
import os
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import Range
from std_msgs.msg import Bool  # Import Bool message type

class TofSubscriberNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TofSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        # Ensure VEHICLE_NAME is set
        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        self._tof_topic = f"/{self._vehicle_name}/front_center_tof_driver_node/range"
        self._prev_obstacle = None

        # Obstacle detection threshold
        self.obstacle_threshold = 0.2  # 20 cm (0.2 meters)

        # Publisher for obstacle detection
        self._obstacle_pub = rospy.Publisher(f"/{self._vehicle_name}/obstacle_detected", Bool, queue_size=10)

        # Subscribe to the ToF sensor topic
        rospy.Subscriber(self._tof_topic, Range, self.cb_tof_range)

    def cb_tof_range(self, msg: Range):
        # Check if obstacle is closer than threshold
        is_obstacle = msg.range < self.obstacle_threshold

        if (self._prev_obstacle == None):
            self._obstacle_pub.publish(is_obstacle)
            self._prev_obstacle = is_obstacle
        
        if is_obstacle and not self._prev_obstacle:
            # Publish obstacle status
            self._obstacle_pub.publish(is_obstacle)
            self._prev_obstacle = True
        if not is_obstacle and self._prev_obstacle:
            self._obstacle_pub.publish(is_obstacle)


        

        # Log the detection
        rospy.loginfo(f"Received range data: {msg.range:.2f} meters | Obstacle: {is_obstacle}")

if __name__ == '__main__':
    node = TofSubscriberNode(node_name='my_subscriber_node')
    rospy.spin()
