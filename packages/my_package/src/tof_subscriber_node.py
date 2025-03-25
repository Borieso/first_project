#!/usr/bin/env python3

import rospy
import os
from duckietown.dtros import DTParam, DTROS, NodeType
from sensor_msgs.msg import Range

class TofSubscriberNode(DTROS):
    # static parameters
    
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TofSubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._tof_topic = f"/{self._vehicle_name}/front_center_tof_driver_node/range"

    
        self._obstacle_present = False
        rospy.Subscriber(self._tof_topic, Range, self.cb_tof_range)
    
    def cb_tof_range(self, msg : Range):
        rospy.loginfo(f"I heard recieved {msg.range}")
    
if __name__ == '__main__':
    # create the node
    node = TofSubscriberNode(node_name='tof_subscriber_node')
    # keep spinning
    rospy.spin()

    
