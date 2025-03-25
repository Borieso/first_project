#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from std_msgs.msg import Bool

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.sub = rospy.Subscriber(f"/{self._vehicle_name}/obstacle_detected", Bool, self.callback)

    def callback(self, msg : Bool):
        rospy.loginfo("I heard '%s'", msg.Bool)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
