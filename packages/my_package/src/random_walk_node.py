#!/usr/bin/env python3

import os
import rospy
from random import uniform
from random import randint
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped


# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 2.0     # angular vel   , rad/s     , counter clock wise (+)


class RandomWalkNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RandomWalkNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            hoek = -1 if randint(0,1) == 0 else 1
            message_angle = Twist2DStamped(v=0, omega=self._omega*hoek)
            tijd =  randint(1,6) # Randomize angular v elocity between -OMEGA and OMEGA rad/s
            for i in range(tijd):
                self._publisher.publish(message_angle)
                rate.sleep()
            
            message_lineair = Twist2DStamped(v=self._v, omega=0)
            tijd =  randint(5,10)
            for count in range(tijd):
                self._publisher.publish(message_lineair)
                rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)
        rospy.loginfo("node stopped")

if __name__ == '__main__':
    # create the node
    node = RandomWalkNode(node_name='random_walk_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
