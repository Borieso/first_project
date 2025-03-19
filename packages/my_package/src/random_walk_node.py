#!/usr/bin/env python3

import os
import rospy
from random import uniform
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
            linear_velocity = self._v # Randomize linear velocity between 0.1 and 0.5 m/s
            angular_velocity = uniform(-OMEGA, OMEGA)  # Randomize angular velocity between -OMEGA and OMEGA rad/s

            message = Twist2DStamped(v=linear_velocity, omega=angular_velocity)
            for count in range(0, 20):
                self._publisher.publish(message)
                rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = RandomWalkNode(node_name='random_walk_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
