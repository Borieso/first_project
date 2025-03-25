#!/usr/bin/env python3

import os
import rospy
from random import uniform
from random import randint
from std_msgs.msg import Bool  # Import Bool message type
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
        self._vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        # construct subscriber
        self.object_topic = f"/{self._vehicle_name}/obstacle_detected"
        self._subscriber_object = rospy.Subscriber(self.object_topic, Bool, self.listen)
        self.object_detected = False
        

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if(self.object_detected):
                rospy.loginfo("Object was detected:  GO BACKWARDS")

                tijd = randint(5,10)
                message_lineair = Twist2DStamped(v=-self._v, omega=0)
                for count in range(tijd):
                    self._publisher.publish(message_lineair)
                    rate.sleep()
            else:
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
                    if(self.object_detected):
                        break

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)
        rospy.loginfo("node stopped")
    
    def listen(self, data):
        self.object_detected = data.data
        rospy.loginfo("Object detected: {%s}", data.data)




if __name__ == '__main__':
    # create the node
    node = RandomWalkNode(node_name='random_walk_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
