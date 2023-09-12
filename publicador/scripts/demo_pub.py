#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publisher():
    nodo = rospy.init_node('demo_node')
    pub = rospy.Publisher('demo_topic',String, queue_size=10)

    r = rospy.Rate(10)
    msg = "Hola clase"

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

if __name__ == "__main__":
    publisher()
