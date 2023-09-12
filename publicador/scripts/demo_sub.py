#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def suscriber():
    node = rospy.init_node('demo_sub_node')
    sub = rospy.Subscriber('demo_topic', String, leemensaje_clbk)
    rospy.spin()

def leemensaje_clbk(msg):
    
    rospy.loginfo(f"Recibi del topico el mensaje: {msg.data}")    


if __name__ == '__main__':
    pass