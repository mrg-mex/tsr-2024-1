#! /usr/bin/env python3

import rospy
from tb3_ctrl.msg import TB3CmdMsg

def publicador():
    nodo = rospy.init_node('tb3cmd_node')
    pub = rospy.Publisher('/tb3cmd_msg_echo', TB3CmdMsg, queue_size=1)

    r = rospy.Rate(10)
    msg = TB3CmdMsg()
    msg.comando = 'avanza' # valores: ['avanza', 'gira', 'detente']
    msg.valor = 1.5 # ROS toma la vel lin como m/s y la vel ang rad/s

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    publicador()
