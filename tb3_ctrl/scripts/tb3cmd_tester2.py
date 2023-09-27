#!/usr/bin/env python3

import rospy
from tb3_ctrl.msg import TB3CmdMsg

def main():
    try:
        rospy.init_node('tb3cmd_tester')
        tb3cmd_pub = rospy.Publisher('/tb3cmd_listener', TB3CmdMsg, queue_size=1)
        rate = rospy.Rate(1)
        cmd_msg = TB3CmdMsg()
        cmd_msg.comando = 'IDLE'
        cmd_msg.valor = 0.0
        while not rospy.is_shutdown():
            cmd = str(input("Comando: " ))
            if cmd.lower() == 'terminar':
                break
            else:
                cmd_msg.comando = cmd
                val = float(input("Valor: "))
                cmd_msg.valor = val

            tb3cmd_pub.publish(cmd_msg)
            rate.sleep()

    except rospy.ROSInterruptException as e:
        print(str(e))


if __name__ == '__main__':
    main()