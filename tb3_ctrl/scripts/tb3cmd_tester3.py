#!/usr/bin/env python3

import rospy
from tb3_ctrl.msg import TB3CmdMsg
import keyutil

def main():
    try:
        rospy.init_node('tb3cmd_tester')
        tb3cmd_pub = rospy.Publisher('/tb3cmd_listener', TB3CmdMsg, queue_size=1)
        rate = rospy.Rate(10)
        cmd_msg = TB3CmdMsg()
        cmd_msg.comando = ''
        cmd_msg.valor = 0.0
        prompt = 'Ingresa el comando'
        input_string = ''

        while not rospy.is_shutdown():
            key = keyutil.getKey()
            if key == keyutil.CTRL_C_CHAR:
                break
            elif key == keyutil.BACKSPC_CHAR:
                input_string = input_string[:-1]
            elif key == keyutil.CR_CHAR:
                cmd_str = keyutil.parseCommand(input_string)
                if len(cmd_str) > 0 and cmd_str[0] != '':
                    if cmd_str[0].lower() == 'terminar':
                        break
                    else:
                        cmd_msg.comando = cmd_str[0] 
                        val = float(cmd_str[1])
                        cmd_msg.valor = val
                        
                    print('\ncomando:  %s valor: %.2f' % (cmd_str[0], float(cmd_str[1])))    
                input_string = ''
            else:
                input_string += key
                print('%s: %s' % (prompt, input_string), end='\r')

            if cmd_msg.comando != '':
                tb3cmd_pub.publish(cmd_msg)
            
            rate.sleep()

    except rospy.ROSInterruptException as e:
        print(str(e))


if __name__ == '__main__':
    main()