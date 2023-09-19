#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from tb3_ctrl.msg import TB3CmdMsg

variable = 5

class TB3CmdListener():
    def __init__(self):
        self._curr_lin_vel = 0
        self._curr_ang_vel = 0
        self._commands = ['avanza', 'gira', 'detente']
        self.WAFFLE_MAX_VEL_LIN = 0.26 # m/seg
        self.WAFFLE_MAX_VEL_ANG = 1.82 # rad/seg
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._tb3cmd_sub = rospy.Subscriber('/tb3cmd_listener',TB3CmdMsg, self._on_tb3cmd_clbk)

    def _constrains(self, input_val, high_val, low_val = 0 ):
        if input_val < low_val:
            input_val = low_val
            rospy.logwarn('Warning:')
        if input_val > high_val:
            input_val = high_val
            rospy.logwarn(f'Advertencia: El valor {input_val} es mas alto que el valor permitido, se usara {high_val} en su lugar.')

        return input_val       

    def _check_lin_vel_limit(self, vel):
        # Verificamos la velocidad lineal
        # vel -> [-0.26 hasta 0.26]
        # constrains ([valor a probar], [valor maximo], [malor minimo])
        vel = self._constrains(vel, self.WAFFLE_MAX_VEL_LIN, -self.WAFFLE_MAX_VEL_LIN)         

    def _check_ang_vel_limit(self, vel):
        # Verificamos la velocidad ang
        # vel -> [-1.82 hasta 1.82]
        # constrains ([valor a probar], [valor maximo], [malor minimo])
        vel = self._constrains(vel, self.WAFFLE_MAX_VEL_ANG, -self.WAFFLE_MAX_VEL_ANG)         

    def _check_vel_limits(self, vel, vel_tipo):
        if vel_tipo == 'lineal':
            vel = self._constrains(vel, self.WAFFLE_MAX_VEL_LIN, -self.WAFFLE_MAX_VEL_LIN)         
        else:
            vel = self._constrains(vel, self.WAFFLE_MAX_VEL_ANG, -self.WAFFLE_MAX_VEL_ANG)         

    def _on_tb3cmd_clbk(self, cmd):
        comando = cmd.comando  # 'Avanza' != 'avanza'
        robot_state = Twist()

        if comando.lower() in self._commands:
            if comando == 'detente':
                robot_state.linear.x = 0
                robot_state.angular.z = 0
            elif comando == 'avanza':
                valor = cmd.valor
                self._check_lin_vel_limit(valor)
                robot_state.linear.x = valor
            else:
                valor = cmd.valor
                self._check_ang_vel_limit(valor)
                robot_state.angular.z = valor    

            self._cmd_vel_pub.publish(robot_state)
        else:
            rospy.logwarn(f"El comando '{comando}' no es reconocido. Comandos válidos {self._commands}")

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('tb3_cmd')
        tb3cmd_obj = TB3CmdListener()
        tb3cmd_obj.loop()
    except rospy.ROSInterruptException as e:
        print(str(e))