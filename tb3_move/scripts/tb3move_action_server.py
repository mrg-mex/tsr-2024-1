#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tb3_move.msg import Tb3MoveAction, Tb3MoveFeedback, Tb3MoveResult
from tf import transformations
import math

class Tb3MoveActionSrv():
    def __init__(self):
        # Descripcion actual del robot
        self._header = Header()
        self._ipose = Pose2D()
        # Estados del robot
        self._irobot_state_idx = 0
        self._robot_states = ['STOP', 'TWIST', 'GO', 'GOAL']
        # GOAL def y variables de control
        self._goal = None
        # self._goal_queue = []
        self._distance_to_goal = 0.0
        self._phi = 0.0
        self._tol_err_yaw = 0.0872665 # rad
        self._tol_err_dist = 0.05
        self._lin_vel = 0.1
        self._ang_vel = 0.1
        # Publicadores y subscriptores 
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_feedback)
        # Inicializacion del Simple Action Server
        self._action_server = actionlib.SimpleActionServer('/tb3move_action_srv', Tb3MoveAction, self._on_execute, False)
        rospy.loginfo("TB3Move Action Server: Inicializado")

    ##########################################
    # Aqui migramos las funciones necesarias #
    # del script tb3_go2point                #
    ##########################################

    # Callback del subscriptor al nodo d odometria '/odom'
    def _on_odom_feedback(self, odom_msg):
        pose_act = odom_msg.pose.pose
        quaternion = [
            pose_act.orientation.x,
            pose_act.orientation.y,
            pose_act.orientation.z,
            pose_act.orientation.w

        ]
        ang_euler = transformations.euler_from_quaternion(quaternion) # tuple -> (roll, pitch, yaw)
        self._ipose.x = pose_act.position.x
        self._ipose.y = pose_act.position.y
        self._ipose.theta = ang_euler[2]

    def _compute_goal(self):
        dx = (self._goal.x - self._ipose.x)
        dy = (self._goal.y - self._ipose.y)
        phi = math.atan2(dy, dx)
        dif_dist = math.hypot(dx, dy)
        dyaw = phi - self._ipose.theta
        return dyaw, dif_dist

    def _heading_goal(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        # rospy.loginfo(f'HEADING: Yaw err: {goal_yaw:.6f} rads, dist to go: {dist_to_goal:.6f} m.')
        self._distance_to_goal = dist_to_goal
        self._phi = goal_yaw
        if math.fabs(goal_yaw) > self._tol_err_yaw:
            ang_vel = self._ang_vel if goal_yaw > 0 else -self._ang_vel
            self._send_cmd_robot(vel_ang=ang_vel, robot_state_idx=1)
        else:
            # self._robot_states[2]: 'GO'
            self._irobot_state_idx = 2   

    def _go_straight(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        # rospy.loginfo(f'GO: Yaw err: {goal_yaw:.6f} rads, dist to go: {dist_to_goal:.6f} m.')
        self._distance_to_goal = dist_to_goal
        self._phi = goal_yaw
        if self._irobot_state_idx not in [0, 3]:
            if dist_to_goal > self._tol_err_dist:
                # muevete hacia 'GOAL'
                self._send_cmd_robot( vel_lin=self._lin_vel, robot_state_idx=2)
            else:
                self._irobot_state_idx = 3  # 'GOAL'
                # rospy.loginfo(f'GOAL! Yaw err: {goal_yaw:.6f} rads, dist to go: {dist_to_goal:.6f} m.')
                # self._goal_reached = True
                self._send_cmd_robot(robot_state_idx=3)
            
            if math.fabs(goal_yaw) > self._tol_err_yaw:
                self._irobot_state_idx = 1  # 'TWIST'

    def _send_cmd_robot(self, vel_ang=0.0, vel_lin=0.0, robot_state_idx=0):
        self._irobot_state_idx = robot_state_idx
        cmd_msg = Twist()
        cmd_msg.angular.z = vel_ang
        cmd_msg.linear.x = vel_lin

        self._cmd_vel_pub.publish(cmd_msg)

    def stop(self):
        self._send_cmd_robot(robot_state_idx=0)
        rospy.sleep(1)

    #######################################################
    # Aqui empieza la implementacion de SimpleActionServer #
    #######################################################

    def _on_execute(self, goal):    # Tb3MoveGoal
        # Bandera de proceso
        success = True
        rospy.loginfo("NEW GOAL received! (Reecibi una nueva meta)")

        if not self._accept_goal(goal):
            # GOAL rejected
            rospy.logerr("NEW GOAL abortada")
            self._action_server.set_aborted()
            return
        
        # NEW GOAL accepted
        rospy.loginfo(f"NEW GOAL accepted, GOAL({self._goal.x}, {self.goal.y}, {self.goal.theta} )")

        # En este punto ejecutamos el proceso de tb3_go2point
        while not self._irobot_state_idx == 3:
            # PREEMPT REQUEST
            if self._action_server.is_preempt_requested():
                success = False
                rospy.logwarn("PREEMPT flag recibida! Finalizando la accion actual")
                break
            if self._irobot_state_idx == 1:
                self._heading_goal()
            elif self._irobot_state_idx == 2:
                self._go_straight()
            else:
                success = False
                rospy.logerr(f"Assert error, irobot_state_code ({self._irobot_state_idx}.)")
                break
            
            self._send_feedback()

        # Al terminar el ciclo while
        # seccess = False - Mandar la respuesta del resultado como falso
        # success = True - Mandar la respuesta con los datos de salida

        result_msg = self._build_result_msg(success)
        self.stop()
        if success:
            self._action_server.set_succeeded(result_msg)
            rospy.loginfo("GOAL terminado exitosamente")
        else:
            self._action_server.set_preempted(result_msg)
            rospy.logwarn("Proceso no exitoso, GOAL PREEMPTED")

    
    def _accept_goal(self, new_goal):
        # Estados del robot
        #    0       1        2     3
        # ['STOP', 'TWIST', 'GO', 'GOAL']
        if self._irobot_state_idx == 0 or self._irobot_state_idx == 3:
            self._irobot_state_idx = 2
            self._goal = new_goal.target

            return True
        
        rospy.logwarn(f"Estado actual del robot: {self._robot_states[self._irobot_state_idx]}. GOAL rechazada")
        return False
    
    def _send_feedback(self):
        feedback_msg = Tb3MoveFeedback()
        feedback_msg.feedback_pose = self._ipose
        feedback_msg.distance_error = self._distance_to_goal
        feedback_msg.phi_error = self._phi
        feedback_msg.robot_state = self._robot_states[self._irobot_state_idx]

        self._action_server.publish_feedback(feedback_msg)

    def _build_result_msg(self, success):
        result_msgs = Tb3MoveResult()
        # HEADER
        result_msgs.header.seq = 1
        result_msgs.header.frame_id = ""
        result_msgs.header.stamp = rospy.Time.now()

        # Parametros de respuesta
        result_msgs.final_pose = self._ipose
        result_msgs.distance_error = self._distance_to_goal
        result_msgs.phi_error = self._phi
        result_msgs.succeess = success
        if success:
            result_msgs.result_message = 'GOAL completada exitosamente'
        else:
            result_msgs.result_message = "GOAL no completada"

        return result_msgs
    
    def start(self):
        rospy.loginfo("Action server started.")
        self._action_server.start()

def main():
    rospy.init_node('tb3_action_server_node')
    tb3actsrv = Tb3MoveActionSrv()
    tb3actsrv.start()
    rospy.spin()

if __name__ == '__main__':
    main()