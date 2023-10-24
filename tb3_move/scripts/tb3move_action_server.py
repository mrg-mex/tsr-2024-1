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
        pass


def main():
    pass

if __name__ == '__main__':
    main()