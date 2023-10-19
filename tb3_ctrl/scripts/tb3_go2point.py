#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from tf import transformations
import math

class Go2GoalNode():
    def __init__(self):
        rospy.init_node("go2goal")
        rospy.loginfo("Starting Go2PointNode as go2goal.")
        self._pose_act = Pose2D()
        self._distance_to_goal = 0.0
        self._phi = 0.0
        self._goal = Pose2D()
        self._tol_err_yaw = 0.0872665 # rad
        self._tol_err_dist = 0.05
        self._lin_vel = 0.01
        self._ang_vel = 0.02
        self._robot_state = 'STOP' # ['STOP', 'GO', 'TWIST', 'GOAL']
        self._goal_reached = False
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_feedback)
        self._cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def _on_odom_feedback(self, odom_msg):
        pose_act = odom_msg.pose.pose
        quaternion = [
            pose_act.orientation.x,
            pose_act.orientation.y,
            pose_act.orientation.z,
            pose_act.orientation.w

        ]
        ang_euler = transformations.euler_from_quaternion(quaternion) # tuple -> (roll, pitch, yaw)
        self._pose_act.x = pose_act.position.x
        self._pose_act.y = pose_act.position.y
        self._pose_act.theta = ang_euler[2]

    def _compute_goal(self):
        dx = (self._goal.x - self._pose_act.x)
        dy = (self._goal.y - self._pose_act.y)
        phi = math.atan2(dy, dx)
        dif_dist = math.hypot(dx, dy)
        dyaw = phi - self._pose_act.theta
        return dyaw, dif_dist
    
    def _heading_goal(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        rospy.loginfo(f'HEADING: Yaw err: {goal_yaw:.6f} rads, dist to go: {dist_to_goal:.6f} m.')

        if math.fabs(goal_yaw) > self._tol_err_yaw:
            ang_vel = self._ang_vel if goal_yaw > 0 else -self._ang_vel
            self._send_cmd_robot(vel_ang=ang_vel)
        else:
            self._robot_state == 'GO'

    def _go_straight(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        rospy.loginfo(f'GO: Yaw err: {goal_yaw:.6f} rads, dist to go: {dist_to_goal:.6f} m.')
        if self._robot_state not in ['STOP', 'GOAL']:
            if dist_to_goal > self._tol_err_dist:
                # muevete hacia 'GOAL'
                self._send_cmd_robot(vel_ang=self._ang_vel, vel_lin=self._lin_vel)
            else:
                self._robot_state = 'GOAL'
                rospy.loginfo(f'GOAL! Yaw err: {goal_yaw:.6f} rads, dist to go: {dist_to_goal:.6f} m.')
                self._goal_reached = True
                self._send_cmd_robot()
            
            if math.fabs(goal_yaw) > self._tol_err_yaw:
                self._robot_state = 'TWIST'
                
    def _send_cmd_robot(self, vel_ang=0.0, vel_lin=0.0):
        cmd_msg = Twist()
        cmd_msg.angular.z = vel_ang
        cmd_msg.linear.x = vel_lin

        self._cmdvel_pub.publish(cmd_msg)

    def set_goal(self, x, y, theta):
        self._goal.x = x
        self._goal.y = y
        self._goal.theta = theta

    def get_robot_state(self):
        return self._robot_state
    
    def start(self):
        self._robot_state = 'GO'

    def is_goal_reached(self):
        return self._goal_reached
    
    def stop(self):
        self._send_cmd_robot()
        rospy.sleep(1)

    def get_robot_pose(self):
        return self._pose_act        


if __name__ == "__main__":
    # declaraciones
    go2point = Go2GoalNode()
    go2point.set_goal(2, 3, 0)
    rate = rospy.Rate(1)
    
    # Condicion de inicio
    if go2point.get_robot_state() == 'STOP':
        # inicio del proceso
        go2point.start()

    # "EL PROCESO"
    while not rospy.is_shutdown():
        rospy.loginfo(f"Estado actual {go2point.get_robot_state()}")
        if go2point.get_robot_state() == 'TWIST':
            go2point._heading_goal()
        elif go2point.get_robot_state() == 'GO':
            go2point._go_straight()
        elif go2point.is_goal_reached():
            go2point.stop()
            goal_pose = go2point.get_robot_pose()
            rospy.loginfo(f"GOAL: Pose({goal_pose.x:.6f}, {goal_pose.y:.6f}, {goal_pose.theta:.6f})")
            break

    rospy.loginfo("YA ACABE!!!")

