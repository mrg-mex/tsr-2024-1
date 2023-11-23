#!/usr/bin/env python3

import rospy
from time import sleep
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from moveit_commander.move_group import MoveGroupCommander
from math import pi

DE2RA = pi/180

if __name__ == "__main__":
    rospy.init_node("dofbot_motion_plan")
    rospy.loginfo("Starting dofbot_motion_plan.")
    # Inicialicializamos MotionGroupCommander con el nombre
    # del grupo que queremos mover 
    dofbot = MoveGroupCommander("dofbot")
    # gripper = MoveGroupCommander("gripper_group")

    # Permiter replanificar cuando motion planning falla
    dofbot.allow_replanning(True)
    # Establecemos el tiempo de planeacion
    dofbot.set_planning_time(5)
    # Establecemos el numero max de intentos
    dofbot.set_num_planning_attempts(10)
    # Establecemos la tolerancia de la posicion de la meta (GOAL)
    dofbot.set_goal_position_tolerance(0.01)
    # Establecemos la tolerancia del error de la orientacion (attitude)
    dofbot.set_goal_orientation_tolerance(0.01)
    # Establece la tolerancia de la meta
    dofbot.set_goal_tolerance(0.01)
    # Establecemos vel y acc maxs
    dofbot.set_max_velocity_scaling_factor(0.1)
    dofbot.set_max_acceleration_scaling_factor(0.1)

    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0597016
    pose.position.z = 0.168051
    roll = -140.0
    pitch = 0.0
    yaw = 0.0
    q = quaternion_from_euler(roll * DE2RA, pitch * DE2RA, yaw * DE2RA)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    dofbot.set_pose_target(pose)
    for i in range(5):
        plan = dofbot.plan()
        if len(plan.joint_trajectory.points) != 0:
            dofbot.execute(plan)
            break
        else:
            rospy.loginfo("Plan error")


