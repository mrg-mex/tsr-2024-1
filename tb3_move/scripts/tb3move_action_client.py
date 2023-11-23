#!7usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from tb3_move.msg import Tb3MoveGoal, Tb3MoveAction

class TB3ActionClient():
    def __init__(self):
        self._goal = Tb3MoveGoal()
        self._action_client = actionlib.SimpleActionClient('tb3move_action_srv', Tb3MoveAction)
        rospy.loginfo("Action Client inicializado.")

    def send_goal(self, x=0.0, y=0.0, theta=0.0, preempt_timeout=0.0):
        self._goal.header.seq = 1
        self._goal.header.frame_id = "map"
        self._goal.header.stamp = rospy.Time().now()

        self._goal.target.x = x
        self._goal.target.y = y
        self._goal.target.theta = theta

        rospy.loginfo("Conecatndo al server")
        self._action_client.wait_for_server()
        # --- El server respondio ---
        rospy.loginfo("Envio el goal")
        self._action_client.send_goal(self._goal, self._on_done, self._on_active, self._on_feedback)

    def _on_active(self):
        rospy.loginfo("ActionClient: New goal active!")
        state = self._action_client.get_state()
        rospy.loginfo(f"Goal [{self.get_state_name(state)}]")

    def _on_done(self):
        pass

    def _on_feedback(self):
        pass

    def get_state_name(self, state_code):
        state_name = ''
        if state_code == 0: state_name = 'PENDING'
        if state_code == 1: state_name = 'ACTIVE'
        if state_code == 2: state_name = 'PREEMPTED'
        if state_code == 3: state_name = 'SUCCEEDED'
        if state_code == 4: state_name = 'ABORTED'
        if state_code == 5: state_name = 'REJECTED'
        if state_code == 6: state_name = 'PREEMPTING'
        if state_code == 7: state_name = 'RECALLING'
        if state_code == 8: state_name = 'RECALLED'
        if state_code == 9: state_name = 'LOST'

        return state_name

def main():
    pass

if __name__ == '__main__':
    main()