#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from nav_msgs.msg import Odometry
from tb3_ctrl.srv import GetDistance, GetDistanceResponse, GetClosest, GetClosestResponse
import math

class GazeboUtils():
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            getwp = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            wp = getwp()
            if wp.success:
                return wp
            else:
                rospy.logwarn(f"Al invocar el servicio se devolvio el estatus {wp.success}, msg '{wp.status_message}'")
                return None
        except rospy.ServiceException as se:
            rospy.logerr(f"Error al invocar el servicio: {se}")

    def getModelState(self, model_name, relative_entity_name='world'):
        try:
            get_mst = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            mst = get_mst(model_name, relative_entity_name)
            if mst.success:
                return mst
            else:
                rospy.logwarn(f"Al invocar el servicio se devolvio el estatus {mst.success}, msg '{mst.status_message}'")
                return None
        except rospy.ServiceException as se:
            rospy.logerr(f"Error al invocar el servicio: {se}")

class DistanceMonitor():
    def __init__(self):
        self._landmarks = {}
        self._excluded_objs = ['ground_plane', 'turtlebot3_waffle']
        self._gazebo_utils = GazeboUtils()
        self._pose2d = Pose2D()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._on_odom_clbk)
        self._getClosestSrv = rospy.Service('/get_closest', GetClosest, self.get_closest_srv)
        self._getDistanceSrv = rospy.Service('/get_distance', GetDistance, self.get_distance_srv)
        self.load_landmarks()

    def load_landmarks(self):
        wp = self._gazebo_utils.getWorldProperties()
        if wp:
            for model in wp.model_names:
                ms = self._gazebo_utils.getModelState(model)
                position = (ms.pose.position.x, ms.pose.position.y)
                self._landmarks.update({model: position})

    def _on_odom_clbk(self, odom_msg):
        self._pose2d.x = odom_msg.pose.pose.position.x
        self._pose2d.y = odom_msg.pose.pose.position.y
        self._pose2d.theta = 0.0

    def is_world_empty(self):
        emp_cond = False
        if not self._landmarks:
            emp_cond = True
        else:
            obj_count = 0
            for model, (x,y) in self._landmarks.items():
                if model not in self._excluded_objs:
                    obj_count = obj_count + 1

            if obj_count == 0:
                emp_cond = True

        return emp_cond

    def get_closest_srv(self, msg):
        srv_resp = GetClosestResponse()

        self.refresh_ladmarks()

        if self.is_world_empty():
            srv_resp.closest_object = ''
            srv_resp.distance = 0.0
            srv_resp.success = False
            srv_resp.status_message = "GetClosest: No objects found."

            return srv_resp

        closest_ladmark = ''
        closest_distance = -1
        for model_name, (x, y) in self._landmarks.items():
            if model_name not in self._excluded_objs:
                dx = x - self._pose2d.x
                dy = y - self._pose2d.y
                # sqrt_dist = (dx^2) + (dy^2)
                sqrt_dist = math.hypot(dx, dy)
                if closest_distance == -1 or sqrt_dist < closest_distance:
                    closest_distance = sqrt_dist
                    closest_ladmark = model_name

        srv_resp.closest_object = closest_ladmark
        srv_resp.distance = math.sqrt(closest_distance)
        srv_resp.success = True
        srv_resp.status_message = "GetClosest: got closest object"

        return srv_resp

    def get_distance_srv(self, srv_req_msg):
        srv_resp = GetDistanceResponse()
        self.refresh_ladmarks()
        if srv_req_msg.object_name not in self._landmarks:
            srv_resp.distance = 0.0
            srv_resp.success = False
            srv_resp.status_message = f"GetDistance: '{srv_req_msg.object_name}' not found."
        
        x, y = self._landmarks[srv_req_msg.object_name]
        dx = x - self._pose2d.x
        dy = y - self._pose2d.y
        # obj_dist = math.sqrt((dx * dx) + (dy * dy))
        obj_dist = math.hypot(dx, dy)


        srv_resp.distance = obj_dist
        srv_resp.success = True
        srv_resp.status_message = "GetDistance: got distance."

        return srv_resp

    def refresh_ladmarks(self):
        self._landmarks.clear()
        self.load_landmarks()


def testGazeboUtils():
    gu = GazeboUtils()
    wp = gu.getWorldProperties()
    if wp:
        for model in wp.model_names:
            print(model)
    tb_vs_world = gu.getModelState('turtlebot3_waffle')
    if tb_vs_world:
        position = tb_vs_world.pose.position
        print(f'Pos TB3vsWorld is ({position.x}, {position.y})')       
    tb_vs_cyl = gu.getModelState('turtlebot3_waffle', 'unit_box')
    if tb_vs_cyl:
        position = tb_vs_cyl.pose.position
        print(f'Pos TB3vsWorld is ({position.x}, {position.y})')       

def main():
    rospy.init_node('tb3_monitor')
    monitor = DistanceMonitor()
    rospy.spin()


if __name__ == '__main__':
    main()
