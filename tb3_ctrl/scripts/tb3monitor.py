#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from tb3_ctrl.srv import GetDistance, GetDistanceResponse, GetClosest, GetClosestResponse

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

def main():
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



if __name__ == '__main__':
    main()
