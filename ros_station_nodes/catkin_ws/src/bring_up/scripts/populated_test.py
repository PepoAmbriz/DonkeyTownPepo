#!/usr/bin/env python
import rospy
import rospkg
import math
import numpy
from std_msgs.msg import Int16
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import SpawnModelRequest
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetLinkStateRequest
from gazebo_msgs.srv import GetLinkStateResponse


class spawn_obs_req:
    def __init__(self,name,x,y,yaw):
        rospack  = rospkg.RosPack()
        pkg_path = rospack.get_path('autonomos_gazebo_simulation')
        self.spawn_req = SpawnModelRequest()
        self.spawn_req.model_name = name
        self.spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini_obstacle/model.sdf','r').read()
        self.spawn_req.robot_namespace = ''
        self.spawn_req.initial_pose.position.x = x
        self.spawn_req.initial_pose.position.y = y
        self.spawn_req.initial_pose.position.z = 0.17
        self.spawn_req.initial_pose.orientation.z = math.sin(yaw/2)
        self.spawn_req.initial_pose.orientation.w = math.cos(yaw/2)
def main():
    rospy.init_node("test2_start")
    print("Waiting for service /gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    print("Service is now available")
    rospack  = rospkg.RosPack()
    pkg_path = rospack.get_path('autonomos_gazebo_simulation')
    
    spawn_model    = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_req      = SpawnModelRequest()
    

    angle = numpy.random.uniform(math.pi-0.3, math.pi+0.3)
    pos_x = numpy.random.uniform(1.2,3.8) 

    spawn_req.model_name = "AutoModelMini"
    spawn_req.model_xml  = open(pkg_path + '/models/AutoNOMOS_mini/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x
    spawn_req.initial_pose.position.y = 3.3
    spawn_req.initial_pose.position.z = 0.17
    spawn_req.initial_pose.orientation.z = math.sin(angle/2)
    spawn_req.initial_pose.orientation.w = math.cos(angle/2)
    #spawn_model(spawn_req)

    spawn_req.model_name = "StopSign"
    spawn_req.model_xml  = open(pkg_path + '/models/stop_sign_small/model.sdf','r').read()
    spawn_req.robot_namespace = ''
    spawn_req.initial_pose.position.x = pos_x - 0.2
    spawn_req.initial_pose.position.y = 3.55
    spawn_req.initial_pose.position.z = 0.0
    spawn_req.initial_pose.orientation.z = math.sin(1.5708/2)
    spawn_req.initial_pose.orientation.w = math.cos(1.5708/2)
    spawn_model(spawn_req)
     
"""
    xgrid = numpy.linspace(-2.0,2.0,1)
    ygrid = numpy.linspace(-1.5,1.5,1)
    count = 1
    for xc in xgrid: 
        for yc in ygrid: 
            yaw = angle = numpy.random.uniform(math.pi-0.3, math.pi+0.3)
            obs_spawn = spawn_obs_req("AutoModel_Obstacle"+str(count),xc,yc,yaw)
            spawn_model(obs_spawn.spawn_req)
            count = count+1
"""

    
if __name__ == "__main__":
    main()
