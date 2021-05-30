## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import sys
import numpy as np
import cv2
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shutil
import os
import pybullet as p
from pybullet import getQuaternionFromEuler,getEulerFromQuaternion,getMatrixFromQuaternion,invertTransform
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.6f}".format(x)})
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

def getHomogeneousFormFromPose(eef_pose):
        #end-effector quaternion orietation
        eef_quat_ori = np.array([eef_pose.orientation.x,eef_pose.orientation.y,eef_pose.orientation.z,eef_pose.orientation.w]) 
        eef_R = getMatrixFromQuaternion(eef_quat_ori);
        Tbe = np.eye(4)
        Tbe[0:3,0:3] = np.reshape(eef_R,[3,3])
        Tbe[0,3] = eef_pose.position.x
        Tbe[1,3] = eef_pose.position.y
        Tbe[2,3] = eef_pose.position.z
        R = np.reshape(p.getMatrixFromQuaternion(p.getQuaternionFromEuler([0,0,-pi/4])),[3,3])
        Temp = np.eye(4)
        Temp[0:3,0:3] = R
        Tbe = np.matmul(Tbe,Temp)
        return Tbe

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_test',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
planning_frame = move_group.get_planning_frame()
count = 0;
shutil.rmtree("./Images")
shutil.rmtree("./Poses")
os.mkdir("./Images")
os.mkdir("./Poses")

while True:

        eef_pose = move_group.get_current_pose().pose 
        Tbe = getHomogeneousFormFromPose(eef_pose) #base to eef

        color_image = np.zeros([640,480,3],dtype=np.uint8)
        # Show images
        cv2.imshow("RGB",color_image)
        
        if cv2.waitKey(1)& 0xFF == 32: 
                imagename = "./Images/"+str(count).zfill(2)+".png"
                posename = "./Poses/"+str(count).zfill(2)+".txt"
                cv2.imwrite(imagename,color_image)
                np.savetxt(posename, Tbe)
                print(Tbe)
                print("saved "+str(count)+"th image & poses");
                count = count+1
        elif cv2.waitKey(1)& 0xFF == 27: 
                break;

