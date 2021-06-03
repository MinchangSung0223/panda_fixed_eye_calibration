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

checkrboard_size = (8, 11) 
def getHomogeneousFormFromPose(eef_pose):
        #end-effector quaternion orietation
        eef_quat_ori = np.array([eef_pose.orientation.x,eef_pose.orientation.y,eef_pose.orientation.z,eef_pose.orientation.w]) 
        eef_R = getMatrixFromQuaternion(eef_quat_ori);
        Tbe = np.eye(4)
        Tbe[0:3,0:3] = np.reshape(eef_R,[3,3])
        Tbe[0,3] = eef_pose.position.x
        Tbe[1,3] = eef_pose.position.y
        Tbe[2,3] = eef_pose.position.z
        
        return Tbe

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_test',anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
planning_frame = move_group.get_planning_frame()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
shutil.rmtree("./Images")
shutil.rmtree("./Poses")
os.mkdir("./Images")
os.mkdir("./Poses")
# Start streaming
pipeline.start(config)
count = 0;
shutil.rmtree("./Images")
shutil.rmtree("./Poses")
os.mkdir("./Images")
os.mkdir("./Poses")

try:
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))



        eef_pose = move_group.get_current_pose().pose 
        Tbe = getHomogeneousFormFromPose(eef_pose) #base to eef

        # Show images
        _img = cv2.cvtColor(color_image.copy(),cv2.COLOR_BGR2RGB)
        img = cv2.cvtColor(_img.copy(),cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img.copy(),cv2.COLOR_RGB2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (checkrboard_size[1],checkrboard_size[0]),None)
        if ret==True:
                img_chess = cv2.drawChessboardCorners(img, (checkrboard_size[1],checkrboard_size[0]), corners,ret)
                cv2.imshow('RealSense', img_chess)
        else:	
                cv2.imshow('RealSense', images)
        if cv2.waitKey(1)& 0xFF == 32: 

                imagename = "./Images/"+str(count).zfill(2)+".png"
                posename = "./Poses/"+str(count).zfill(2)+".txt"
                cv2.imwrite(imagename,color_image)
                np.savetxt(posename, Tbe)
                print("saved "+str(count)+"th image & poses");
                count = count+1
        elif cv2.waitKey(1)& 0xFF == 27: 
                print("EXIT");
                break;

finally:

    # Stop streaming
    pipeline.stop()
