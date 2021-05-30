import pybullet as p
import time
import numpy as np
import math
from datetime import datetime
import pybullet_data
import rospy
import cv2
from sensor_msgs.msg import JointState
import struct
from scipy.io import savemat
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import shutil
import os
import threading
joint_states = [0.000 ,-0.785 ,0.000 ,-2.356 ,0.000 ,1.571 ,1.585];
rp =  [0.000 ,-0.785 ,0.000 ,-2.356 ,0.000 ,1.571 ,1.585];
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
ll = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]
ul = [2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973]

near = 0.01
far = 1000
def callback(data):
	global joint_states
	joint_states = data.position
	
	#print(data)
	print(joint_states)

def convert_depth_frame_to_pointcloud(depth_image):
	camera_intrinsics ={"fx":554.2563,"ppx": 320,"fy":415.6922,"ppy":240}
	[height, width] = depth_image.shape
	nx = np.linspace(0, width-1, width)
	ny = np.linspace(0, height-1, height)
	u, v = np.meshgrid(nx, ny)
	x = (u.flatten() - camera_intrinsics["ppx"])/camera_intrinsics["fx"]
	y = (v.flatten() - camera_intrinsics["ppy"])/camera_intrinsics["fy"]

	z = depth_image.flatten() / 1000;
	x = np.multiply(x,z)
	y = np.multiply(y,z)

	x = x[np.nonzero(z)]
	y = y[np.nonzero(z)]
	z = z[np.nonzero(z)]
	return x, y, z

 
def getCameraImage(cam_pos,cam_orn):
	fov = 60
	aspect = 640/480
	angle = 0.0;
	q = p.getQuaternionFromEuler(cam_orn)
	cam_orn = np.reshape(p.getMatrixFromQuaternion(q ),(3,3));
	view_pos = np.matmul(cam_orn,np.array([-0.001,0,0.0]).T)
	view_pos = np.array(view_pos+cam_pos)
	view_matrix = p.computeViewMatrix([cam_pos[0],cam_pos[1],cam_pos[2]], view_pos, [0,0,1])
	projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
	images = p.getCameraImage(640,
					480,
					view_matrix,
					projection_matrix,
					shadow=False,
					renderer=p.ER_BULLET_HARDWARE_OPENGL)
	return images
if __name__ == "__main__":
	clid = p.connect(p.SHARED_MEMORY)
	if (clid < 0):
		p.connect(p.GUI)
		#p.connect(p.SHARED_MEMORY_GUI)

	p.setAdditionalSearchPath(pybullet_data.getDataPath())

	#p.loadURDF("plane.urdf", [0, 0, -1.0])
	tableId=p.loadURDF("urdf/shelfandtable/shelfandtable.urdf", [0, 0, 0.0])
	
	d435Id = p.loadURDF("urdf/d435/d435.urdf", [0, 0, 0.0])
	p.resetBasePositionAndOrientation(d435Id, [0.5, 0.5, 0.5],p.getQuaternionFromEuler([0,-math.pi,-math.pi/2]))

	pandaId = p.loadURDF("urdf/Panda/panda_.urdf", [0, 0, 0])
	p.resetBasePositionAndOrientation(pandaId, [0, 0, 0], [0, 0, 0, 1])
	cid = p.createConstraint(tableId, -1, pandaId, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0., 0., 0],[0, 0, 0, 1])
	x_Id = p.addUserDebugParameter("x", 0, 1, 0.4)
	y_Id = p.addUserDebugParameter("y", -1, 1, 0.0)
	z_Id = p.addUserDebugParameter("z", 0, 2, 0.589)
	roll_Id = p.addUserDebugParameter("roll", -math.pi*2, math.pi*2, 0.0)
	pitch_Id = p.addUserDebugParameter("pitch", -math.pi*2, math.pi*2, -math.pi/2)
	yaw_Id = p.addUserDebugParameter("yaw", -math.pi*2, math.pi*2, math.pi/2)

	pandaEndEffectorIndex = 7
	numJoints = 7
	t = 0
	capture_count = 0;
	save_count = 0;
	for i in range(numJoints):
			p.resetJointState(pandaId, i, joint_states[i])
	images_list = np.array([]);
	poses_list = np.array([]);
	poses2_list = np.array([]);
	
	jointPoses_list = np.array([]);
	
	while 1:
		events = p.getKeyboardEvents()	
		key=0;
		for key in events:
			key=key;
			if key==32:
				capture_count =capture_count+1;

		x = p.readUserDebugParameter(x_Id)
		y = p.readUserDebugParameter(y_Id)
		z = p.readUserDebugParameter(z_Id)
		roll = p.readUserDebugParameter(roll_Id)
		pitch = p.readUserDebugParameter(pitch_Id)
		yaw = p.readUserDebugParameter(yaw_Id)
		
		
		t = t + 0.001
		pos = [x, y,z]
		orn = p.getQuaternionFromEuler([roll,pitch,yaw])
		jointPoses = p.calculateInverseKinematics(pandaId, pandaEndEffectorIndex, pos, orn, ll, ul,jr, rp)


		d435pos, d435orn = p.getBasePositionAndOrientation(d435Id)
		d435quat = d435orn
		d435orn =  p.getEulerFromQuaternion(d435orn)
		T = np.eye(4)
		
		R = p.getMatrixFromQuaternion(d435quat)
		T[0:3,0:3]=np.reshape(R,[3,3]);
		T[0,3] = d435pos[0];
		T[1,3] = d435pos[1];
		T[2,3] = d435pos[2];
		

		for i in range(numJoints):
			p.resetJointState(pandaId, i, jointPoses[i])
		image = getCameraImage(d435pos,d435orn)

		depth_img = np.array(image[3],dtype=np.float)
		depth_img = far * near / (far - (far - near) * depth_img)
		
		color_img = image[2]
		T = np.eye(4)
		eef_pose = p.getLinkState(pandaId,pandaEndEffectorIndex)
		eef_tvec = eef_pose[0];
		R = p.getMatrixFromQuaternion(eef_pose[1])
		T[0:3,0:3] = np.reshape(R,[3,3])
		T[0,3] =eef_tvec[0]
		T[1,3] =eef_tvec[1]
		T[2,3] =eef_tvec[2]
		eef_pose = T
		eef_pose2 = p.getLinkState(pandaId,pandaEndEffectorIndex+2)
		eef_tvec2 = eef_pose2[0];
		T2 = np.eye(4);
		R2 = p.getMatrixFromQuaternion(eef_pose2[1])
		T2[0:3,0:3] = np.reshape(R2,[3,3])
		T2[0,3] =eef_tvec2[0]
		T2[1,3] =eef_tvec2[1]
		T2[2,3] =eef_tvec2[2]
		eef_pose2 = T2
		
		jointPoses = np.array(jointPoses)
		if capture_count==3:
			print(str(save_count)+"th image saved.")
			
			po = np.matmul(T2,np.array([0,0,0,1]).T)
			px = np.matmul(T2,np.array([0.5,0,0,1]).T)
			py = np.matmul(T2,np.array([0,0.5,0,1]).T)
			pz = np.matmul(T2,np.array([0,0,0.5,1]).T)
			x_color=np.array([1,0,0])
			y_color=np.array([0,1,0])
			z_color=np.array([0,0,1])
			lineWidth = 1
			lifeTime = 1
			p.addUserDebugLine([po[0],po[1],po[2]], [px[0],px[1],px[2]], x_color, 1, 0)
			p.addUserDebugLine([po[0],po[1],po[2]], [py[0],py[1],py[2]], y_color, 1, 0)
			p.addUserDebugLine([po[0],po[1],po[2]], [pz[0],pz[1],pz[2]], z_color, 1, 0)				

			po = np.matmul(T,np.array([0,0,0,1]).T)
			px = np.matmul(T,np.array([0.5,0,0,1]).T)
			py = np.matmul(T,np.array([0,0.5,0,1]).T)
			pz = np.matmul(T,np.array([0,0,0.5,1]).T)
			x_color=np.array([1,0,0])
			y_color=np.array([0,1,0])
			z_color=np.array([0,0,1])
			lineWidth = 1
			lifeTime = 1
			p.addUserDebugLine([po[0],po[1],po[2]], [px[0],px[1],px[2]], x_color, 1, 0)
			p.addUserDebugLine([po[0],po[1],po[2]], [py[0],py[1],py[2]], y_color, 1, 0)
			p.addUserDebugLine([po[0],po[1],po[2]], [pz[0],pz[1],pz[2]], z_color, 1, 0)				


			if save_count==0:
				images_list=np.array(color_img,dtype=np.uint8);
				s = color_img.shape
				images_list = np.reshape(images_list,(s[0],s[1],s[2],1))

				s2 = eef_pose.shape
				poses_list = np.reshape(eef_pose,(s2[0],s2[1],1))
				poses2_list = np.reshape(eef_pose2,(s2[0],s2[1],1))
				s3 = jointPoses.shape
				jointPoses_list = np.reshape(jointPoses,(s3[0],1))

				save_count = save_count+1
				capture_count=0;

				continue
			s = color_img.shape
			color_img = np.reshape(color_img,(s[0],s[1],s[2],1))
			s2 = eef_pose.shape
			eef_pose = np.reshape(eef_pose,(s2[0],s2[1],1))
			eef_pose2 = np.reshape(eef_pose2,(s2[0],s2[1],1))
			s3 = jointPoses.shape
			jointPoses = np.reshape(jointPoses,(s3[0],1))

			images_list=np.append(images_list,color_img,axis=3);
			poses_list=np.append(poses_list,eef_pose,axis=2);
			poses2_list=np.append(poses2_list,eef_pose2,axis=2);			
			jointPoses_list=np.append(jointPoses_list,jointPoses,axis=1);


			print(images_list.shape)
			print(poses_list.shape)
			print(jointPoses_list.shape)
			
			save_count = save_count+1

			#savedic = {"image": color_img, "pose": [x,y,z,roll,pitch,yaw],"jointPoses":jointPoses}
			#savemat(str(save_count)+".mat", savedic)
			#save_count = save_count+1;
			capture_count=0;
		if key==65309:
			savedic = {"image": images_list, "armMat": poses_list,"pose2":poses2_list,"jointPoses":jointPoses_list}
			s = images_list.shape
			shutil.rmtree("./matlab_files/Images")
			shutil.rmtree("./matlab_files/Poses")
			os.mkdir("./matlab_files/Images")
			os.mkdir("./matlab_files/Poses")
			
			
			for i in range(0,s[3]):
				imagename = "./matlab_files/Images/"+str(i).zfill(2)+".png"
				posename = "./matlab_files/Poses/"+str(i).zfill(2)+".txt"
				
				cv2.imwrite(imagename,images_list[:,:,0:3,i])
				np.savetxt(posename, poses_list[:,:,i])
			
			savemat("./matlab_files/data.mat", savedic)
			break;
			
			
		p.stepSimulation()
	print("\n\n\n\n\n\n")
	print("===========================DATA WAS SAVED=================================");
	print("\n\n\n\n\n\n")
	p.disconnect()
