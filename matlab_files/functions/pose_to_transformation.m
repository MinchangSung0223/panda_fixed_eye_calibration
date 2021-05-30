



function T = pose_to_transformation(pose)
T = eye(4);
x=pose(1);
y=pose(2);
z=pose(3);
roll = pose(4);
pitch = pose(5);
yaw = pose(6);
rotationMatrix = rotationVectorToMatrix([roll,pitch,yaw])
tform = trvec2tform([x,y,z]);
tform(1:3,1:3) =  rotationMatrix;
T = tform;
end