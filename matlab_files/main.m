close all;
clear
addpath("functions")
format shortG

imageFolder='./Images'
poseFolder='./Poses'
squareSize = 15%mm
board_size =[8,11]

Tbc = [-1 0 0 0.5;...
        0 0 -1 0.5;...
        0 -1 0 0.5...
        ; 0 0 0 1]
    
[imagePoints,worldPoints]=load_images(imageFolder,squareSize,board_size);
armMat = load_poses(poseFolder);
image_num = size(imagePoints,3);
cameraParams = estimateCameraParameters(imagePoints,worldPoints);
worldPoints = [worldPoints, zeros(size(worldPoints,1),1), ones(size(worldPoints,1),1)]';

A = zeros(4,4,image_num-1);
B = zeros(4,4,image_num-1);

for i=1:1:image_num-2
    TbE1 = (armMat(:,:,i));    
    TbE2 = (armMat(:,:,i+1));    
    TH1C = TransInv(pose_R_to_transformation(cameraParams.TranslationVectors(i,:)./1000,cameraParams.RotationMatrices(:,:,i)'));
    TH2C = TransInv(pose_R_to_transformation(cameraParams.TranslationVectors(i+1,:)./1000 ,cameraParams.RotationMatrices(:,:,i+1)'));
    A(:,:,i) = TransInv(TbE2)*(TbE1);
    B(:,:,i) = (TH2C)*TransInv(TH1C);
end
X=calibration_AX_XB(A,B)
cam_pose_list = zeros(4,4,image_num)
for i =1:1:image_num
    TbE1 = (armMat(:,:,i));
    TH1C = TransInv(pose_R_to_transformation(cameraParams.TranslationVectors(i,:)./1000,cameraParams.RotationMatrices(:,:,i)'));
    cam_pose = TbE1*X*(TH1C);
    cam_pose_list(:,:,i) = cam_pose;
end

clc;
initial_baseToCamTransformation = mean(cam_pose_list,3)
initial_EndEffectorToBoardTransformation =X

squareSize = 15;
[TBaseToCam, TEndToBoard, cameraParams, TBaseStd, TEndStd, pixelErr] = CalCamArm(imageFolder, armMat, squareSize,'maxBaseOffset',1,'gripEst',initial_EndEffectorToBoardTransformation,'baseEst',TransInv(initial_baseToCamTransformation));
save("./Result/TBaseToCam.txt",'TBaseToCam','-ascii');
save("./Result/TEndToBoard.txt",'TEndToBoard','-ascii');


