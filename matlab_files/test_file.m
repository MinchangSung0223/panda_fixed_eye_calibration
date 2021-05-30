close all;
clear
addpath("functions")
format shortG
load data.mat
s = size(image)
squareSize = 15%mm
board_size =[8,11]
imagePoints_=zeros(board_size(1)*board_size(2),2,s(4));
worldPoints_=zeros(board_size(1)*board_size(2),2,s(4));
Tbc = [-1 0 0 0.5;...
        0 0 -1 0.5;...
        0 -1 0 0.5...
        ; 0 0 0 1]
    
for i =1:1:s(4)
    I = image(:,:,1:3,i);
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    imagePoints_(:,:,i)=imagePoints;
    worldPoints_(:,:,i)=worldPoints;
    imagename = sprintf('%02d', i-1)
    imwrite(I,"./Images/"+imagename+".png")
end
cameraParams = estimateCameraParameters(imagePoints_,worldPoints);
worldPoints = [worldPoints, zeros(size(worldPoints,1),1), ones(size(worldPoints,1),1)]';
  
A = zeros(4,4,s(4)-1);
B = zeros(4,4,s(4)-1);

for i = 1:1:s(4)
    TbE1 = (armMat(:,:,i));
    posename = sprintf('%02d', i-1)
    save("./Poses/"+posename+".txt",'TbE1','-ascii');
end
for i=1:1:s(4)-2
    TbE1 = (armMat(:,:,i));    
    TbE2 = (armMat(:,:,i+1));    
    TH1C = TransInv(pose_R_to_transformation(cameraParams.TranslationVectors(i,:)./1000,cameraParams.RotationMatrices(:,:,i)'));
    TH2C = TransInv(pose_R_to_transformation(cameraParams.TranslationVectors(i+1,:)./1000 ,cameraParams.RotationMatrices(:,:,i+1)'));
    A(:,:,i) = TransInv(TbE2)*(TbE1);
    B(:,:,i) = (TH2C)*TransInv(TH1C);
end
X=calibration_AX_XB(A,B)
cam_pose_list = zeros(4,4,s(4))
for i =1:1:s(4)
    TbE1 = (armMat(:,:,i));
hold on;
TH1C = TransInv(pose_R_to_transformation(cameraParams.TranslationVectors(i,:)./1000,cameraParams.RotationMatrices(:,:,i)'));
cam_pose = TbE1*X*(TH1C);

Tbh = TbE1*X;
drawAxis(Tbh,1,"Tbh"+string(i));
cam_pose_list(:,:,i) = cam_pose
end


clc;
baseToCamTransformation = mean(cam_pose_list,3)
EndEffectorToBoardTransformation =X
view(115,10)


imageFolder = './Images';
squareSize = 15;
[TBaseToCam, TEndToBoard, cameraParams, TBaseStd, TEndStd, pixelErr] = CalCamArm(imageFolder, armMat, squareSize,'maxBaseOffset',1,'gripEst',EndEffectorToBoardTransformation,'baseEst',TransInv(baseToCamTransformation));


