% Sample script to demonstrate execution of function [TBaseToCam, TEndToBoard] = start_calibration(imageFolder, poseFolder, squareSize, board_size, show_image)
addpath('functions')
imageFolder = 'Images'; % Initialize imageFolder here
poseFolder = 'Poses'; % Initialize poseFolder here
squareSize = 15; % Initialize squareSize here
board_size = [8,11]; % Initialize board_size here
show_image = 1; % Initialize show_image here
[TBaseToCam, TEndToBoard] = start_calibration(imageFolder, poseFolder, squareSize, board_size, show_image);
