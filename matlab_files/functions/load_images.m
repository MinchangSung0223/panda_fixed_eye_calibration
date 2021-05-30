function [imagePoints_,worldPoints]=load_images(imageFolder,squareSize,board_size)
imds = imageDatastore(imageFolder);
image_num = size(imds.Files,1);
imagePoints_=zeros(board_size(1)*board_size(2),2,image_num);
for i = 1:1:image_num
    I = imread(imds.Files{i});
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    imagePoints_(:,:,i)=imagePoints;
end
end