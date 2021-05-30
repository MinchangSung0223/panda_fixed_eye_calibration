close all;
clear
format shortG
load data.mat
s = size(image)
imagePoints_list = {}
worldPoints_list = {}
squareSize = 15%mm
board_size =[8,11]
imagePoints_=zeros(board_size(1)*board_size(2),2,s(4));
worldPoints_=zeros(board_size(1)*board_size(2),2,s(4));

for i =1:1:s(4)
    I = image(:,:,1:3,i);
    [imagePoints,boardSize] = detectCheckerboardPoints(I);
    boardSize
    imagePoints_list{end+1} = imagePoints;
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    worldPoints_list{end+1} = worldPoints-repmat([75,105/2],88,1);
    imageSize = [size(I, 1),size(I, 2)];
    
    imagePoints_(:,:,i)=imagePoints;
    worldPoints_(:,:,i)=worldPoints;
    
%      figure(i)
%      imwrite(I,"./Images/"+string(i)+".png")
%      imshow(I)
%      hold on;
%      plot(imagePoints(:,1),imagePoints(:,2),'ro');
%      pause(0.5);
end

params = estimateCameraParameters(imagePoints_,worldPoints);


% 3x1 = 3x3 * 3x1
% u= [A B C]   [x]
% v= [D E F] * [y]
% 1= [G H 1]   [1]

% (2*6*9 )x1 = (2*6*9  x 8 ) * 8x1
% [ u1  ] = [ x1  y1  1  0   0   0  -u1*x1  -u1*y1 ] * [A]
% [ u2  ] = [ x2  y2  1  0   0   0  -u2*x2  -u2*y2 ]   [B]
% [ u3  ] = [ x3  y3  1  0   0   0  -u3*x3  -u3*y3 ]   [C]
% [ u4  ] = [ x4  y4  1  0   0   0  -u4*x4  -u4*y4 ]   [D]
% [ ... ]   [ ...                                  ]   [E]
% [ un  ] = [ xn  yn  1  0   0   0  -un*xn  -un*yn ]   [F]
% [ v1  ] = [ 0   0   0  x1  y1  1  -v1*x1  -v1*y1 ]   [G]
% [ v2  ] = [ 0   0   0  x2  y2  1  -v2*x2  -v2*y2 ]   [H]
% [ ... ]   [ ...                                  ]   
% [ vn  ] = [ 0   0   0  xn  yn  1  -vn*xn  -vn*yn ]   


% inv(2*6*9  x 8 ) * (2*6*9 )x1 =  8x1
% find Homography


numImages = s(4);
homographies = zeros(3, 3, numImages);
for i = 1:1:numImages
     homographies(:, :, i) = computeHomography(double(imagePoints_list{i}), worldPoints);
end
V = computeV(homographies);
B = computeB(V);
A = computeIntrinsics(B);



%% Functions

function [ptsNorm, normMatrixInv] = normalizeControlPoints(pts)
N = size(pts,1);
% Compute [xCentroid,yCentroid]
cent = mean(pts, 1);
% Shift centroid of the input points to the origin.
%   ptsNorm(:, 1) = pts(:, 1) - cent(1);
%   ptsNorm(:, 2) = pts(:, 2) - cent(2);
ptsNorm = bsxfun(@minus,pts,cent);
sumOfPointDistancesFromOriginSquared = sum( hypot(ptsNorm(:,1),ptsNorm(:,2)).^2 );
if sumOfPointDistancesFromOriginSquared > 0
    scaleFactor = sqrt(2*N) / sqrt(sumOfPointDistancesFromOriginSquared);
else
    if isa(pts,'single')
        scaleFactor = single(1);
    else
        scaleFactor = 1.0;
    end
end
ptsNorm = ptsNorm .* scaleFactor;
normMatrixInv = [...
    1/scaleFactor,     0,            0;...
    0,            1/scaleFactor,     0;...
    cent(1),      cent(2),      1];
end


function H = computeHomography(imagePoints, worldPoints)
% Compute projective transformation from worldPoints to imagePoints

[uv,normMatrix1] = normalizeControlPoints(worldPoints);
[xy,normMatrix2] = normalizeControlPoints(imagePoints);

minRequiredNonCollinearPairs = 4;
M = size(xy,1);
x = xy(:,1);
y = xy(:,2);
vec_1 = ones(M,1);
vec_0 = zeros(M,1);
u = uv(:,1);
v = uv(:,2);

U = [u; v];

X = [x      y      vec_1  vec_0  vec_0  vec_0  -u.*x  -u.*y;
     vec_0  vec_0  vec_0  x      y      vec_1  -v.*x  -v.*y  ];

Tvec = X \ U;    
Tvec(9) = 1;

Tinv = reshape(Tvec,3,3);

Tinv = normMatrix2 \ (Tinv * normMatrix1);

T = inv(Tinv);
T = T ./ T(3,3);

H = T';
H = H / H(3,3);
end

function V = computeV(homographies)
% Vb = 0

numImages = size(homographies, 3);
V = zeros(2 * numImages, 6);
for i = 1:numImages
    H = homographies(:, :, i)';
    V(i*2-1,:) = computeLittleV(H, 1, 2);
    V(i*2, :) = computeLittleV(H, 1, 1) - computeLittleV(H, 2, 2);
end
end
%--------------------------------------------------------------------------
function v = computeLittleV(H, i, j)
    v = [H(i,1)*H(j,1), H(i,1)*H(j,2)+H(i,2)*H(j,1), H(i,2)*H(j,2),...
         H(i,3)*H(j,1)+H(i,1)*H(j,3), H(i,3)*H(j,2)+H(i,2)*H(j,3), H(i,3)*H(j,3)];
end

function B = computeB(V)
% lambda * B = inv(A)' * inv(A), where A is the intrinsic matrix

[~, ~, U] = svd(V);
b = U(:, end);

% b = [B11, B12, B22, B13, B23, B33]
B = [b(1), b(2), b(4); b(2), b(3), b(5); b(4), b(5), b(6)];
end


function A = computeIntrinsics(B)
% Compute the intrinsic matrix

cy = (B(1,2)*B(1,3) - B(1,1)*B(2,3)) / (B(1,1)*B(2,2)-B(1,2)^2);
lambda = B(3,3) - (B(1,3)^2 + cy * (B(1,2)*B(1,3) - B(1,1)*B(2,3))) / B(1,1);
fx = sqrt(lambda / B(1,1));
fy = sqrt(lambda * B(1,1) / (B(1,1) * B(2,2) - B(1,2)^2));
skew = -B(1,2) * fx^2 * fy / lambda;
cx = skew * cy / fx - B(1,3) * fx^2 / lambda;
A = vision.internal.calibration.constructIntrinsicMatrix(fx, fy, cx, cy, skew);
if ~isreal(A)
    error(message('vision:calibrate:complexCameraMatrix'));
end
end
