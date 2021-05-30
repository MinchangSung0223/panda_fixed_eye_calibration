
function T = pose_R_to_transformation(t,R)
T = eye(4);
T(1:3,1:3) = R;
T(1,4) =t(1);
T(2,4) =t(2);
T(3,4) =t(3);

end
