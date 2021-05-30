

function T = calibration_AX_XB(A,B)
    T = eye(4);
    N = size(A,3);
    M =zeros(3,3);
    for i = 1:1:N
        Ra = A(1:3,1:3,i);
        Rb = B(1:3,1:3,i);
        M = M+(so3ToVec(MatrixLog3(Rb))*so3ToVec(MatrixLog3(Ra))')
    end
    Rx = sqrtm(inv(M'*M))*M';
    C = zeros(3*N,3);
    d = zeros(3*N,1);
    for i = 1:1:N
        Ra = A(1:3,1:3,i);
        ta = A(1:3,4,i);
        Rb = B(1:3,1:3,i);
        tb = B(1:3,4,i);
        C(3*i:3*i+2,:) = eye(3)-Ra;
        d(3*i:3*i+2,1) = ta-Rx*tb;
    end
    tx = (pinv(C)*d);
    T(1:3,1:3) = Rx;
    T(1:3,4) = tx;
end