function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

%% mycode
X = [X ones( length(X(:,1)), 1)];
A = [];
for i=1:length(x(:,1))
    %calculate calibrated u and v (for numerical stability)
    temp = K\[x(i,:)';1];
    u = temp(1)/temp(3);
    v = temp(2)/temp(3);
    
    % calculate LS matrix A where Ax=0
    z4 = zeros (1,4);
    Xmat = [X(i,:)   z4     z4
              z4    X(i,:)  z4
              z4     z4    X(i,:)];
    Asmall = Vec2Skew([u,v,1]) * Xmat;
    % Asmall = [ z4      -X(i,:)    v*X(i,:)
    %            X(i,:)     z4     -u*X(i,:)
    %          -v*X(i,:)  u*X(i,:)     z4   ];
    Asmall = Asmall(1:2 , :);%the rows are linearly dependent so use 2 rows
    A = [A; Asmall];
end
% Since I calibrated the points temp = inv(K)*x_small,
% I find the calibrated P = eye(3)*[R t] = [R t]
% In Ax=0, x = [P1 P2 P3]' P_i = row i of projection matrix P = [R t]
% [u v 1]' = [R t]*X

%solve least squares for P
[U,S,V] = svd(A);
lastcolV = V(:,end);
P = reshape(lastcolV, 4, 3)';

% extract rotation and translation
R = P(1:3,1:3);
t = P(:,4);

% CLEANUP of R and t
[UU,SS,VV] = svd(R);
% R must have a positive determinant since it's a rotation matrix
if det(UU*VV') > 0
    R = UU * VV';
    % P is defined up to a scale
    % since we rescaled R we must rescale t as well
    t = t / SS(1,1); 
else
    R = -UU * VV';
    % P is defined up to a scale
    % since we rescaled R we must rescale t as well
    t = -t / SS(1,1);
end

% calculate camera translation in world coordinates (CAMERA 1 FRAME)
C = -R' * t;



