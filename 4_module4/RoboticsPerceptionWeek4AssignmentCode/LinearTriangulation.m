function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

%% mycode
%construct Projection matrices
% P = K*[R t] = K*[R -R*C]; with C = -R'*t
P1 = K*[R1 -R1*C1]; %first camera = world frame
P2 = K*[R2 -R2*C2]; %second camera

%construct matrix A where Ax=0 for a single 3D point
A = [];
for i=1:size(x1(:,1)) %for all points
    %construct cross product matrices for a single 3D point
    tmp = [x1(i,1) x1(i,2) 1]; %temp
    
    tcross = [ 0    -tmp(3)  tmp(2)
               tmp(3)  0    -tmp(1)
              -tmp(2)  tmp(1)  0   ];
    x1cross = tcross;

    tmp = [x2(i,1) x2(i,2) 1]; %temp
    tcross = [ 0     -tmp(3)  tmp(2)
               tmp(3)  0    -tmp(1)
              -tmp(2)  tmp(1)  0   ];
    x2cross = tcross;

    A = [x1cross * P1 ;
         x2cross * P2];

    % solve LeastSquares problem
    [U,S,V] = svd(A);
    sol = V(:,end);

    % homog coord in 3D have 4th elements equal to 1
    sol = sol ./ sol(4);

    % make sure Z coord is positive (point in front of camera)
    X_with_z_pos = sol(1:3)  * sign( sol(3));

    % store triangulated point
    X(i,:) = X_with_z_pos; 
end






