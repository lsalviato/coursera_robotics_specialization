function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     -K: size (3 x 3) camera calibration (intrinsics) matrix
%     -C2, R2: the second camera pose
%     -C3, R3: the third camera pose
%     -x1, x2, x3: N × 2 matrices whose row represents correspondence
%                  between the first, second, and third images where 
%                  N is the number of correspondences.
%     -X0: N × 3 matrix whose row represents NON REFINED 3D triangulated points
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 

%% mycode
% NOTE: the Jacobian is calculated only with respect to the points
% In this assignment we do not refine camera poses

X = X0;
for j=1:1 %one iteration is enough for this assigment
    for i = 1:length(X0(:,1)) % for every 3D point perform nonlinear LS
        Xsol = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:)', x2(i,:)', x3(i,:)', X(i,:)' );
        X(i,:) = Xsol';
    end
end
end

%perform one iterative step for a single point
function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
J1 = Jacobian_Triangulation(C1, R1, K, X0);
J2 = Jacobian_Triangulation(C2, R2, K, X0);
J3 = Jacobian_Triangulation(C3, R3, K, X0);
uvw1 = K*R1*(X0-C1);
uvw2 = K*R2*(X0-C2);
uvw3 = K*R3*(X0-C3);
Fx = [ uvw1(1)/uvw1(3), uvw1(2)/uvw1(3), ...
       uvw2(1)/uvw2(3), uvw2(2)/uvw2(3), ...
       uvw3(1)/uvw3(3), uvw3(2)/uvw3(3)]';
b = [x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
J = [J1; J2; J3]; %Jacobian
deltaX = (J'*J)\J'*(b-Fx);
X = X0 + deltaX;
end

%calculate small Jacobian dfdX for 1 point and 1 camera pose
function J = Jacobian_Triangulation(C, R, K, X)
proj = K*R*(X - C);
u = proj(1);
v = proj(2);
w = proj(3);
fx = K(1,1);
fy = K(2,2);
px = K(1,3);
py = K(2,3);
dudX = [fx*R(1,1)+px*R(3,1) fx*R(1,2)+px*R(3,2) fx*R(1,3)+px*R(3,3)];
dvdX = [fy*R(2,1)+py*R(3,1) fy*R(2,2)+py*R(3,2) fy*R(2,3)+py*R(3,3)];
dwdX = [R(3,1) R(3,2) R(3,3)];
J = [ (w*dudX - u*dwdX) / w^2 ;
      (w*dvdX - v*dwdX) / w^2];
end