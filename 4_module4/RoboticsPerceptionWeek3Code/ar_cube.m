function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
H_first = H;
H_first_orth = [H_first(:,1) H_first(:,2) cross(H_first(:,1), H_first(:,2)) ];
[U,~,V] = svd(H_first_orth);
R = U * diag([1 1 det(U*V')]) * V'
t = H(:,3) / norm( H(:,1) )

% YOUR CODE HERE: Project the points using the pose
render_points_homog = [render_points ones(length(render_points),1) ]
proj_points = (K*[R t] * render_points_homog')'
proj_points = proj_points./proj_points(:,3)
end
