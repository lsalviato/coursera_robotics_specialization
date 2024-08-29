function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
%logo_pts ~ H*video_pts     xp = H*x

for i=1:4
    x1 = video_pts(i,1);%video point
    x2 = video_pts(i,2);%video point
    x1p = logo_pts(i,1);%logo point
    x2p = logo_pts(i,2);%logo point
    
    ax(i,:) = [-x1 -x2 -1  0   0   0  x1*x1p  x2*x1p  x1p];
    ay(i,:) = [0    0   0 -x1 -x2 -1  x1*x2p  x2*x2p  x2p];
end

A = [ax(1,:) ; ay(1,:);
     ax(2,:) ; ay(2,:);
     ax(3,:) ; ay(3,:);
     ax(4,:) ; ay(4,:)];

[U, S, V] = svd(A);
h = V(:,end);
H = reshape(h,3,3)';
%H = H ./ norm(H(:,1))
end

