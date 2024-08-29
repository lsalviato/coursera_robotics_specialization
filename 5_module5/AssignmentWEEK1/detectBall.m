% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
load mu 
load Sigma
thre = 0.95

mu = double(mu');
Sigma = double(Sigma);
I = double(I);
D = length(mu);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
[ii,jj,kk] = size(I);
for i=1:ii
    for j=1:jj
        x = double ( squeeze( I(i,j,:) ) );
        coeff = (2*pi)^(D/2) * sqrt(det(Sigma));
        coeff = 1/coeff;
        expon = -1/2 * ( (x-mu)' * inv(Sigma) * (x-mu) );
        y(i,j) = coeff * exp(expon);
    end
end

seg = y > 2e-5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

segI = seg
stats = regionprops(segI, 'Centroid');
centroid = stats.Centroid;

loc = centroid
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
