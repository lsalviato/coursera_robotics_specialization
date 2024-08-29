function [endeff] = computeMiniForwardKinematics(rads1,rads2)

%rads1 = 3.5
%rads2 = 1.5

l1 = 1
l2 = 2

alpha = 0.5*(rads1 + rads2) + pi
beta = 0.5*(rads1 + rads2)

b2 = beta;

p1= [l1*cos(rads1) , l1*sin(rads1)]
p2= [l1*cos(rads2) , l1*sin(rads2)]

dist = sqrt( (p1(1) - p2(1))^2 + (p1(2) - p2(2))^2 )

k = dist/2;
R = sqrt( l2^2 - k^2) - sqrt( l1^2 - k^2 )

endeff = [R*cos(alpha), R*sin(alpha)];