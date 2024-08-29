function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;
L1 = 1
L2 = 1

rads2 = acos( (X^2 + Y^2 -L1^2 - L2^2)/(2*L1*L2) )
rads1 = atan(Y/X) - asin(L2/sqrt(X^2 + Y^2)*sin(pi - rads2))