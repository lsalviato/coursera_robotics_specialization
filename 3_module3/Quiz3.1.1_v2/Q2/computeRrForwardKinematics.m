function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]
l1=1;
l2=1;

elbow = [l1*cos(rads1), l1*sin(rads1)]
endeff = [elbow(1) + l2*cos(rads1 + rads2), elbow(2) + l2*sin(rads1 + rads2)]
