function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

%% my code

% construct A
A = [];
for i=1:length(x1(:,1))
    u1 = x1(i,1);
    v1 = x1(i,2);
    u2 = x2(i,1);
    v2 = x2(i,2);
    r = [u1*u2 u1*v2 u1 v1*u2 v1*v2 v1 u2 v2 1];
    A = [A;r];
end % A has been constructed

% Solve LS problem Ax=0, x are the elements fij of fundamental matrix F
[U,S,V] = svd(A);
x = V(:,9);
F = reshape(x,3,3)'; 

% Ensure F has rank 2
[U2,S2,V2] = svd(F);
S2(3,3) = 0;
Frank2 = U2*S2*V2';
F = Frank2;
F = F ./ norm(F); %normalize: F is defined up to a scale



