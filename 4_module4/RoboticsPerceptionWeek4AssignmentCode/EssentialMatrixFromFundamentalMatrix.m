function E = EssentialMatrixFromFundamentalMatrix(F,K)
%% EssentialMatrixFromFundamentalMatrix
% Use the camera calibration matrix to esimate the Essential matrix
% Inputs:
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     F - size (3 x 3) fundamental matrix from EstimateFundamentalMatrix
% Outputs:
%     E - size (3 x 3) Essential matrix with singular values (1,1,0)

%% mycode
% USe definition
E = K' * F * K;

% CLEANUP: ensure singular values of E are 1
[U,S,V] = svd(E);
M = [1 0 0;
     0 1 0;
     0 0 0];
E = U * M * V';
E = E ./ norm(E); %normalize: E is defined up to a scale