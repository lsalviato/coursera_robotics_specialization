%  Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% number of poses to be estimated
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
% initialize poses for speed
% myPose = zeros(3, N); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 

% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%use as few as possible for speed
Np = 200;       % Please decide a reasonable number of M, 
                % based on your experiment using the practice data.

% define thresholds for correlation score
map_thres_low  = mode(map,'all') - 0.2;
map_thres_high = mode(map,'all') + 0.2;
%resample_threshold = 0.7;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create Np number of particles -> P
P = repmat(myPose(:,1), [1, Np]);
weights = repmat(1.0/Np, [1, Np]); %init weights for each particle (pose)

sigma_x = 0.3;
sigma_y = sigma_x; 
sigma_theta = 7*pi/180; 
% noise to be added to the particles at every iteration
noise = [sigma_x^2 * randn(1,Np)  ;
         sigma_y^2 * randn(1,Np)  ;
         sigma_theta^2 * randn(1,Np)  ];

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    % 1) Propagate the particles 
    P = repmat(myPose(:,j-1), [1, Np]);
    P = P + noise;

    % at ever iteration reset the weights
    weights = repmat(1.0/Np, [1, Np]);    
    % reset correlation between observations and map
    P_correl = zeros(1, Np); 
    
    % 2) Measurement Update 
    for i = 1:Np
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)
        occ_cells(:,1) = ceil(( ranges(:,j).*cos(scanAngles + P(3,i)) + P(1,i))*myResolution + myOrigin(1));
        occ_cells(:,2) = ceil((-ranges(:,j).*sin(scanAngles + P(3,i)) + P(2,i))*myResolution + myOrigin(2));

    %   2-2) For each particle, calculate the correlation scores of the particles
        % discard out of range positions
        occ_cells = occ_cells( occ_cells(:,1)>1 , : );
        occ_cells = occ_cells( occ_cells(:,2)>1 , : );
        occ_cells = occ_cells( occ_cells(:,1)<length(map(1,:)) , : );
        occ_cells = occ_cells( occ_cells(:,2)<length(map(2,:)) , : );

        occ_cells_idx = sub2ind(size(map), occ_cells(:,2), occ_cells(:,1));
        occ_cells_list = map(occ_cells_idx);

        %calculate correlation score for every particle for this measurement
        P_correl(i) = - 5 * sum(occ_cells_list <= map_thres_low) ...
                      + 10 * sum(occ_cells_list >= map_thres_high);
        clear('occ_cells');
    end
    %rescale correlation
    P_correl = P_correl - min(P_correl);
    
    %   2-3) Update the particle weights
    weights = (weights .* P_correl) / sum(P_correl);
    weights = weights / sum(weights);
    
    %   2-4) Choose the best particle to update the pose
    % pick pose with biggest weight
    [ ~, idx ] = max(weights);
    myPose(:,j) = P(:,idx);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
    %n_eff_part = sum(weights)^2/sum(weights.^2);
    
disp(j)
end

end