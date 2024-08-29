% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
myResol = param.resol;
% the initial myMap size in pixels
myMap = zeros(param.size);
% the origin of the myMap in pixels
myOrigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
n_Angles = size(scanAngles);
for j = 1:N % for each time,
    display(j)
    x = pose(1, j);
    y = pose(2, j);
    theta = pose(3, j);

    %get d vector
    d_vec = ranges(:, j);
    %find obstacle positions
    x_occ = d_vec .* cos(scanAngles + theta) + x;
    y_occ = -d_vec .* sin(scanAngles + theta) + y;
    %find obstacle indexes
    ix_occ = ceil(x_occ * myResol) + myOrigin(1);
    iy_occ = ceil(y_occ * myResol) + myOrigin(2);

    %find robot index
    ix_bot = ceil(x * myResol) + myOrigin(1);
    iy_bot = ceil(y * myResol) + myOrigin(2);

    % Find occupied-measurement cells and free-measurement cells
    occ = sub2ind(size(myMap), iy_occ, ix_occ); % Convert to 1d

    free = [];
    for obst = 1:n_Angles %for every angle-obstacle
        [ix_free, iy_free] = bresenham(ix_bot, iy_bot, ix_occ(obst), iy_occ(obst));  
        free = [free; iy_free, ix_free];
    end

    free = unique(free,'rows');
    free = sub2ind(size(myMap), free(:, 1), free(:, 2));

    % Update the log-odds
    myMap(occ) = myMap(occ) + lo_occ;
    myMap(free) = myMap(free) - lo_free;

    % Saturate the log-odd values
    myMap(myMap > lo_max) = lo_max;
    myMap(myMap < lo_min) = lo_min;

end

end