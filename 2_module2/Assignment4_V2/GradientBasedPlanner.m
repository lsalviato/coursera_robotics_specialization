function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
route = start_coords;
curr_pos = start_coords;

for i = 1:max_its
    rounded_pos = round(curr_pos)
    gx_interp = interp2(gx, rounded_pos(1), rounded_pos(2));
    gy_interp = interp2(gy, rounded_pos(1), rounded_pos(2));
    grad = [ gx_interp , gy_interp ];
    grad_vers = grad / norm(grad);
    
    next_pos = curr_pos + grad_vers ;   
    %round(next_pos);
    route = [route ; next_pos];
    curr_pos = next_pos;
    
    if norm( curr_pos - end_coords) < 2
        route = double(route)
        return
    end
    
end
route = double(route)
% *******************************************************************
end
