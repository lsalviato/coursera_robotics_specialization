function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
%% Controller coefficients
kp_3 = 5000;
kd_3 = 6;

kp_phi = 10000;
kd_phi = 1;
kp_theta = 10000;
kd_theta = 1;
kp_psi = 10;
kd_psi = 1;

kp_x = 30;
kd_x = 2;
kp_y = 30;
kd_y = 2;
%% Assumptions
p_d = 0;
q_d = 0;
r_d = des_state.yawdot;

m = params.mass;
g = params.gravity;
I = params.I;
%% Position control
% Thrust
u1 = m*(g+des_state.acc(3)) ...
    - m*( kd_3*(state.vel(3)-des_state.vel(3)) ...
    + kp_3*(state.pos(3)-des_state.pos(3)) ); 

% Angles
psi_d   = des_state.yaw;

e_acc_x = -kd_x*( state.vel(1) - des_state.vel(1) ) ...
    - kp_x*( state.pos(1) - des_state.pos(1) );
e_acc_y = -kd_y*( state.vel(2) - des_state.vel(2) ) ...
    - kp_y*( state.pos(2) - des_state.pos(2) );
% e_acc_x = 0;
% e_acc_y = 0;

phi_d   = 1/g*( (des_state.acc(1) + e_acc_x)*sin(psi_d) ...
    - (des_state.acc(2) + e_acc_y)*cos(psi_d));
theta_d = 1/g*( (des_state.acc(1) + e_acc_x)*cos(psi_d) ...
    + (des_state.acc(2) + e_acc_y)*sin(psi_d));
% des_state = r_des vector
% des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot]
%% Attitude control
% Moment
u2(1,1) = kp_phi * (phi_d - state.rot(1)) + ...
    kd_phi * (p_d - state.omega(1));
u2(2,1) = kp_theta*(theta_d-state.rot(2)) + ...
    kd_theta*(q_d - state.omega(2));
u2(3,1) = kp_psi * (psi_d - state.rot(3)) + ...
    kd_psi * (r_d - state.omega(3));
%  state.rot = [phi; theta; psi], state.omega = [p; q; r]
%% Conversion
% Thrust
F = u1;
% Moment
M = u2;

% =================== Your code ends here ===================
end
