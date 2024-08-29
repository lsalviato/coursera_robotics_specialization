function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

g = params.gravity;
m = params.mass;
Ixx = params.Ixx;

p = state.pos;
v = state.vel;
phi = state.rot;
phi_dot = state.omega;%= [phi_dot]

pdes = des_state.pos;
vdes = des_state.vel;
accdes = des_state.acc;

kv_z = 100;
kp_z = 59.9996;
kp_phi = 8000;
rat = 3000; % W_zero_PD / w_cross
kv_phi = sqrt((0.00025*rat^2*kp_phi)/sqrt(1+rat^2));
kv_y = 10;
kp_y = 0.3;

u1 = m*( g + accdes(2) + kv_z*(vdes(2)-v(2)) + kp_z*(pdes(2)-p(2)) );%ok

phi_c =  -1/g*( accdes(1) + kv_y*(vdes(1)-v(1)) + kp_y*(pdes(1)-p(1)) );

u2 = Ixx*(0 + kv_phi*(-phi_dot) + kp_phi*(phi_c - phi) );


end

