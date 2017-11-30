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

% FILL IN YOUR CODE HERE

k_v = 10; k_p = 10; 
k_vp = 0.1; k_pp = 15;
r_c = des_state.acc + k_v*(des_state.vel - state.vel)+ k_p*(des_state.pos - state.pos);
y_c = r_c(1);
z_c = r_c(2);
u1 = params.mass*(params.gravity + z_c);
phi_c = -y_c/(params.gravity);
e_p = phi_c - state.rot;
e_v = (0 - state.omega);
phi_t = k_pp*(e_p)+ k_vp*(e_v);
phi_t = atan2(sin(phi_t), cos(phi_t));
u2 = phi_t;

end

