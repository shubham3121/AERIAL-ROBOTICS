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

    Kd = 0.8; Kp = 2; 
% Thrust
    t_cap = des_state.vel/norm(des_state.vel);
    n_cap = t_cap./t;
    b_cap = cross(t_cap,n_cap);
    e_p = (dot((des_state.pos - state.pos),n_cap))*n_cap + (dot((des_state.pos - state.pos),b_cap))*b_cap;
    e_v = des_state.vel - state.vel;
    
    rddot_c = des_state.acc + Kd*e_v + Kp*e_p;
    F = params.mass*(params.gravity + rddot_c(3));
    
% Moment
    phi_c = (1/params.gravity)*(rddot_c(1)*sin(des_state.yaw) - rddot_c(2)*cos(des_state.yaw));
    theta_c = (1/params.gravity)*(rddot_c(2)*cos(des_state.yaw) - rddot_c(2)*sin(des_state.yaw));
    yaw_c = des_state.yaw;
        
    kdx = 0.9; kpx = 5;
    kdy = 0.8; kpy = 5;
    kdz = 1; kpz = 5;

    u2_x = kpx*(phi_c - state.rot(1))- kdx*(state.omega(1));
    u2_y = kpy*(theta_c - state.rot(2)) - kdy*(state.omega(2));
    u2_z = kpz*(yaw_c - state.rot(3))+ kdz*(des_state.yawdot - state.omega(3));
    
    M = (params.invI)*[u2_x;u2_y;u2_z];
    
% =================== Your code ends here ===================
end
