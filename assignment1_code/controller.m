function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters


% FILL IN YOUR CODE HERE
K_p = 350; K_v = 35;
e = s_des - s;
u = params.mass*(s_des(1) + K_p*e(1) + K_v*e(2) + params.gravity);
u = min(params.u_max, u);
end

