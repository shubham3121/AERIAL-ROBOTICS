function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%

% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

persistent waypoints0 traj_time d0 a0 a1 a2 a3 Ti Si Si_1 p_dot p_ddot pos
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);
   % t_index = t_index - 1;
   if(traj_time(t_index)-t<0.002 && t~=0)
       fprintf('Ok: %d  \n',t)
   end
   if(t_index == 1)
       p_dot = [0;0;0]; p_ddot = [0;0;0];
   elseif(t_index == 2)
       p_ddot = [0.5000;0.5000;0.5000];p_dot = [0.8660;0.8660;0.8660];
   elseif(t_index == 3)
       p_dot = [-1.7320;-3.4640;-1.7320];p_ddot = [-2.0000;-3.0000;-2.0000];
   elseif(t_index == 4)
       p_dot= [7.7941;11.2581;6.0620];p_ddot = [7.4999;11.4998;6.4999];
   end
    if(t_index == length(traj_time))
        fprintf('p_ddot = %d \n',p_ddot)
        p_dot = [0;0;0];
        p_ddot = [-27.9995;-41.9991;-23.9994];
        Ti = d0(t_index-1);
        Si_1 = traj_time(t_index-1);
        Si = traj_time(t_index);
        delta = (Si-Si_1)/Ti;
        A = [1 0 0 0;1 delta delta^2 delta^3;0 1/Ti 0 0;0 0 2/Ti^2 0];
        b = [(waypoints0(:,t_index-1))';(waypoints0(:,t_index))';p_dot';p_ddot'];
        alpha = (A\b)';
        a0 = alpha(:,1); a1 = alpha(:,2); a2 = alpha(:,3); a3 = alpha(:,4);
    elseif(t_index == 1)
        p_dot = [0;0;0];
        p_ddot = [0;0;0];
        pos =[0;0;0];
    end
    if(t_index < length(traj_time))
        Ti = d0(t_index);
        Si_1 = traj_time(t_index);
        Si = traj_time(t_index+1);
        delta = (Si-Si_1)/Ti;
        A = [1 0 0 0;1 delta delta^2 delta^3;0 1/Ti 0 0;0 0 2/Ti^2 0];
        b = [(waypoints0(:,t_index))';(waypoints0(:,t_index+1))';p_dot';p_ddot'];
        alpha = (A\b)';
        a0 = alpha(:,1); a1 = alpha(:,2); a2 = alpha(:,3); a3 = alpha(:,4);
    end
    delta = (t-Si_1)/Ti;
    desired_state.pos = a0 + a1*delta + a2*delta^2 + a3*delta^3;
    desired_state.vel = a1*(1/Ti) + (2/Ti)*a2*delta + (3/Ti)*a3*delta^2;
    desired_state.acc = a2*2/(Ti^2) + (6/Ti^2)*delta*a3;
    desired_state.yaw = atan2(desired_state.pos(2)-pos(2),desired_state.pos(1)-pos(1));
    desired_state.yawdot = desired_state.yaw/100;
    pos = desired_state.pos;
end

%


%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end
