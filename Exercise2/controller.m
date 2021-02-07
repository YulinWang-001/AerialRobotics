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

% FILL IN YOUR CODE HERE
% Define all the parameters 
Kp_z = 400; % Set initial values of derivative and proportional 
Kv_z = 30;
Kp_y = 15;
Kv_y = 20;
Kp_phai = 900;
Kv_phai = 50;
e = des_state.pos - state.pos;% The Error Vector
e_dot = des_state.vel - state.vel;
% Define the parameters from the input of function 

y_ddot = des_state.acc(1);
z_ddot = des_state.acc(2);
phai = state.rot(1);
phai_dot = state.omega(1);
% Apply the Hover Controller Inputs Equation 
% Angle controller 
phai_c = -(1/g)*(y_ddot+Kv_y*e_dot(1)+Kp_y*e(1));
phai_dotc = 0;
% Position Controller 
u1 = m*(g+z_ddot+Kv_z*e_dot(2)+Kp_z*e(2));
u2 = Ixx*(Kv_phai*(phai_dotc-phai_dot)+Kp_phai*(phai_c-phai));
end

