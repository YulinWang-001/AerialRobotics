function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

g= params.gravity ;
u_max = params.u_max;
u_min = params.u_min;
m = params.mass;
l = params.arm_length;

u = 0;

% FILL IN YOUR CODE HERE
kv = 20;
kp = 400;
z_dot = 0;
e = s_des - s;
u = m * (z_dot+kp*e(1)+kv*e(2)+g); % u = z_dot + kp*e(1)+kv*e(2)+m*g
% In order to make u convergent 
if u > u_max
    u = u_max;
elseif u < u_min
    u = u_min;
end 

end

