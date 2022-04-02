% Set up the state space model of the system
function [x_dot] = SSM(x, Train, F)
x_dot = Train.A * x + Train.B * F;
end
