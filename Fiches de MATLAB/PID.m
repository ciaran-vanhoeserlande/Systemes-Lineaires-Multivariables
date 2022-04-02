% Design a simple PID controller
function [i, d, output] = PID(K_p, K_i, K_d, error, i_prev, d_prev, error_prev, dt)
tau = dt;
p = K_p * error;
i = K_i * dt / 2 * (error + error_prev) + i_prev;
d = 2 * K_d / (2 * tau + dt) * (error - error_prev) + (2 * tau - dt) / (2 * tau + dt) * d_prev;
output = p + i + d;
end
