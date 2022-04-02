% Clean up the command window, erase variables and arrays, and close all
% open figures from previous runs
clc
clear
close all

% Declare global variables
global Train
global x
global R
global flag

% Initialize the state space model of the small freight train
Initialize()

%% ------------------------------Question 10------------------------------
% Calculate the eigenvalues and -vectors of A
[eigenvectors_A, eigenvalues_A] = eig(Train.A);

%% ------------------------------Question 11------------------------------
% Calculate the transfer function
C = [0 0 0 1 0 0];
[numerator, denominator] = ss2tf(Train.A, Train.B, C, Train.D);
H = tf(numerator, denominator);

%% ------------------------------Question 12------------------------------
% Calculate the observability matrix
Ob = obsv(Train.A, Train.C);
% Check whether the system is observable using the Kalman criterium
if rank(Ob) == 6
    disp('The system is observable.')
else
    disp('The system is not observable.')
end

%% ------------------------------Question 13------------------------------
% Calculate the controllability matrix
Co = ctrb(Train.A,Train.B);
% Check whether the system is controllable using the Kalman criterium
if rank(Co) == 6
    disp('The system is fully controllable.')
else
    disp('The system is not fully controllable.')
end

%% ------------------------------Question 14------------------------------
% % Simulate the behavior of the train when a force of 200 Newton is applied
% % to the locomotive (comment out the code below when running the code under
% % either question 15, 17, or 19 & 20 is desired)
% % Set the initial values of the variables of the small freight train
% Train.F = 200;
% x = zeros(6,1);
% % Set the amount of time in seconds the simulation is to be run
% T = 10;
% % Set the interval in seconds at which each datapoint is to be plotted
% dt = 0.01;
% % Declare a uniform time vector 
% tv = 0:dt:T;
% % Calculate the different states
% [t_new, x_new] = ode45(@(t,x) SSM(x, Train, Train.F), tv, x);
% % Plot the elongation of the springs
% subplot(2, 1, 1);
% plot(t_new, x_new(:,2:3));
% xlabel('Time (s)');
% ylabel('Elongation (m)');
% legend("Spring 1", "Spring 2");
% title('Elongation of Springs');
% % Plot the velocity of the train as a whole
% subplot(2, 1, 2);
% plot(t_new, x_new(:,4));
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Velocity of Train');

%% ------------------------------Question 15------------------------------
% % Perform a 'release' experiment to verify the accuracy of the state
% % feedback (comment out the code below when running the code under either
% % question 14, 17, or 19 & 20 is desired)
% % Set the initial values of the variables of the small freight train
% Train.F = 0;
% x_2_0 = 0.1;
% x_3_0 = -0.1;
% x = [0;
%      x_2_0;
%      x_3_0;
%      0;
%      0;
%      0];
% % Set the amount of time in seconds the simulation is to be run
% T = 10;
% % Set the interval in seconds at which each datapoint is to be plotted
% dt = 0.01;
% % Declare a uniform time vector 
% tv = 0:dt:T;
% % Calculate the different states
% [t_new, x_new] = ode45(@(t,x) SSM(x, Train, Train.F), tv, x);
% % Plot the elongation of the springs
% subplot(2, 1, 1);
% plot(t_new, x_new(:,2:3));
% xlabel('Time (s)');
% ylabel('Elongation (m)');
% legend("Spring 1", "Spring 2");
% title('Elongation of Springs');
% % Plot the velocity of the train as a whole
% subplot(2, 1, 2);
% plot(t_new, x_new(:,4));
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Velocity of Train');

%% ------------------------------Question 16------------------------------
% Realize the synthesis of a state feedback controlled system through pole
% placement
% Choose desired poles
P_Des = [-2 -2.4 -2.6 -2.8 -3 -3.2];
% Calculate the state feedback matrix R
R = place(Train.A, Train.B, P_Des);
% Calculate the eigenvalues of R
eigenvalues_R = eig(Train.A - (Train.B * R))';

%% ------------------------------Question 17------------------------------
% % Re-perform the 'release' experiment under control, this time implementing
% % the state feedback matrix found in question 16 (comment out the code
% % below when running the code under either question 14, 15, or 19 & 20 is 
% % desired)
% % Set the initial values of the variables of the small freight train
% Train.F = 0;
% x_2_0 = 0.1;
% x_3_0 = -0.1;
% x = [0;
%      x_2_0;
%      x_3_0;
%      0;
%      0;
%      0];
% % Set the amount of time in seconds the simulation is to be run
% T = 10;
% % Set the interval in seconds at which each datapoint is to be plotted
% dt = 0.01;
% % Declare a uniform time vector 
% tv = 0:dt:T;
% % Calculate the different states
% [t_new, x_new] = ode45(@(t,x) SSM(x, Train, -(R * x)), tv, x);
% % Plot the elongation of the springs
% subplot(3, 1, 1);
% plot(t_new, x_new(:,2:3));
% xlabel('Time (s)');
% ylabel('Elongation (m)');
% legend("Spring 1", "Spring 2");
% title('Elongation of Springs');
% % Plot the velocity of the train as a whole
% subplot(3, 1, 2);
% plot(t_new, x_new(:,4));
% xlabel('Time (s)');
% ylabel('Velocity (m/s)');
% title('Velocity of Train');
% % Calculate the produced force
% F_new = -(R * x_new');
% % Plot the produced force
% subplot(3, 1, 3);
% plot(t_new, F_new);
% xlabel('Time (s)');
% ylabel('Force (N)');
% title('Produced Force');

%% ------------------------------Question 18------------------------------
% Realize the synthesis of an observer from the measurement of the position
% of the locomotive
% Set a sampling period in seconds
T_e = 0.05;
% Calculate the new matrices resulting from the transition from a
% continuous to a discrete time system
A_D = expm(Train.A * T_e);
arg_B = @(mu) expm(Train.A * mu) * Train.B;
B_D = integral(arg_B, 0 , T_e, 'ArrayValued', true);
C_D = Train.C;
D_D = Train.D;
% Convert the desired poles chosen in question 16 to their discrete
% equivalents
i = 1;
while i <= length(P_Des)
    Z_Des(i) = exp(T_e * (4 * P_Des(i)));
    i = i + 1;
end
% Calculate the discrete pole placement matrix 
K_O = place(A_D', C_D', Z_Des)';
% Determine the differential equation of the observer
x_obs = zeros(6,1);
x_obs == (A_D - K_O * C_D) * x_obs + [B_D K_O] * [Train.F; C_D * x];

%% ------------------------------Question 19------------------------------
% Set up the speed control of the train
% Set the amount of time in seconds the simulation is to be run
T = 20;
% Set the interval in seconds at which each datapoint is to be plotted
dt = 0.05;
% Declare a uniform time vector 
tv = dt:dt:T;
% Initiate the backup matrices
x_new_save = zeros(T / dt, 6);
x_obs_save = zeros(T / dt, 6);
% Set a braking coefficient
bk = -3;
% Set the coefficients and variables for the PID controller
K_p = 25;
K_i = 35;
K_d = 3/2;
i = 0;
d = 0;
error = 0;
i_prev = 0;
d_prev = 0;
error_prev = 0;

%% ------------------------------Question 20------------------------------
% Simulate the behavior of the train at a specified speed
% Set a desired speed
V_Des = 10;
% Set the initial values of the auxiliary variables
t = 0;
n = 1;
V_Desired = zeros(T / dt, 1);
% Loop to continually update the different states
while (t < T)
    % Calculate the different states
    [t_new, x_new] = ode45(@(t,x) SSM(x, Train, Train.F), [0 dt], x);
    x = x_new(end, :)';
    clf
    Drawing(x, Train, flag)
    pause(dt);
    % Calculate the observer matrix
    x_obs = (A_D - K_O * C_D) * x_obs + [B_D K_O] * [Train.F; C_D * x];
    % Save the states of both the state and obeserver matrices, x and
    % x_obs respectively, over time
    x_new_save(n, :) = x;
    x_obs_save(n, :) = x_obs;
    % Calculate the speed error
    error = V_Des - x_obs(4);
    % Implement a PID controller to regulate the speed
    [i, d, output] = PID(K_p, K_i, K_d, error, i_prev, d_prev, error_prev, dt);
    i_prev = i;
    d_prev = d;
    error_prev = error;
    % Calculate the produced force
    Train.F = -(R * x) + output;
    % Update the auxiliary variables
    t = t + t_new(end, :);
    n = n + 1;
    %% ----------------------------Question 21----------------------------
    % Decrease the velocity to zero when it reaches the desired velocity
    if t >= 10 && t <= 41 / 3
        V_Des = bk * (t - 10) + 10;
        if V_Des > 0
            V_Des = V_Des;
        else
            V_Des = 0;
        end
    end
    %% ------------------------Question 20 (cont.)------------------------
    V_Desired(n, :) = V_Des;
end
% Plot the elongation of the springs
subplot(2, 1, 1);
plot(tv, x_new_save(:,2:3));
xlabel('Time (s)');
ylabel('Elongation (m)');
legend("Spring 1", "Spring 2");
title('Elongation of Springs');
% Plot the velocity of the train as a whole
subplot(2, 1, 2);
plot(tv, x_obs_save(:,4));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity of Train');

    


