% Declare the initialization function
function Initialize()

% Define the variables of the small freight train
global Train
Train.m_1 = 3;
Train.m_2 = 1;
Train.m_3 = 1;
Train.m = Train.m_1 + Train.m_2 + Train.m_3;
Train.k_2 = 80;
Train.k_3 = 80;
Train.F = 0;
Train.f_1 = 3;
Train.f_2 = 5;
Train.f_3 = 5;
Train.L_0 = 2;

% Initialize the state vector
global x
x = zeros(6,1);

% Initialize the flag
global flag
flag = true;

% Define matrix M and calculate its inverse
Train.M = [Train.m                  Train.m_2 + Train.m_3   Train.m_3;
           Train.m_2 + Train.m_3    Train.m_2 + Train.m_3   Train.m_3;
           Train.m_3                Train.m_3               Train.m_3];
M_inv = inv(Train.M);

% Define matrix K and matrix P
Train.K = [0    0           0;
           0    Train.k_2   0;
           0    0           Train.k_3];
Train.P = [Train.f_1    0           0;
           0            Train.f_2   0;
           0            0           Train.f_3];

% Calculate matrices A and B and define matrices C and D
Train.A = [zeros(3)         eye(3);
           -M_inv*Train.K   -M_inv*Train.P];
Train.B_1 = [zeros(3);
             M_inv];
Train.B_2 = [1;
             0;
             0];
Train.B = Train.B_1 * Train.B_2;
Train.C = [1 zeros(1,5)];
Train.D = 0;
end


