run('init_quadcopter_model')
run('init_inner_PD_d')
max_ang_ref = pi/8;

PD_position_param.K = 0.1;
PD_position_param.Td = 0.1;
PD_position_param.Ti = 100;
PD_position_param.N = 100;
PD_position_param.h = 0.002;

% No I part
PosP = 0.20;
PosI = 0;
PosD = 0.24;

% simulink constants:
var_gyro = 0.0083;
var_gyro_z = 0.0543;

h = 0.002;

% Process covariance
Q = diag([4.8366*h*h 4.8366*h*h 0.4530*h*h 4.8366*h 4.8366*h 0.4530*h 0.0083*h 0.0083*h 0.0547*h].^2);

% Measurement covariance
R_alti = 0.0429;
R_uwb = 0.0625;
uwb_var = 0.0625;

R = [R_alti R_uwb];

% Initial values for states etc.
P_init = diag([10000,10000,1,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001]);
Rotmat_init = eye(3);
q_init = [1; 0; 0; 0];
x_hat_init = [0; 0; 0; 0; 0; 0; 0; 0; 0];

anchors = [-4.715, -4.539, 2.67;
           -4.819, 4.965, 1.66;
            4.902, 5.029, 0.97;
           4.799, -4.606, 0.28];

bias = [0.02998003, -0.03306038,  0.05456951; 
    -0.03206713,  0.03006817, -0.22472976; 
    -0.0592517, -0.03451476, -0.23091704; 
    -0.02753215, -0.02114928, -0.04914086];

yaw_seed = randi(1000, 1);
gyro_seed = randi(1000, [3, 1]);
acc_seed = randi(1000, [3, 1]);
uwb_seed = randi(1000, [4, 1]);
outlier_seed = randi(1000, [4, 1]);
% procent_modifier = 10; %0%
% procent_modifier = 2.24; %2.5%
procent_modifier = 1.96; %5%
% procent_modifier = 1.78; %7.5%
% procent_modifier = 1.645; %10%
% procent_modifier = 1.534; %12.5%
% procent_modifier = 1.44; %15%
% procent_modifier = 1.355; %17.5%
% procent_modifier = 1.28; %20%