% Control gains
% PD gains along roll (\phi) angle
kpp = 0.2;
kdp = 0.1;

% PD gains along pitch (\theta) angle
kpt = 1;
kdt = 0.1;

% PD gains along yaw (\psi) angle
kpps = 0.001;
kdps = 0.001;

% PD gains along z-axis
kpz = 15;
kiz = 3;
kdz = 8;

Gains = [kpp kdp kpt kdt kpps kdps kpz kdz];

% Backstepping coefficients
c_9 = 2;
c_10 = 0.5;
c_11 = 2;
c_12 = 0.5;
lambda_5 = 0.015;
lambda_6 = 0.015;

% Sampling time
Ts = 1e-3;

% Quadrotor paramters 
Ix = 1.657171e-5;  % Quadrotor moment of inertia along X axis
Iy = 1.657171e-5;  % Quadrotor moment of inertia along Y axis
Iz = 2.9261652e-5;  % Quadrotor moment of inertia along Z axis

bf = 1.28192e-8;  % Thrust factor
bm = 8.06428e-5;  % Drag factor
l = 0.046;  % Arm lenght [m]
m = 0.027;  % Mass of the Quadrotor [Kg]
g = 9.81;   % Gravitational acceleration [m/s^2]

T_m = 0.0125; % Motor time constant [s]
Sigma_max = 2618; % Max propeller angular velocity [rad/s]
Fz_max = 4*bf*(Sigma_max^2); % Max thrust force

%% Waypoint filter

h = 1;
tsf = 1.5;