close all;
clear queue;
%% Load all data
data = readtable('280619_test4.csv');
gyro_x = data.gyro_x(~isnan(data.gyro_x))*pi/180; gyro_y = data.gyro_y(~isnan(data.gyro_y))*pi/180; gyro_z = data.gyro_z(~isnan(data.gyro_z))*pi/180; 
acc_x = data.acc_x(~isnan(data.acc_x))*9.81; acc_y = data.acc_y(~isnan(data.acc_y))*9.81; acc_z = data.acc_z(~isnan(data.acc_z))*9.81; 
ranging_distance0 = data.ranging_distance0(~isnan(data.ranging_distance0)); ranging_distance1 = data.ranging_distance1(~isnan(data.ranging_distance1)); 
ranging_distance2 = data.ranging_distance2(~isnan(data.ranging_distance2)); ranging_distance3 = data.ranging_distance3(~isnan(data.ranging_distance3)); 

gyro_data = [gyro_x gyro_y gyro_z];
acc_data = [acc_x acc_y acc_z];
uwb_data = [ranging_distance0 ranging_distance1 ranging_distance2 ranging_distance3];
time_data = data.Var1(~isnan(data.gyro_x));

baro = data.baro_asl(~isnan(data.gyro_x));
baro_press = data.baro_pressure(~isnan(data.gyro_x));
baro_temp = data.baro_temp(~isnan(data.gyro_x));

z_vals = data.z(~isnan(data.gyro_x));
x_real = data.x(~isnan(data.gyro_x));
y_real = data.y(~isnan(data.gyro_x));
%% Initialize parameters
start_i = 8400;
end_i = 16000;
window = 40;

%% Initialize storage arrays
xp = [0.397 0]';
yp = [0 0]';
x_hat = xp;
y_hat = yp;
dx = [0 0]';
dy = [0 0]';

seq_xh = [0.397 0 0 0]';
seq_xp = [0 0 0 0]';
times = time_data(start_i)/1000;
meas = [ranging_distance0(start_i); 1; 0.0863];
indices = start_i;

%% initialize queue
queue.front = 1;
queue.time = time_data(start_i)/1000;
queue.var = 0.025;
queue.xp = xp;
queue.yp = yp;
queue.x_hat = xp;
queue.y_hat = yp;

queue.xtilde = [0.397];
queue.ytilde = [0];
queue.p = 0.025;

% queue.xtilde = [0.397; 0.397];
% queue.ytilde = [0; 0];
% queue.p = [0.025; 0.025];

queue.size = 1;

tk = time_data(start_i)/1000;

%% Initialize sensor accumulation

gyro = [0;0;0]; gyro_count = 0;
acc = [0;0;0]; acc_count = 0;
uwb = [ranging_distance0(start_i);ranging_distance1(start_i);ranging_distance2(start_i);ranging_distance3(start_i)];
seq_uwb = uwb;
queue.uwb = uwb;
%% Initialize attitude estimation
mahony = MadgwickAHRS();
% mahony.Kp = 0.4;
% mahony.Ki = 0.001;
mahony.Beta = 0.01;
eta_mahony = [0 0 0]';


%% Anchor locations
anchors = [-4.715, -4.539, 2.67;
           -4.819, 4.965, 1.66;
            4.902, 5.029, 0.97;
           4.799, -4.606, 0.28];
bias = [0.02998003, -0.03306038,  0.05456951; 
    -0.03206713,  0.03006817, -0.22472976; 
    -0.0592517, -0.03451476, -0.23091704; 
    -0.02753215, -0.02114928, -0.04914086];

uwb = [ranging_distance0(start_i);ranging_distance1(start_i);ranging_distance2(start_i);ranging_distance3(start_i)];
uwb_queue = {uwb(1), uwb(2), uwb(3), uwb(4)};

R_alti = 0.0429;

%% Estimate states
tic
for i = start_i+1:end_i%63693%length(gyro_x)-1%27468%
    
    Ts = (time_data(i) - time_data(i-1))/1000;

    
    mahony.SamplePeriod = Ts;
    mahony.UpdateIMU(gyro_data(i, :)', acc_data(i, :)');
    q = mahony.Quaternion';
    yaw  = atan2(2*(q(2)*q(3)+q(1)*q(4)) , q(1)*q(1) + q(2)*q(2) - q(3)*q(3) - q(4)*q(4));
    pitch= -asin(-2*(q(2)*q(4) - q(1)*q(3)));
    roll = atan2(2*(q(3)*q(4)+q(1)*q(2)) , q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4));
    eta_mahony = [eta_mahony [yaw pitch roll]'];
    
    eta_rotated = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [tan(pitch); tan(roll)];
    xp = predict_mhmp(xp, eta_rotated(1), Ts);
    yp = predict_mhmp(yp, -eta_rotated(2), Ts);
    
    while queue.size > window
        queue.xp(:, 1) = [];
        queue.yp(:, 1) = [];
        queue.p(:, 1) = [];
        queue.xtilde(:, 1) = [];
        queue.ytilde(:, 1) = [];
        queue.time(1) = [];
        queue.size = queue.size - 1;
        queue.front = queue.front - 1;
        queue.uwb(:, 1) = [];
    end
    
    for j = [1, 2, 3, 4]
        if (uwb_data(i, j) - uwb_data(i - 1, j) ~= 0)
            
%             remove outliers
            if queue.size > window - 1 && (abs(uwb_data(i, j) - mean(uwb_queue{j}))/std(uwb_queue{j}) > 3.5)
                abs(uwb_data(i, j) - mean(uwb_queue{j}))/std(uwb_queue{j});
                queue.uwb(j, :);
            elseif uwb(j) ~= 0
                uwb(j) = 0.5*(uwb(j) + uwb_data(i, j));
                flagNewPoseEstimation = 1;
                break
            else
                uwb(j) = uwb_data(i, j);
                flagNewPoseEstimation = 1;
                break
            end
            uwb_queue{j}(end+1) = uwb_data(i, j);
            if length(uwb_queue{j}) > 32
                uwb_queue{j}(1) = [];
            end

        end
    end
    
    if all(uwb ~= 0)
        flagNewPoseEstimation = 1;
    end
    if flagNewPoseEstimation
        queue.front = queue.front + 1;
        queue.time(queue.front) = time_data(i)/1000;
        queue.xp(:, queue.front) = xp;
        queue.yp(:, queue.front) = yp;
        queue.x_hat(:, queue.front) = x_hat;
        queue.y_hat(:, queue.front) = y_hat;
        queue.x0 = [dx, dy];
        queue.uwb(:, queue.front) = uwb;
        
%         for j=1:4
%             uwb(j) = uwb(j) - (bias(j, 1)*x_hat(1) + bias(j, 2)*y_hat(1) + bias(j, 3));
%         end
%         

%         [~, queue.xtilde(queue.front), queue.ytilde(queue.front)] = solve_position(uwb, anchors, 1.5);

        uwb(j) = uwb(j) - (bias(j, 1)*x_hat(1) + bias(j, 2)*y_hat(1) + bias(j, 3));
        [queue.xtilde(1, queue.front), queue.ytilde(1, queue.front)] = meas_to_x_proj(anchors(j, :), uwb(j), x_hat(1), y_hat(1), 1.5);
        queue.p(:, queue.front) = [4*uwb(j)*uwb(j)*queue.var + 2*queue.var*queue.var];

%         queue.xtilde(1, queue.front) = meas_to_x_diff(anchors(1, :)', anchors(3, :)', uwb(1), uwb(3), y_hat(1), 1.5, 1);
%         queue.xtilde(2, queue.front) = meas_to_x_diff(anchors(2, :)', anchors(4, :)', uwb(2), uwb(4), y_hat(1), 1.5, 1);
%         queue.ytilde(1, queue.front) = meas_to_x_diff(anchors(1, :)', anchors(3, :)', uwb(1), uwb(3), x_hat(1), 1.5, 0);
%         queue.ytilde(2, queue.front) = meas_to_x_diff(anchors(2, :)', anchors(4, :)', uwb(2), uwb(4), x_hat(1), 1.5, 0);
%         queue.p(:, queue.front) = [4*uwb(1)*uwb(1)*queue.var + 4*uwb(3)*uwb(3)*queue.var + 4*queue.var*queue.var; 
%                                    4*uwb(2)*uwb(2)*queue.var + 4*uwb(4)*uwb(4)*queue.var + 4*queue.var*queue.var];

        queue.size = queue.size + 1;
        
        if queue.size > 4
            [dx, dy] = MHMP(queue);
%             [dx, dy] = MHMP_RANSAC(queue);
            tk = queue.time(1);
        end
        
%         indices = [indices i];
%         times = [times time_data(i)/1000];

        seq_uwb = [seq_uwb uwb];
        uwb = [0 0 0 0]';
        flagNewPoseEstimation = 0;
%         seq_xh = [seq_xh [x_hat; y_hat]];
    end
    indices = [indices i];
    times = [times time_data(i)/1000];
    x_hat = update_mhmp(xp, dx, time_data(i)/1000 - tk);
    y_hat = update_mhmp(yp, dy, time_data(i)/1000 - tk);
    seq_xh = [seq_xh [x_hat; y_hat]];
    seq_xp = [seq_xp [xp; yp]];
end
toc
%% Position plot

x_real = data.x(~isnan(data.gyro_x));
y_real = data.y(~isnan(data.gyro_x));
z_real = data.z(~isnan(data.gyro_x));

figure(1);
subplot(2, 1, 1);
plot(times, seq_xh(1, :), 'DisplayName', 'mhmp x');
hold on; plot(times, x_real(indices), 'DisplayName', 'real x');
% hold on; plot(times_ekf, seq_x_ekf(1, :), 'DisplayName', 'EKF x');
% hold on; plot(times, meas(1:4, :), 'DisplayName', 'UWB');
title('Position (mhmp vs ground-truth)')
legend()
ylabel('x-position (m)')
xlabel('time (s)')

subplot(2, 1, 2);
plot(times, seq_xh(3, :), 'DisplayName', 'mhmp y');
hold on; plot(times, y_real(indices), 'DisplayName', 'real y');
% hold on; plot(times_ekf, seq_x_ekf(2, :), 'DisplayName', 'EKF y');
title('Position (mhmp vs ground-truth)')
ylabel('y-position (m)')
xlabel('time (s)')

set(gcf, 'Position', [0, 0, 900, 600]);

%% Position error plot
% 
% figure(2)
% subplot(2, 1, 1);
% plot(times, seq_x_mhmp(1, :) - x_real(indices)');
% hold on; plot(times_ekf, seq_x_ekf(1, :) - x_real(indices_ekf)');
% 
% subplot(2, 1, 2);
% plot(times, seq_x_mhmp(3, :) - y_real(indices)');
% hold on; plot(times_ekf, seq_x_ekf(2, :) - y_real(indices_ekf)');
% 

% 
%% Velocity plot

vxOpti = diff(x_real(indices))./diff(times'); vxOpti = [vxOpti;vxOpti(length(vxOpti))]; vxOpti = smooth(vxOpti, 40); % smooth velocity from optiTrack
vyOpti = diff(y_real(indices))./diff(times'); vyOpti = [vyOpti;vyOpti(length(vyOpti))]; vyOpti = smooth(vyOpti, 40);
vzOpti = diff(z_real(indices))./diff(times'); vzOpti = [vzOpti;vzOpti(length(vzOpti))]; vzOpti = smooth(vzOpti, 40);

figure(3);
subplot(2, 1, 1);
plot(times, seq_xh(2, :), 'DisplayName', 'mhmp vx');
% hold on; plot(times_ekf, seq_x_ekf(4, :), 'DisplayName', 'EKF vx');
hold on; plot(times, vxOpti, 'DisplayName', 'Real vx');
% hold on; plot(times, eta_mahony(2, :), 'DisplayName', 'eta');
title('Velocity (mhmp vs ground-truth)')
legend()
ylabel('x-velocity (m/s)')
xlabel('time (s)')

subplot(2, 1, 2);
plot(times, seq_xh(4, :), 'DisplayName', 'mhmp vy');
% hold on; plot(times_ekf, seq_x_ekf(5, :), 'DisplayName', 'EKF vy');
hold on; plot(times, vyOpti, 'DisplayName', 'Real vy');
title('Velocity (mhmp vs ground-truth)')
ylabel('y-velocity (m/s)')
xlabel('time (s)')

set(gcf, 'Position', [0, 0, 900, 600]);

%%

% fprintf('mhmp error x: %d\n', rms(seq_xh(1, 5:end) - x_real(indices(5:end))'));
% fprintf('mhmp error y: %d\n', rms(seq_xh(3, 5:end) - y_real(indices(5:end))'));
% fprintf('mhmp error vx: %d\n', rms(seq_xh(2, 5:end) - vxOpti(5:end)'));
% fprintf('mhmp error vy: %d\n\n', rms(seq_xh(4, 5:end) - vyOpti(5:end)'));
% 
% fprintf('mhmp std x: %d\n', std(seq_xh(1, 5:end) - x_real(indices(5:end))'))
% fprintf('mhmp std y: %d\n', std(seq_xh(3, 5:end) - y_real(indices(5:end))'))
% fprintf('mhmp std vx: %d\n', std(seq_xh(2, 5:end) - vxOpti(5:end)'))
% fprintf('mhmp std vy: %d\n\n', std(seq_xh(4, 5:end) - vyOpti(5:end)'))

fprintf('mhmp rms error total: %.2d\n', ...
      rms(vecnorm([seq_xh(1, 5:end)' seq_xh(3, 5:end)'] - [x_real(indices(5:end)) y_real(indices(5:end))], 2, 2)))
 
fprintf('mhmp std error total: %.2d\n', ...
      std(vecnorm([seq_xh(1, 5:end)' seq_xh(3, 5:end)'] - [x_real(indices(5:end)) y_real(indices(5:end))], 2, 2)))
  
fprintf('mhmp rms error v total: %.2d\n', ...
      rms(vecnorm([seq_xh(2, 5:end)' seq_xh(4, 5:end)'] - [vxOpti(5:end) vyOpti(5:end)], 2, 2)))
  
fprintf('mhmp std error v total: %.2d\n', ...
      std(vecnorm([seq_xh(2, 5:end)' seq_xh(4, 5:end)'] - [vxOpti(5:end) vyOpti(5:end)], 2, 2)))

%%

%% Attitude plot

figure(4);
subplot(3, 1, 1);
plot(times, eta_mahony(1, :), 'DisplayName', 'mhmp');
% hold on; plot(times_ekf, eta_ekf(:, 1), 'DisplayName', 'EKF');
title('Yaw (mhmp vs EKF)')
ylabel('angle (rad)');
xlabel('time (s)');
legend();

subplot(3, 1, 2);
plot(times, eta_mahony(2, :), 'DisplayName', 'mhmp');
% hold on; plot(times_ekf, eta_ekf(:, 2), 'DisplayName', 'EKF');
title('Pitch (mhmp vs EKF)')
ylabel('angle (rad)');
xlabel('time (s)');
legend();

subplot(3, 1, 3);
plot(times, eta_mahony(3, :), 'DisplayName', 'mhmp');
% hold on; plot(times_ekf, eta_ekf(:, 3), 'DisplayName', 'EKF');
title('Roll (mhmp vs EKF)')
ylabel('angle (rad)');
xlabel('time (s)');
legend();

set(gcf, 'Position', [0, 0, 900, 600]);

% 
% %%
% 
% figure(5);
% 
% subplot(2, 1, 1);
% plot(times, seq_x_mhmp(1, :), 'DisplayName', 'mhmp x');
% hold on; plot(times, x_real(indices), 'DisplayName', 'real x');
% hold on; plot(times_ekf, seq_x_ekf(1, :), 'DisplayName', 'EKF x');
% % hold on; plot(times, meas(1:4, :), 'DisplayName', 'UWB');
% title('Position (mhmp vs ground-truth)')
% legend()
% ylabel('x-position (m)')
% xlabel('time (s)')
% 
% subplot(2, 1, 2);
% plot(times, seq_x_mhmp(2, :), 'DisplayName', 'mhmp vx');
% hold on; plot(times_ekf, seq_x_ekf(4, :), 'DisplayName', 'EKF vx');
% hold on; plot(times, vxOpti, 'DisplayName', 'Real vx');
% title('Velocity (mhmp vs ground-truth)')
% legend()
% ylabel('x-velocity (m/s)')
% xlabel('time (s)')
% 
% set(gcf, 'Position', [0, 0, 900, 600]);

%% Plot trajectory and height

% subplot(2, 2, 1);

figure();
plot(-4.715, -4.539, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
plot(-4.819, 4.965, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
plot(4.902, 5.029, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
plot(4.799, -4.606, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;

plot(x_real(start_i:end_i), y_real(start_i:end_i)); hold on;  
% plot(seq_xh(1, :), seq_xh(3, :)); 
xlim([-5.5, 5.5]);
ylim([-5.5, 5.5]);
xlabel('x (m)')
ylabel('y (m)')
title('x versus y')

% subplot(2, 2, 2);
figure();
plot(times, z_vals(start_i:end_i));
xlabel('time (s)')
ylabel('z (m)')
title('z height')

% subplot(2, 1, 2);
figure()
plot(times, ranging_distance0(start_i:end_i), 'DisplayName', 'Anchor 0'); hold on;
plot(times, ranging_distance1(start_i:end_i), 'DisplayName', 'Anchor 1'); hold on;
plot(times, ranging_distance2(start_i:end_i), 'DisplayName', 'Anchor 2'); hold on;
plot(times, ranging_distance3(start_i:end_i), 'DisplayName', 'Anchor 3');
xlabel('time (s)')
ylabel('distance (m)')
title('UWB distance measurements')
legend()

% close all