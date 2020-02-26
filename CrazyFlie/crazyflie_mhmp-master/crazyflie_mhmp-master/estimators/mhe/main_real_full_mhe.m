close all;
clear queue;
% clear all;
%% Load all data
data = readtable('280619_test1.csv');
gyro_x = data.gyro_x(~isnan(data.gyro_x))*pi/180; gyro_y = data.gyro_y(~isnan(data.gyro_y))*pi/180; gyro_z = data.gyro_z(~isnan(data.gyro_z))*pi/180; 
acc_x = data.acc_x(~isnan(data.acc_x))*9.81; acc_y = data.acc_y(~isnan(data.acc_y))*9.81; acc_z = data.acc_z(~isnan(data.acc_z))*9.81; 
ranging_distance0 = data.ranging_distance0(~isnan(data.ranging_distance0)); ranging_distance1 = data.ranging_distance1(~isnan(data.ranging_distance1)); 
ranging_distance2 = data.ranging_distance2(~isnan(data.ranging_distance2)); ranging_distance3 = data.ranging_distance3(~isnan(data.ranging_distance3)); 

gyro_data = [gyro_x gyro_y gyro_z];
acc_data = [acc_x acc_y acc_z];
uwb_data = [ranging_distance0 ranging_distance1 ranging_distance2 ranging_distance3];
time_data = data.Var1(~isnan(data.gyro_x));

z_vals = data.z(~isnan(data.gyro_x));
x_real = data.x(~isnan(data.gyro_x));
y_real = data.y(~isnan(data.gyro_x));
%% Initialize parameters
start_i = 3000;
end_i = 9000;
window = 45;

%% Initialize storage arrays
x_hat = [0.397 0]';
y_hat = [0 0]';

seq_xh = [0.397 0 0 0]';
seq_xp = seq_xh;
times = time_data(start_i)/1000;
indices = start_i;

%% initialize queue
queue.front = 1;
queue.time = time_data(start_i)/1000;
queue.x_hat = x_hat;
queue.y_hat = y_hat;
queue.xtilde = [0.397];
queue.eta = [0;0];
queue.size = 1;

%% Initialize sensor accumulation
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
flagNewPoseEstimation = 0;

%% Estimate states
counter = 0; 
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
    
%     while queue.time(end) - queue.time(1) > 0.7
    while queue.size > window
        queue.eta(:, 1) = [];
        queue.x_hat(:, 1) = [];
        queue.y_hat(:, 1) = [];
        queue.xtilde(:, 1) = [];
        queue.time(1) = [];
        queue.size = queue.size - 1;
        queue.front = queue.front - 1;
    end
    
    for j = 1:4
        if (uwb_data(i, j) - uwb_data(i - 1, j) ~= 0)
            
%             remove outliers
            if queue.size > window - 1 && (abs(uwb_data(i, j) - mean(uwb_queue{j}))/std(uwb_queue{j}) > 3.5)
                abs(uwb_data(i, j) - mean(uwb_queue{j}))/std(uwb_queue{j});
            elseif uwb(j) ~= 0
%             if uwb(j) ~= 0
                uwb(j) = 0.5*(uwb(j) + uwb_data(i, j));
%                 flagNewPoseEstimation = 1;
%                 break
            else
                uwb(j) = uwb_data(i, j);
%                 flagNewPoseEstimation = 1;
%                 break
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
        queue.eta(1, queue.front) = eta_rotated(1);
        queue.eta(2, queue.front) = eta_rotated(2);
        Ts = queue.time(queue.front) - queue.time(queue.front-1);
        xp = predict_mhmp(queue.x_hat(:, queue.front - 1), eta_rotated(1), Ts);
        yp = predict_mhmp(queue.y_hat(:, queue.front - 1), -eta_rotated(2), Ts);
        
        queue.x_hat(:, queue.front) = xp;
        queue.y_hat(:, queue.front) = yp;
        
        fake_height = z_vals(i)+R_alti*randn;  
        
        for j=1:4
            uwb(j) = uwb(j) - (bias(j, 1)*x_hat(1) + bias(j, 2)*y_hat(1) + bias(j, 3));
        end
        [~, x_sol, y_sol] = solve_position(uwb, anchors, 1.5);
        queue.xtilde(1, queue.front) = x_sol;
        queue.xtilde(2, queue.front) = y_sol;
        queue.size = queue.size + 1;
        
        if queue.size > window
            [x_hat, y_hat] = MHE_FULL(queue);
            queue.x_hat = x_hat;
            queue.y_hat = y_hat;
            x_hat = x_hat(:, end);
            y_hat = y_hat(:, end);
            counter = counter + 1;
        end
        indices = [indices i];
        times = [times time_data(i)/1000];
        seq_xh = [seq_xh [x_hat; y_hat]];
        seq_xp = [seq_xp [xp; yp]];
        seq_uwb = [seq_uwb uwb];
        uwb = [0 0 0 0]';
        flagNewPoseEstimation = 0;
        i
    end
end
toc
%% Position plot
counter 

x_real = data.x(~isnan(data.gyro_x));
y_real = data.y(~isnan(data.gyro_x));
z_real = data.z(~isnan(data.gyro_x));

figure(1);
subplot(2, 1, 1);
plot(times, seq_xp(1, :), 'DisplayName', 'MHE x');
hold on; plot(times, x_real(indices), 'DisplayName', 'real x');
% hold on; plot(times_ekf, seq_x_ekf(1, :), 'DisplayName', 'EKF x');
% hold on; plot(times, meas(1:4, :), 'DisplayName', 'UWB');
title('Position (MHE vs ground-truth)')
legend()
ylabel('x-position (m)')
xlabel('time (s)')

subplot(2, 1, 2);
plot(times, seq_xp(3, :), 'DisplayName', 'MHE y');
hold on; plot(times, y_real(indices), 'DisplayName', 'real y');
% hold on; plot(times_ekf, seq_x_ekf(2, :), 'DisplayName', 'EKF y');
title('Position (MHE vs ground-truth)')
ylabel('y-position (m)')
xlabel('time (s)')

set(gcf, 'Position', [0, 0, 900, 600]);

%% Position error plot
% 
% figure(2)
% subplot(2, 1, 1);
% plot(times, seq_x_mhe(1, :) - x_real(indices)');
% hold on; plot(times_ekf, seq_x_ekf(1, :) - x_real(indices_ekf)');
% 
% subplot(2, 1, 2);
% plot(times, seq_x_mhe(3, :) - y_real(indices)');
% hold on; plot(times_ekf, seq_x_ekf(2, :) - y_real(indices_ekf)');
% 

% 
%% Velocity plot

vxOpti = diff(x_real(indices))./diff(times'); vxOpti = [vxOpti;vxOpti(length(vxOpti))]; vxOpti = smooth(vxOpti,5); % smooth velocity from optiTrack
vyOpti = diff(y_real(indices))./diff(times'); vyOpti = [vyOpti;vyOpti(length(vyOpti))]; vyOpti = smooth(vyOpti,5);
vzOpti = diff(z_real(indices))./diff(times'); vzOpti = [vzOpti;vzOpti(length(vzOpti))]; vzOpti = smooth(vzOpti,5);

figure(3);
subplot(2, 1, 1);
plot(times, seq_xh(2, :), '.', 'DisplayName', 'MHE vx');
% hold on; plot(times_ekf, seq_x_ekf(4, :), 'DisplayName', 'EKF vx');
hold on; plot(times, vxOpti, 'DisplayName', 'Real vx');
% hold on; plot(times, eta_mahony(2, :), 'DisplayName', 'eta');
title('Velocity (MHE vs ground-truth)')
legend()
ylabel('x-velocity (m/s)')
xlabel('time (s)')

subplot(2, 1, 2);
plot(times, seq_xh(4, :),'.', 'DisplayName', 'MHE vy');
% hold on; plot(times_ekf, seq_x_ekf(5, :), 'DisplayName', 'EKF vy');
hold on; plot(times, vyOpti, 'DisplayName', 'Real vy');
title('Velocity (MHE vs ground-truth)')
ylabel('y-velocity (m/s)')
xlabel('time (s)')

set(gcf, 'Position', [0, 0, 900, 600]);

%%

fprintf('MHE error x: %d\n', rms(seq_xh(1, 5:end) - x_real(indices(5:end))'));
fprintf('MHE error y: %d\n', rms(seq_xh(3, 5:end) - y_real(indices(5:end))'));
fprintf('MHE error vx: %d\n', rms(seq_xh(2, 5:end) - vxOpti(5:end)'));
fprintf('MHE error vy: %d\n\n', rms(seq_xh(4, 5:end) - vyOpti(5:end)'));

fprintf('MHE std x: %d\n', std(seq_xh(1, 5:end) - x_real(indices(5:end))'))
fprintf('MHE std y: %d\n', std(seq_xh(3, 5:end) - y_real(indices(5:end))'))
fprintf('MHE std vx: %d\n', std(seq_xh(2, 5:end) - vxOpti(5:end)'))
fprintf('MHE std vy: %d\n\n', std(seq_xh(4, 5:end) - vyOpti(5:end)'))

fprintf('MHE rms error total: %.2d\n', ...
      rms(vecnorm([seq_xh(1, 5:end)' seq_xh(3, 5:end)'] - [x_real(indices(5:end)) y_real(indices(5:end))], 2, 2)))
fprintf('MHE rms error v total: %.2d\n', ...
      rms(vecnorm([seq_xh(2, 5:end)' seq_xh(4, 5:end)'] - [vxOpti(5:end) vyOpti(5:end)], 2, 2)))

%%

%% Attitude plot

% figure(4);
% subplot(3, 1, 1);
% plot(times, eta_mahony(1, :), 'DisplayName', 'MHE');
% % hold on; plot(times_ekf, eta_ekf(:, 1), 'DisplayName', 'EKF');
% title('Yaw (MHE vs EKF)')
% ylabel('angle (rad)');
% xlabel('time (s)');
% legend();
% 
% subplot(3, 1, 2);
% plot(times, eta_mahony(2, :), 'DisplayName', 'MHE');
% % hold on; plot(times_ekf, eta_ekf(:, 2), 'DisplayName', 'EKF');
% title('Pitch (MHE vs EKF)')
% ylabel('angle (rad)');
% xlabel('time (s)');
% legend();
% 
% subplot(3, 1, 3);
% plot(times, eta_mahony(3, :), 'DisplayName', 'MHE');
% % hold on; plot(times_ekf, eta_ekf(:, 3), 'DisplayName', 'EKF');
% title('Roll (MHE vs EKF)')
% ylabel('angle (rad)');
% xlabel('time (s)');
% legend();
% 
% set(gcf, 'Position', [0, 0, 900, 600]);
% 
% % 
% % %%
% % 
% % figure(5);
% % 
% % subplot(2, 1, 1);
% % plot(times, seq_x_mhe(1, :), 'DisplayName', 'MHE x');
% % hold on; plot(times, x_real(indices), 'DisplayName', 'real x');
% % hold on; plot(times_ekf, seq_x_ekf(1, :), 'DisplayName', 'EKF x');
% % % hold on; plot(times, meas(1:4, :), 'DisplayName', 'UWB');
% % title('Position (MHE vs ground-truth)')
% % legend()
% % ylabel('x-position (m)')
% % xlabel('time (s)')
% % 
% % subplot(2, 1, 2);
% % plot(times, seq_x_mhe(2, :), 'DisplayName', 'MHE vx');
% % hold on; plot(times_ekf, seq_x_ekf(4, :), 'DisplayName', 'EKF vx');
% % hold on; plot(times, vxOpti, 'DisplayName', 'Real vx');
% % title('Velocity (MHE vs ground-truth)')
% % legend()
% % ylabel('x-velocity (m/s)')
% % xlabel('time (s)')
% % 
% % set(gcf, 'Position', [0, 0, 900, 600]);
% 
% %% Plot trajectory and height
% 
% % subplot(2, 2, 1);
% 
% figure();
% plot(-4.715, -4.539, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
% plot(-4.819, 4.965, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
% plot(4.902, 5.029, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
% plot(4.799, -4.606, 'r', 'Marker', 'x', 'MarkerSize', 10); hold on;
% 
% plot(x_real(start_i:end_i), y_real(start_i:end_i)); hold on;  
% % plot(seq_xh(1, :), seq_xh(3, :)); 
% xlim([-5.5, 5.5]);
% ylim([-5.5, 5.5]);
% xlabel('x (m)')
% ylabel('y (m)')
% title('x versus y')
% 
% % subplot(2, 2, 2);
% figure();
% plot(times, z_vals(start_i:end_i));
% xlabel('time (s)')
% ylabel('z (m)')
% title('z height')
% 
% % subplot(2, 1, 2);
% figure()
% plot(times, ranging_distance0(start_i:end_i), 'DisplayName', 'Anchor 0'); hold on;
% plot(times, ranging_distance1(start_i:end_i), 'DisplayName', 'Anchor 1'); hold on;
% plot(times, ranging_distance2(start_i:end_i), 'DisplayName', 'Anchor 2'); hold on;
% plot(times, ranging_distance3(start_i:end_i), 'DisplayName', 'Anchor 3');
% xlabel('time (s)')
% ylabel('distance (m)')
% title('UWB distance measurements')
% legend()