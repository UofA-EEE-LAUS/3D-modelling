close all;
clear seq_x_ekf;
clear seq_xh;
time_data = states.Time;


window = 40;

% for window = 5:5:45
%%

xp = [1 0]';
yp = [1 0]';
x_hat = xp;
y_hat = yp;
x_hat_ransac = xp;
y_hat_ransac = yp;
dx = [0 0]';
dy = [0 0]';
dx_ransac = [0 0]';
dy_ransac = [0 0]';

seq_xh = [1 0 1 0]';
seq_xp = [1 0 1 0]';
seq_xh_ransac = [0 0 0 0]';

%% setup mhmp queue

queue.front = 1;
queue.time = time_data(1);
queue.var = 0.025;
queue.xp = xp;
queue.yp = yp;
queue.x_hat = xp;
queue.y_hat = yp;
% queue.xtilde = [xp(1); xp(1)];
queue.xtilde = xp(1);
% queue.xtilde = [xp(1); xp(1); xp(1); xp(1)];
queue.p = [0.025; 0.025];
% queue.p = [0.025; 0.025; 0.025; 0.025];
% queue.ytilde = [yp(1); yp(1)];
queue.ytilde = yp(1);
% queue.ytilde = [yp(1); yp(1); yp(1); yp(1)];
queue.size = 1;

tk = 0;



%%
times = input_data.Time(1);
theta = -measurements.Data(1, 16);
phi = measurements.Data(1, 15);
gyro = [0;0;0];
acc = [0;0;0];
meas = [measurements.Data(1, 11:14)'; states.Data(1, 3)];
indices = 1;


gyro = [0;0;0]; gyro_count = 0;
acc = [0;0;0]; acc_count = 0;

%% EKF 
x_hat_ekf = [1 1 0 0 0 0 0 0 0]';
seq_x_ekf = x_hat_ekf;
eta_ekf = [0 0 0];  
q_ekf = [1; 0; 0; 0];
P = diag([10000,10000,1,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001]);

R_alti = 0.0429;
R_uwb = 0.0625;
R = [R_alti R_uwb];
Rotmat = eye(3);

%%
anchors = [-4.715, -4.539, 2.67;
           -4.819, 4.965, 1.66;
            4.902, 5.029, 0.97;
           4.799, -4.606, 0.28];
bias = [0.02998003, -0.03306038,  0.05456951; 
    -0.03206713,  0.03006817, -0.22472976; 
    -0.0592517, -0.03451476, -0.23091704; 
    -0.02753215, -0.02114928, -0.04914086];

%%

mahony = MadgwickAHRS();
% mahony.Kp = 0.4;
% mahony.Ki = 0.001;
mahony.Beta = 0.01;
eta_mahony = [0 0 0]';

gyroo = [0;0;0];

uwb = measurements.Data(1, 11:14);
uwb_queue = {uwb(1), uwb(2), uwb(3), uwb(4)};


% For every sample
tic
for i = 2:length(measurements.Data(:, 1))-1
    theta = -measurements.Data(i, 16);
    phi = measurements.Data(i, 15); 
    gyro = gyro + measurements.Data(i, 1:3)'; gyro_count = gyro_count + 1;
    acc = acc + measurements.Data(i, 4:6)'; acc_count = acc_count + 1;
    
    Ts = (time_data(i) - time_data(i-1));
    
    mahony.SamplePeriod = Ts;
    mahony.UpdateIMU( measurements.Data(i, 1:3)', measurements.Data(i, 4:6)');
    q = mahony.Quaternion';
    yaw  = atan2(2*(q(2)*q(3)+q(1)*q(4)) , q(1)*q(1) + q(2)*q(2) - q(3)*q(3) - q(4)*q(4));
    pitch= -asin(-2*(q(2)*q(4) - q(1)*q(3)));
    roll = atan2(2*(q(3)*q(4)+q(1)*q(2)) , q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4));
    eta_mahony = [eta_mahony [yaw pitch roll]'];
    
    eta_rotated = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)] * [pitch; roll];
    xp = predict_mhmp(xp, tan(eta_rotated(1)), Ts);
    yp = predict_mhmp(yp, tan(eta_rotated(2)), Ts);

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
        queue.x_hat(:, 1) = [];
    end
    
    flagOutlier = 0;
    if all(measurements.Data(i, 11:14) - measurements.Data(i - 1, 11:14) ~= 0)
        uwb = measurements.Data(i, 11:14);
        for j=1:4
            if queue.size > window - 1 && (abs(uwb(j) - mean(uwb_queue{j}))/std(uwb_queue{j}) > 40)
                flagOutlier = 1;
                abs(uwb(j)  - mean(uwb_queue{j}))/std(uwb_queue{j})
            end
            uwb_queue{j}(end+1) = uwb(j);
            if length(uwb_queue{j}) > 20
                uwb_queue{j}(1) = [];
            end
        end
        if ~flagOutlier
            for j=1:4
                uwb(j) = uwb(j) - (bias(j, 1)*x_hat(1) + bias(j, 2)*y_hat(1) + bias(j, 3));
            end
            z = states.Data(i, 3);
            times = [times input_data.Time(i)];
            meas = [meas [uwb'; z]];

            %             Perform EKF update
            dt = max(min(times(end) - times(end-1), 0.15), 0.01);
            meas_ekf = [gyro'/gyro_count acc'/acc_count z NaN NaN NaN uwb];
            gyroo = [gyroo gyro/gyro_count];
            Q = diag([4.8366*dt*dt 4.8366*dt*dt 0.4530*dt*dt 4.8366*dt 4.8366*dt 0.4530*dt 0.0083*dt 0.0083*dt 0.0547*dt].^2); 
            [x_hat_ekf, q_ekf, Rotmat, P, yaw, pitch, roll] = EKF_crazyflie(meas_ekf', x_hat_ekf, q_ekf, Rotmat, P, Q, R, dt);
    %         eta_ekf = [eta_ekf; yaw, pitch, roll];
    %         seq_x_ekf = [seq_x_ekf x_hat_ekf];


            queue.front = queue.front + 1;
            queue.time(queue.front) = time_data(i);
            queue.xp(:, queue.front) = xp;
            queue.yp(:, queue.front) = yp;
            queue.x_hat(:, queue.front) = x_hat;
            queue.y_hat(:, queue.front) = y_hat;
            queue.x0 = [dx, dy];
            queue.uwb(:, queue.front) = uwb;

    %         for j=1:4
    %             [queue.xtilde(j, queue.front), queue.ytilde(j, queue.front)] = meas_to_x_proj(anchors(j, :), uwb(j), x_hat(1), y_hat(1), 1.5);
    %             queue.p(j, queue.front) = 4*uwb(j)*uwb(j)*queue.var + 2*queue.var*queue.var;
    %         end

            [~, queue.xtilde(1, queue.front), queue.ytilde(1, queue.front)] = solve_position(uwb, anchors, z);

    %         queue.xtilde(1, queue.front) = meas_to_x_diff(anchors(1, :)', anchors(3, :)', uwb(1), uwb(3), y_hat(1), z, 1);
    %         queue.xtilde(2, queue.front) = meas_to_x_diff(anchors(2, :)', anchors(4, :)', uwb(2), uwb(4), y_hat(1), z, 1);
    %         queue.ytilde(1, queue.front) = meas_to_x_diff(anchors(1, :)', anchors(3, :)', uwb(1), uwb(3), x_hat(1), z, 0);
    %         queue.ytilde(2, queue.front) = meas_to_x_diff(anchors(2, :)', anchors(4, :)', uwb(2), uwb(4), x_hat(1), z, 0);
            queue.p(:, queue.front) = [4*uwb(1)*uwb(1)*queue.var + 4*uwb(3)*uwb(3)*queue.var + 4*queue.var*queue.var; 
                                       4*uwb(2)*uwb(2)*queue.var + 4*uwb(4)*uwb(4)*queue.var + 4*queue.var*queue.var];

            queue.size = queue.size + 1;
            if queue.size > window-1
                [dx, dy] = MHMP(queue);
                [dx_ransac, dy_ransac] = MHMP_RANSAC(queue);
                tk = queue.time(1);
            end
            theta_i = 0; theta_count = 0;
            phi_i = 0; phi_count = 0;
            gyro = [0; 0; 0]; gyro_count = 0;
            acc = [0; 0; 0]; acc_count = 0;
    %         indices = [indices i];
    %         seq_xh = [seq_xh [x_hat; y_hat]];
        end

    end

    eta_ekf = [eta_ekf; yaw, pitch, roll];
    seq_x_ekf = [seq_x_ekf x_hat_ekf];
    indices = [indices i];
    
    x_hat = update_mhmp(xp, dx, time_data(i) - tk);
    y_hat = update_mhmp(yp, dy, time_data(i) - tk);
    
    x_hat_ransac = update_mhmp(xp, dx_ransac, time_data(i) - tk);
    y_hat_ransac = update_mhmp(yp, dy_ransac, time_data(i) - tk);
        
    seq_xh = [seq_xh [x_hat; y_hat]];
    seq_xh_ransac = [seq_xh_ransac [x_hat_ransac; y_hat_ransac]];
    seq_xp = [seq_xp [xp; yp]];
end
toc

%% Position plot

figure(1);
subplot(2, 2, 1);
plot(indices/500, seq_xh(1, :), 'DisplayName', 'mhmp');
% hold on; plot(indices/500, seq_x_mhmp_ransac(1, :), 'DisplayName', 'mhmp ransac x');
hold on; plot(indices/500, states.Data(indices, 1), 'DisplayName', 'real');
% hold on; plot(indices/500, seq_x_ekf(1, :), 'DisplayName', 'EKF');
hold on; plot(indices/500, seq_xh_ransac(1, :)', 'DisplayName', 'RANSAC');
title('x position (mhmp vs ground-truth)')
legend()
ylabel('x-position (m)')
xlabel('time (s)')
xlim([0, 15])
subplot(2, 2, 2);
plot(indices/500, seq_xh(3, :), 'DisplayName', 'mhmp y');
% hold on; plot(indices/500, seq_x_mhmp_ransac(3, :), 'DisplayName', 'mhmp ransac y');
hold on; plot(indices/500, states.Data(indices, 2), 'DisplayName', 'real y');
% hold on; plot(indices/500, seq_x_ekf(2, :), 'DisplayName', 'EKF y');
hold on; plot(indices/500, seq_xh_ransac(3, :)','--', 'DisplayName', 'RANSAC');
title('y position (mhmp vs ground-truth)')
ylabel('y-position (m)')
xlabel('time (s)')
xlim([0, 15])
set(gcf, 'Position', [0, 0, 900, 600]);

fprintf('window: %i\n', window);
fprintf('mhmp rms error total: %.2d\n', ...
      rms(vecnorm([seq_xh(1, :)' seq_xh(3, :)'] - [states.Data(indices, 1) states.Data(indices, 2)], 2, 2)))
fprintf('mhmp rms error v total: %.2d\n\n', ...
      rms(vecnorm([seq_xh(2, :)' seq_xh(4, :)'] - [states.Data(indices, 4) states.Data(indices, 5)], 2, 2)))

fprintf('mhmp ransac rms error total: %.2d\n', ...
      rms(vecnorm([seq_xh_ransac(1, :)' seq_xh_ransac(3, :)'] - [states.Data(indices, 1) states.Data(indices, 2)], 2, 2)))
fprintf('mhmp ransac rms error v total: %.2d\n\n', ...
      rms(vecnorm([seq_xh_ransac(2, :)' seq_xh_ransac(4, :)'] - [states.Data(indices, 4) states.Data(indices, 5)], 2, 2)))

fprintf('mhmp std error total: %.2d\n', ...
      std(vecnorm([seq_xh(1, :)' seq_xh(3, :)'] - [states.Data(indices, 1) states.Data(indices, 2)], 2, 2)))
fprintf('mhmp std error v total: %.2d\n\n', ...
      std(vecnorm([seq_xh(2, :)' seq_xh(4, :)'] - [states.Data(indices, 4) states.Data(indices, 5)], 2, 2)))

fprintf('mhmp ransac rms error total: %.2d\n', ...
      std(vecnorm([seq_xh_ransac(1, :)' seq_xh_ransac(3, :)'] - [states.Data(indices, 1) states.Data(indices, 2)], 2, 2)))
fprintf('mhmp ransac rms error v total: %.2d\n\n', ...
      std(vecnorm([seq_xh_ransac(2, :)' seq_xh_ransac(4, :)'] - [states.Data(indices, 4) states.Data(indices, 5)], 2, 2)))
% 
% fprintf('EKF rms error total: %.2d\n', ...
%       rms(vecnorm([seq_x_ekf(1, :)' seq_x_ekf(2, :)'] - [states.Data(indices, 1) states.Data(indices, 2)], 2, 2)))
% fprintf('EKF rms error v total: %.2d\n\n', ...
%       rms(vecnorm([seq_x_ekf(4, :)' seq_x_ekf(5, :)'] - [states.Data(indices, 4) states.Data(indices, 5)], 2, 2)))
  
% fprintf('EKF std error total: %.2d\n', ...
%       std(vecnorm([seq_x_ekf(1, :)' seq_x_ekf(2, :)'] - [states.Data(indices, 1) states.Data(indices, 2)], 2, 2)))
% fprintf('EKF std error v total: %.2d\n\n', ...
%       std(vecnorm([seq_x_ekf(4, :)' seq_x_ekf(5, :)'] - [states.Data(indices, 4) states.Data(indices, 5)], 2, 2)))
%   
% fprintf('mhmp rms error x: %.2d\nRANSAC rms error x: %.2d\nEKF rms error x: %.2d\n', ...
%     rms(seq_xh(1, :) - states.Data(indices, 1)'), ...
%     rms(seq_xh_ransac(1, :) - states.Data(indices, 1)'), ...
%     rms(seq_x_ekf(1, :) - states.Data(indices, 1)'))
% fprintf('mhmp rms error y: %.2d\nRANSAC rms error y: %.2d\nEKF rms error y: %.2d\n', ...
%     rms(seq_xh(3, :) - states.Data(indices, 2)'), ...
%     rms(seq_xh_ransac(3, :) - states.Data(indices, 2)'), ...
%     rms(seq_x_ekf(2, :) - states.Data(indices, 2)'))
% 
% fprintf('mhmp std x: %.2d\nRANSAC std x: %.2d\nEKF std x: %.2d\n', ...
%     std(seq_xh(1, :) - states.Data(indices, 1)'), ...
%     std(seq_xh_ransac(1, :) - states.Data(indices, 1)'), ...
%     std(seq_x_ekf(1, :) - states.Data(indices, 1)'))
% fprintf('mhmp std y: %.2d\nRANSAC std y: %.2d\nEKF std y: %.2d\n\n', ...
%     std(seq_xh(3, :) - states.Data(indices, 2)'), ...
%     std(seq_xh_ransac(3, :) - states.Data(indices, 2)'), ...
%     std(seq_x_ekf(2, :) - states.Data(indices, 2)'))
% 
% fprintf('mhmp rms error vx: %.2d\nRANSAC rms error vx: %.2d\nEKF rms error vx: %.2d\n', ...
%     rms(seq_xh(2, :) - states.Data(indices, 4)'), ...
%     rms(seq_xh_ransac(2, :) - states.Data(indices, 4)'), ...
%     rms(seq_x_ekf(4, :) - states.Data(indices, 4)'))
% fprintf('mhmp rms error vy: %.2d\nRANSAC rms error vy: %.2d\nEKF rms error vy: %.2d\n', ...
%     rms(seq_xh(4, :) - states.Data(indices, 5)'), ...
%     rms(seq_xh_ransac(4, :) - states.Data(indices, 5)'), ...
%     rms(seq_x_ekf(5, :) - states.Data(indices, 5)'))
% 
% fprintf('mhmp std vx: %.2d\nRANSAC std vx: %.2d\nEKF std vx: %.2d\n', ...
%     std(seq_xh(2, :) - states.Data(indices, 4)'), ...
%     std(seq_xh_ransac(2, :) - states.Data(indices, 4)'), ...
%     std(seq_x_ekf(4, :) - states.Data(indices, 4)'))
% fprintf('mhmp std vy: %.2d\nRANSAC std vy: %.2d\nEKF std vy: %.2d\n\n', ...
%     std(seq_xh(4, :) - states.Data(indices, 5)'), ...
%     std(seq_xh_ransac(4, :) - states.Data(indices, 5)'), ...
%     std(seq_x_ekf(5, :) - states.Data(indices, 5)'))

% figure(1);
% subplot(2, 1, 1);
% plot(indices/500, seq_x_mhmp(1, :)-states.Data(indices, 1)', 'DisplayName', 'Error x');
% ylim([-0.2, 0.2]);
% title('Position error x(mhmp vs ground-truth)')
% legend()
% ylabel('x-error (m)')
% xlabel('time (s)')
% 
% subplot(2, 1, 2);
% plot(indices/500, seq_x_mhmp(3, :)-states.Data(indices, 2)', 'DisplayName', 'Error y');
% ylim([-0.2, 0.2]);
% title('Position error y (mhmp vs ground-truth)')
% ylabel('y-error (m)')
% xlabel('time (s)')

%%

% figure(2);
subplot(2, 2, 3);
plot(indices/500, seq_xh(2, :), 'DisplayName', 'mhmp vx');
% hold on; plot(indices/500, seq_x_mhmp_ransac(2, :), 'DisplayName', 'mhmp ransac vx');
hold on; plot(indices/500, states.Data(indices, 4), 'DisplayName', 'Real vx');
% hold on; plot(indices/500, seq_x_ekf(4, :), 'DisplayName', 'EKF vx');
hold on; plot(indices/500, seq_xh_ransac(2, :), '--', 'DisplayName', 'RANSAC');
title('x velocity (mhmp vs ground-truth)')
legend()
ylabel('x-velocity (m/s)')
xlabel('time (s)')
xlim([0, 15])
subplot(2, 2, 4);
plot(indices/500, seq_xh(4, :), 'DisplayName', 'mhmp vy');
% hold on; plot(indices/500, seq_x_mhmp_ransac(4, :));
hold on; plot(indices/500, states.Data(indices, 5), 'DisplayName', 'Real vy');
% hold on; plot(indices/500, seq_x_ekf(5, :), 'DisplayName', 'EKF vy');
hold on; plot(indices/500, seq_xh_ransac(4, :), '--', 'DisplayName', 'RANSAC');
title('y velocity (mhmp vs ground-truth)')
% legend()
ylabel('y-velocity (m/s)')
xlabel('time (s)')
xlim([0, 15])
set(gcf, 'Position', [0, 0, 900, 600]);

%%

% figure(3)
% 
% subplot(2, 1, 1);
% plot(indices/500, seq_xh(1, :), 'DisplayName', 'mhmp x');
% % hold on; plot(indices/500, seq_x_mhmp_ransac(1, :), 'DisplayName', 'mhmp ransac x');
% hold on; plot(indices/500, states.Data(indices, 1), 'DisplayName', 'Opti x');
% hold on; plot(indices/500, seq_x_ekf(1, :), 'DisplayName', 'EKF x');
% title('Position (mhmp vs ground-truth)')
% legend()
% ylabel('x-position (m)')
% xlabel('time (s)')
% 
% subplot(2, 1, 2);
% plot(indices/500, seq_xh(2, :), 'DisplayName', 'mhmp vx');
% % hold on; plot(indices/500, seq_x_mhmp_ransac(2, :), 'DisplayName', 'mhmp ransac vx');
% hold on; plot(indices/500, states.Data(indices, 4), 'DisplayName', 'Opti vx');
% hold on; plot(indices/500, seq_x_ekf(4, :), 'DisplayName', 'EKF vx');
% title('Velocity (mhmp vs ground-truth)')
% legend()
% ylabel('x-velocity (m/s)')
% xlabel('time (s)')
% 
% set(gcf, 'Position', [0, 0, 900, 600]);

%%
% 
% figure(4)
% % 
% subplot(3, 1, 1);
% plot(indices/500, measurements.Data(indices, 17), 'DisplayName', 'mhmp yaw')
% hold on;
% plot(indices/500, eta_mahony(1, indices), 'DisplayName', 'comp yaw')
% % plot(indices/500, eta_ekf(:, 1), 'DisplayName', 'EKF yaw')
% plot(indices/500, states.Data(indices, 9), 'DisplayName', 'Real yaw')
% legend()
% 
% subplot(3, 1, 2);
% plot(indices/500, measurements.Data(indices, 16), 'DisplayName', 'mhmp pitch')
% hold on;
% plot(indices/500, -eta_mahony(2, indices), 'DisplayName', 'comp pitch')
% % plot(indices/500, eta_ekf(:, 2), 'DisplayName', 'EKF pitch')
% plot(indices/500, states.Data(indices, 8), 'DisplayName', 'Real pitch')
% legend()
% 
% subplot(3, 1, 3);
% plot(indices/500, -measurements.Data(indices, 15), 'DisplayName', 'mhmp roll')
% hold on;
% plot(indices/500, eta_mahony(3, indices), 'DisplayName', 'comp roll')
% % plot(indices/500, eta_ekf(:,3), 'DisplayName', 'EKF roll')
% plot(indices/500, states.Data(indices, 7), 'DisplayName', 'Real roll')
% legend()
%%
% 
% close all
% end