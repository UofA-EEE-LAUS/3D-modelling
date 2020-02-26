function [x_hat, y_hat] = MHE_FULL(queue)
%MHE_FULL Summary of this function goes here
%   Detailed explanation goes here


% Minimize the cost-function

% options = optimoptions('fmincon','Display', 'final', 'Algorithm','interior-point', ...
%     'StepTolerance',1e-4, 'OptimalityTolerance',1e-4);
% states = fmincon(@(x)obj_full_mhe(x, queue, 0), [queue.x_hat ; queue.y_hat], [],[],[],[],[],[],[],options);

% options = optimoptions('fminunc','Display', 'iter', 'Algorithm','quasi-newton');
% states = fminunc(@(x)obj_full_mhe(x, queue, 0), [queue.x_hat ; queue.y_hat], options);

% states = fminsearch(@(x)obj_full_mhe(x, queue, 0), [queue.x_hat ; queue.y_hat]);

options = optimoptions('lsqnonlin', 'Display', 'off', 'Algorithm', 'levenberg-marquardt');

% x0 = [queue.x_hat + 0.01*rand(2, queue.size); queue.y_hat + 0.01*rand(2, queue.size)];
x0 = [queue.x_hat; queue.y_hat];
states = lsqnonlin(@(x)obj_full_mhe(x, queue, 0), x0, [], [], options);

% obj_full_mhe(states, queue, 1);
x_hat = states(1:2, :);
y_hat = states(3:4, :);

end

