function [cost] = obj_full_mhe(x, queue, print)
%OBJ_FULL_MHE Summary of this function goes here
%   Detailed explanation goes here



% Calculate measurement cost
anchors = [-4.715, -4.539, 2.67;
           -4.819, 4.965, 1.66;
            4.902, 5.029, 0.97;
           4.799, -4.606, 0.28];
       
nMeas = 2;
h = zeros(nMeas, queue.size - 1);
for i=2:queue.size
%     h(i-1) = sqrt((anchors(queue.xtilde(2, i), 1) - x(1, i))^2 + ...
%                 (anchors(queue.xtilde(2, i), 2) - x(3, i))^2 + ...
%                 (anchors(queue.xtilde(2, i), 3) - 1.5)^2); % z fixed at 1.5m
    h(1, i-1) = x(1, i);
    h(2, i-1) = x(3, i);
end

Wm = diag(ones(nMeas*(queue.size-1), 1)/0.0625);

meas_cost = (reshape(queue.xtilde(:, 2:end) - h, [], nMeas*(queue.size-1))) * Wm; %* (queue.xtilde(1, 2:end) - h)';

% Calculate process cost
c = 0.5;
% c = 3.7*10^-5;
g = 9.81; 

f = zeros(4, queue.size - 1);
p = zeros(4, queue.size - 1);
for i=1:queue.size-1
    dt = queue.time(i+1) - queue.time(i);
    f(1:2, i) = [1 dt; 0 1 - c*dt] * x(1:2, i) + [0; -g*dt] * tan(queue.eta(1, i));
    f(3:4, i) = [1 dt; 0 1 - c*dt] * x(3:4, i) + [0; -g*dt] * tan(-queue.eta(2, i));
    p(:, i) = 5000*[1 10000 1 10000]';
%     p(:, i) = 5000*[1 100 1 100]'; %simulation
end

proc_size = (queue.size - 1) * 4;
Wp = diag(reshape(p, [], proc_size));
process_cost = (reshape(x(:, 2:end), [], proc_size) - reshape(f, [], proc_size)) ...
                * Wp; %* (reshape(x(:, 2:end), proc_size, []) - reshape(f, [], proc_size)');

% Calculate arrival cost

% Wa = [100 0 0 0;
%       0 50 0 0;
%       0 0 100 0;
%       0 0 0 50];
   
Wa = [1 0 0 0;
      0 1 0 0;
      0 0 1 0;
      0 0 0 1];

arrival_cost = ([queue.x_hat(:, 1); queue.y_hat(:, 1)] - x(:, 1))' ...
                * Wa; %* ([queue.x_hat(:, 1); queue.y_hat(:, 1)] - x(:, 1));
if print
    sum(meas_cost)
    sum(process_cost)
    sum(arrival_cost)
end
% cost = meas_cost + process_cost + arrival_cost;
cost = [meas_cost'; process_cost'; arrival_cost'];

end

