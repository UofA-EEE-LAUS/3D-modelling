function [xhat] = predict_mhmp(x, u, dt)
%PREDICT_mhmp Summary of this function goes here
%   u is angle

c = 0.35;
% c = 3.7*10^-5;
g = 9.81;

% A = [0 1; 0 -c];
% B = [0; -g];
% C = [1 0];
% 
% sys = ss(A, B, C, 0);
% sysd = c2d(sys, dt);
% xhat = sysd.A*x + sysd.B*tan(u);

A = [1 dt; 0 1-c*dt];
B = [0; -g*dt];

xhat = A*x + B*u;
end

