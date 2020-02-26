function [x] = update_mhmp(xp, dx, dt)
%UPDATE_mhmp Summary of this function goes here
%   Detailed explanation goes here
x = xp - [1 dt; 0 1]* dx;
end

