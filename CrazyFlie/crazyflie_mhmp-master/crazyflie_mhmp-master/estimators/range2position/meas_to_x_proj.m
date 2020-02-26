function [xm, ym] = meas_to_x_proj(anchor, r, xh, yh, zh)
%MEAS_TO_X_PROJ Determine position measurement from distance measurement by
%projection the measurement on the current estimate
%   Detailed explanation goes here

pos = [xh; yh; zh];
xy = anchor(1:2)' + r * (pos(1:2) - anchor(1:2)')/norm(pos(1:2) - anchor(1:2)');
xm = xy(1); ym = xy(2);

% xyz = anchor + r * (pos - anchor)/norm(pos - anchor);
% xm = xyz(1); ym = xyz(2);
end

