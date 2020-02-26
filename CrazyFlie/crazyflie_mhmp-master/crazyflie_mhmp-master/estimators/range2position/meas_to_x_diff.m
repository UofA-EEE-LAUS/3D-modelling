function [xm] = meas_to_x_diff(anchor1,anchor2, r1, r2, ym, zm, is_x)
%MEAS_TO_X_DIFF Determine position measurement based on difference between
%two distance measurements and the last estimate. 
x1 = anchor1(1); x2 = anchor2(1);
y1 = anchor1(2); y2 = anchor2(2);
z1 = anchor1(3); z2 = anchor2(3);

switch is_x
    case 1
        xm = (0.5*(x2^2 - x1^2 + y2^2 - y1^2 + z2^2 - z1^2 + r1^2 - r2^2) - zm*(z2 - z1) - ym*(y2 - y1))/(x2 - x1);
    case 0
        xm = (0.5*(x2^2 - x1^2 + y2^2 - y1^2 + z2^2 - z1^2 + r1^2 - r2^2) - zm*(z2 - z1) - ym*(x2 - x1))/(y2 - y1);
    case 2
        xm = (0.5*(x2^2 - x1^2 + y2^2 - y1^2 + z2^2 - z1^2 + r1^2 - r2^2) - zm*(x2 - x1) - ym*(y2 - y1))/(z2 - z1);
end
end

