% convert a angle to a quaternion
function y = angle2q(v)
angle = norm(v);
ca = cos(angle/2);
sa = sin(angle/2);
y = [ca sa*v(1)/angle sa*v(2)/angle sa*v(3)/angle];

% function y = quat2rotmatrix(v)
% y = [1-2*v(3)^2-2*v(4)^2      2*v(2)*v(3)-2*v(4)*v(1)     2*v(2)*v(4)+2*v(3)*v(1);
%     2*v(2)*v(3)+2*v(4)*v(1)   1-2*v(2)^2-2*v(4)^2         2*v(3)*v(4)-2*v(2)*v(1);
%     2*v(2)*v(4)-2*v(3)*v(1)   2*v(3)*v(4)+2*v(2)*v(1)     1-2*v(2)^2-2*v(3)^2];