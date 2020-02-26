function [qout] = quat_multiply(q, r)
%QUAT_MULTIPLY Summary of this function goes here
%   Detailed explanation goes here

% Calculate vector portion of quaternion product
% vec = s1*v2 + s2*v1 + cross(v1,v2)
vec = zeros(3,1);
vec = [q(1).*r(2) q(1).*r(3) q(1).*r(4)] + ...
         [r(1).*q(2) r(1).*q(3) r(1).*q(4)]+...
         [ q(3).*r(4)-q(4).*r(3) ...
           q(4).*r(2)-q(2).*r(4) ...
           q(2).*r(3)-q(3).*r(2)];

% Calculate scalar portion of quaternion product
% scalar = s1*s2 - dot(v1,v2)
scalar = 0;
scalar = q(1).*r(1) - q(2).*r(2) - ...
             q(3).*r(3) - q(4).*r(4);

qout = zeros(4,1);
qout = [scalar;  vec(1); vec(2); vec(3)];
end

