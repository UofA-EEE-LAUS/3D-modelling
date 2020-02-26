function [squared, x, y, z] = solve_position(ranges, anchor_pos, z)
% SOLVE_POSITION determine x, y location based on multilaterion with 4
% anchors. 
d1 = ranges(1)^2; d2 = ranges(2)^2; d3 = ranges(3)^2; d4 = ranges(4)^2;
x1 = anchor_pos(1, 1); y1 = anchor_pos(1, 2); z1 = anchor_pos(1, 3);
x2 = anchor_pos(2, 1); y2 = anchor_pos(2, 2); z2 = anchor_pos(2, 3);
x3 = anchor_pos(3, 1); y3 = anchor_pos(3, 2); z3 = anchor_pos(3, 3);
x4 = anchor_pos(4, 1); y4 = anchor_pos(4, 2); z4 = anchor_pos(4, 3);
% 
A = [1, -2*x1, -2*y1;
     1, -2*x2, -2*y2;
     1, -2*x3, -2*y3;
     1, -2*x4, -2*y4];
 
V = [4*ranges(1)*ranges(1)*0.025 + 2*0.025*0.025 0 0 0;
    0 4*ranges(2)*ranges(2)*0.025 + 2*0.025*0.025 0 0; 
    0 0 4*ranges(3)*ranges(3)*0.025 + 2*0.025*0.025 0; 
    0 0 0 4*ranges(4)*ranges(4)*0.025 + 2*0.025*0.025];

b = [d1 - x1^2 - y1^2 - z1^2 - z^2 + 2*z*z1;
     d2 - x2^2 - y2^2 - z2^2 - z^2 + 2*z*z2;
     d3 - x3^2 - y3^2 - z3^2 - z^2 + 2*z*z3;
     d4 - x4^2 - y4^2 - z4^2 - z^2 + 2*z*z4];

% A = [2*(x4 - x2), 2*(y4 - y2);
%      2*(x1 - x3), 2*(y1 - y3)];
% 
% b = [d2 - d4 - x2^2 - y2^2 - z2^2 + x4^2 + y4^2 + z4^2 - 2*(z4 - z2)*z;
%      d3 - d1 - x3^2 - y3^2 - z3^2 + x1^2 + y1^2 + z1^2 - 2*(z1 - z3)*z];


Vinv = inv(V);
pos = (A' *Vinv* A) \ A' * Vinv * b;
% pos = (A' * A) \ A'*b;
squared = pos(1);
x = pos(2);
y = pos(3);
end

