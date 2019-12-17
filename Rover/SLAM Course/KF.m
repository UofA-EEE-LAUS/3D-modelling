% KF Kalman Filter
%
% I. System
%
%   x+ = F_x * x + F_u * u + F_n *n
%   y  = H * x + v
%
%   x : state vector           - P : cov. matrix
%   u : control vector         
%   n : perturbation vector    - Q : cov. matrix
%   y : measurement vector
%   v : measurement noise      - R : cov. matrix
%
%   F_x : transition matrix
%   F_u : control matrix
%   F_n : pert. matrix
%   H   : measurement matrix
%
%   

