function [dx, dy] = MHMP(queue)
%MHMP Summary of this function goes here
%   Detailed explanation goes here

N = queue.size;

X = ones(N, 2);
X(:, 2) = queue.time - queue.time(1);
nEqs = size(queue.xtilde, 1);
Y = zeros(nEqs*N, 1);
Yx = Y;
Yy = Y;
X = kron(X, ones(nEqs, 1));

P = zeros(size(Yx, 1));

for i=1:N
%     Yx(nEqs*(i-1)+1:nEqs*i, 1) = [queue.xp(1, i) - queue.xtilde(1, i);
%                                   queue.xp(1, i) - queue.xtilde(2, i)];
%     Yy(nEqs*(i-1)+1:nEqs*i, 1) = [queue.yp(1, i) - queue.ytilde(1, i);
%                                   queue.yp(1, i) - queue.ytilde(2, i)];
%     P(nEqs*(i-1)+1:nEqs*i, nEqs*(i-1)+1:nEqs*i) = diag(queue.p(:, i));
    
%     P(nEqs*(i-1)+1:nEqs*i, nEqs*(i-1)+1:nEqs*i) = diag(queue.p(i));
    Yx(nEqs*(i-1)+1:nEqs*i, 1) = queue.xp(1, i) - queue.xtilde(i);
    Yy(nEqs*(i-1)+1:nEqs*i, 1) = queue.yp(1, i) - queue.ytilde(i);

%     Yx(nEqs*(i-1)+1:nEqs*i, 1) = [queue.xp(1, i) - queue.xtilde(1, i);
%                                   queue.xp(1, i) - queue.xtilde(2, i);
%                                   queue.xp(1, i) - queue.xtilde(3, i);
%                                   queue.xp(1, i) - queue.xtilde(4, i)];
%     Yy(nEqs*(i-1)+1:nEqs*i, 1) = [queue.yp(1, i) - queue.ytilde(1, i);
%                                   queue.yp(1, i) - queue.ytilde(2, i);
%                                   queue.yp(1, i) - queue.ytilde(3, i);
%                                   queue.yp(1, i) - queue.ytilde(4, i)];
end

Xx = X;
Xy = X;

% [xmax, idxmax] = max(abs(Yx));
% [ymax, idymax] = max(abs(Yy));
% 
% xmax
% 
% if xmax/idxmax > 0.08
%     Yx(idxmax) = [];
%     Xx(idxmax, :) = [];
% end
% 
% if ymax/idymax > 0.08
%     Yy(idymax) = [];
%     Xy(idymax, :) = [];
% end
% 
% P = [0 0; 0 0.1];
% dx = (Xx'*Xx + P)\(Xx'*Yx);
% dy = (Xy'*Xy + P)\(Xy'*Yy);

Pinv = eye(size(Xx, 1));

% Pinv = inv(P);
% % 
Q = [0 0; 0 100];
% Qy = [0 0; 0 1];
% 
dx = (Xx'*Pinv*Xx + Q)\(Xx'*Pinv*Yx + Q*queue.x0(:, 1));
dy = (Xy'*Pinv*Xy + Q)\(Xy'*Pinv*Yy + Q*queue.x0(:, 2));
end

