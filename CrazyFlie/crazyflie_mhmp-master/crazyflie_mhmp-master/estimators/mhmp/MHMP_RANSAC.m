function [dx, dy] = MHMP_RANSAC(queue)
%MHMP_RANSAC Summary of this function goes here
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

% Add the ransac algorithm
sigma = 0.6;
emin = inf;

% samples = nchoosek(1:nEqs*N, ceil(sigma*nEqs*N));
% samples = randperm(nEqs*N, [10, ceil(sigma*nEqs*N)]);
P = [0 0; 0 1];
Pinv = eye(size(Xx, 1));
% P = inv(P);
Q = [0 0; 0 70];
for i=1:60
    sample_id = randperm(nEqs*N, ceil(sigma*nEqs*N));
%     sample_id = samples(i, :);
    X_temp = X(sample_id, :);
    Yx_temp = Yx(sample_id);
    Yy_temp = Yy(sample_id);
%     P_temp = P(sample_id, :);
%     P_temp = P_temp(:, sample_id);
    P_temp = eye(size(X_temp,1));
%     temp = (X_temp'*X_temp + P)\X_temp';
%     dx_temp = temp*Yx_temp;
%     dy_temp = temp*Yy_temp;
    temp = (X_temp'*P_temp*X_temp + Q);
    dx_temp = temp\(X_temp'*P_temp*Yx_temp + Q*queue.x0(:, 1));
    dy_temp = temp\(X_temp'*P_temp*Yy_temp + Q*queue.x0(:, 2));

    ei = 0;
%     eiy = 0;
    for j=1:N
        ei = ei + min(norm(X(j, 2)*dx_temp(2) + dx_temp(1) - Yx(j)), 1.5);
        ei = ei + min(norm(X(j, 2)*dy_temp(2) + dy_temp(1) - Yy(j)), 1.5);
    end
    
    if ei < emin
        emin = ei;
        dx = dx_temp;
        dy = dy_temp;
        sample_id_min = sample_id;
    end
end

% emin = inf;

% for i=1:4000
%     sample_id = randperm(nEqs*N, ceil(sigma*nEqs*N));
% %   sample_id = samples(i, :);
%     X_temp = X(sample_id, :);
%     Yy_temp = Yy(sample_id);
%     temp = (X_temp'*X_temp + P)\X_temp';
%     dy_temp = temp*Yy_temp;
% %     P_temp = P(sample_id, :);
% %     P_temp = P_temp(:, sample_id);
%     P_temp = eye(size(X_temp,1));
% %     temp = (X_temp'*P_temp*X_temp + Q);
% %     dy_temp = temp\(X_temp'*P_temp*Yy_temp + Q*queue.x0(:, 2));
%     
%     eiy = 0;
%     for j=1:N
% %         test = min(norm(X(j, 2)*dy_temp(2) + dy_temp(1) - Yy(j))^2, 15);
%         eiy = eiy + min(norm(X(j, 2)*dy_temp(2) + dy_temp(1) - Yy(j)), 1.5);
%     end
%     if eiy < emin
%         emin = eiy;
%         dy = dy_temp;
%         sample_id_min = sample_id;
%     end
% end


% if emin > 10
%     emin
%     sample_id_min
% end
% P = [0 0; 0 0.000001];
% dx = (Xx'*Xx + P)\(Xx'*Yx);
% dy = (Xy'*Xy + P)\(Xy'*Yy);


% Pinv = inv(P);
% % Pinv = eye(size(Xx, 1));
% Q = [0 0; 0 0.001];
% dx = (Xx'*Pinv*Xx + Q)\(Xx'*Pinv*Yx + Q*queue.x0(:, 1));
% dy = (Xy'*Pinv*Xy + Q)\(Xy'*Pinv*Yy + Q*queue.x0(:, 2));
end

