function [x_hat, q, Rotmat, P, yaw, pitch, roll] = EKF_crazyflie(meas, x_hat, q, Rotmat, P, Q, R, dt)

stateNum = 9;
dt2 = dt*dt;

w = meas(1:3);
if (all(abs(w) < 0.00001))
    w = w + 0.00001;
end
acc = meas(4:6)';
alti = meas(7);
uwb = meas(8:10);
uwb_range = meas(11:14);

R_alti = R(1);
R_uwb = R(2);

A = eye(stateNum);
e3 = [0;0;1];

w1_2t = w*dt/2;

% Calculate the Jacobian
A(1:3,1:3) = eye(3); A(1:3,4:6) = Rotmat*dt; A(1:3,7:9) = -Rotmat*matCrossPro(x_hat(4:6))*dt;
A(4:6,4:6) = eye(3)-matCrossPro(w)*dt;   A(4:6,7:9) = -9.81*matCrossPro(Rotmat'*e3)*dt;
A(7:9,7:9) = eye(3)+matCrossPro(-w1_2t)+(matCrossPro(-w1_2t))^2/2;

% Calculate state prediction based on forward euler
x_hat(1:3)=x_hat(1:3)+Rotmat*x_hat(4:6)*dt+Rotmat*acc'*dt2/2-9.81*e3*dt2/2;
x_hat(4:6)=x_hat(4:6)+acc'*dt-matCrossPro(w)*x_hat(4:6)*dt-9.81*Rotmat'*e3*dt;     

% Quadrotor z value cannot be less than zero..
% if x_hat(3)<0
%     x_hat(3)=0;x_hat(4)=0;x_hat(5)=0;x_hat(6)=0;
% end

dq = angle2q(w*dt)';  q = quat_multiply(q,dq); q = q/norm(q);

% Calculate the covariance prediction
P = A*P*A' + Q;

if alti < 10 && ~isnan(alti)
    % ??Trying to prevent singularities??
    % Scalar Kalman update
    if Rotmat(3,3)>0.1
        predictD = x_hat(3)/Rotmat(3,3);
        measureD = alti;
        H = [0 0 1/Rotmat(3,3) 0 0 0 0 0 0];
        err = measureD-predictD;
        R = R_alti^2;
        K = P*H'/(H*P*H'+R);
        x_hat = x_hat+K*err;
        P = (eye(stateNum)-K*H)*P*(eye(stateNum)-K*H)'+K*R*K';
    end
end   

if ~isnan(uwb)
    for j=1:3
        predictD = x_hat(j);
        measureD = uwb(j);
        H = [0 0 0 0 0 0 0 0 0];
        H(j) = 1;
        err = measureD-predictD;
        R = R_uwb^2;
        K = P*H'/(H*P*H'+R);
        x_hat = x_hat+K*err;
        P = (eye(stateNum)-K*H)*P*(eye(stateNum)-K*H)'+K*R*K';
    end
end

if ~isnan(uwb_range)
%     uwb_anchors = [-5 -5 0; -5 5 1.6; 5 5 1.6; 5 -5 2.7];
    uwb_anchors = [-4.715, -4.539, 2.67;
        -4.819, 4.965, 1.66;
        4.902, 5.029, 0.97;
        4.799, -4.606, 0.28];
    uwb_bias = [0.02998003, -0.03306038,  0.05456951; 
        -0.03206713,  0.03006817, -0.22472976; 
        -0.0592517, -0.03451476, -0.23091704; 
        -0.02753215, -0.02114928, -0.04914086];
    for i = 1:4
        distXYZ = x_hat(1:3) - uwb_anchors(i, :)';
        predictD = norm(distXYZ);
%         bias = uwb_bias(i, 1)*x_hat(1) + uwb_bias(i, 2)*x_hat(2) + uwb_bias(i, 3);
        measureD = uwb_range(i);
        if abs(predictD) > 0.001
            H = [distXYZ(1)/predictD distXYZ(2)/predictD distXYZ(3)/predictD 0 0 0 0 0 0];
        else
            H = [1 0 0 0 0 0 0 0 0];
        end
        err = measureD-predictD;
        R = R_uwb^2;
        K = P*H'/(H*P*H'+R);
        x_hat = x_hat+K*err;
        P = (eye(stateNum)-K*H)*P*(eye(stateNum)-K*H)'+K*R*K';
    end
end

tempq = zeros(4,1);
if ((abs(x_hat(7))>0.0001)||(abs(x_hat(8))>0.0001)||(abs(x_hat(9))>0.0001)) && ((abs(x_hat(7)) < 10) && (abs(x_hat(8)) < 10) && (abs(x_hat(9)) < 10))
    tempq=angle2q(x_hat(7:9));
    tempq=quat_multiply(q,tempq);
    q=tempq/norm(tempq);
    A=eye(stateNum);
    A(7:9,7:9)=A(7:9,7:9)+matCrossPro(-x_hat(7:9))/2+(matCrossPro(-x_hat(7:9)))^2/2;
    P = A*P*A';
end

Rotmat=quat2rotm(q');
x_hat(7)=0;x_hat(8)=0;x_hat(9)=0;

% for j=1:3
%     if (x_hat(j)<-100)
%         x_hat(j)=-100;
%     elseif (x_hat(j)>100)
%         x_hat(j)=100;
%     end
%     if (x_hat(3+j)<-10)
%         x_hat(3+j)=-10;
%     elseif (x_hat(j+3)>10)
%         x_hat(j+3)=10;
%     end
% end   

yaw  = atan2(2*(q(2)*q(3)+q(1)*q(4)) , q(1)*q(1) + q(2)*q(2) - q(3)*q(3) - q(4)*q(4));
pitch= asin(-2*(q(2)*q(4) - q(1)*q(3)));
roll = atan2(2*(q(3)*q(4)+q(1)*q(2)) , q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4));
