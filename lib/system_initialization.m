
%NOTE: this function probably won't be used in the real system
function sys = system_initialization(deltaT)

sys.gfun = @(mu, u) imuDynamics(mu, u, deltaT); %discrete IMU motion model
    
% sys.hfun %=  %identity?
%Hfun not needed?
% sys.M = [zeros(3,2), eye(3), zeros(3,2); zeros
%Represents noise characteristics for GPS(?)

sys.Q = blkdiag(eye(3)*(0.35)^2, eye(3)*(0.015)^2, zeros(3));
%IMU noise characteristics
%Using default values from pixhawk px4 controller
%https://dev.px4.io/v1.9.0/en/advanced/parameter_reference.html
%accel: first three values, (m/s^2)^2
%gyro: next three values, (rad/s)^2 

%from SE(2) example:
% sys.gfun = @(mu, u) [...
%     mu(1) + (-u(1) / u(2) * sin(mu(3)) + u(1) / u(2) * sin(mu(3) + u(2)));
%     mu(2) + ( u(1) / u(2) * cos(mu(3)) - u(1) / u(2) * cos(mu(3) + u(2)));
%     mu(3) + u(2) + u(3)];
% 
% sys.hfun = @(landmark_x, landmark_y, mu_pred) [...
%     wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));
%     sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2)];
% 
% sys.M = @(u) [...
%     alphas(1)*u(1)^2+alphas(2)*u(2)^2, 0, 0;
%     0, alphas(3)*u(1)^2+alphas(4)*u(2)^2, 0;
%     0, 0, alphas(5)*u(1)^2+alphas(6)*u(2)^2];
% 
% sys.Q = [...
%         beta^2,    0;
%         0,      25^2];
end