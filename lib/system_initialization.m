function sys = system_initialization(deltaT)

sys.gfun = @(mu, u) imuDynamics(mu, u, deltaT); %discrete IMU motion model
    
sys.hfun = @(gps_measurement_data)[...] %identity?

sys.M = %Represents noise characteristics for GPS(?)

%IMU noise characteristics
%Using default values from pixhawk px4 controller
%https://dev.px4.io/v1.9.0/en/advanced/parameter_reference.html
%accel: first three values, (m/s^2)^2
%gyro: next three values, (rad/s)^2 
sys.Q = blkdiag(eye(3)*(0.35)^2, eye(3)*(0.015)^2, zeros(3));


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

function f = imuDynamics(mu, u, deltaT)
%mu 5x5 state matrix
%u 3x2, first column accels, second column gyros
R_k = mu(1:3, 1:3);
v_k = mu(1:3, 4);
p_k = mu(1:3, 5);
a_k = u(1:3, 1);
omega_k = u(1:3, 2);
g = [0; 0; -9.81];

R_k1 = R_k * Gamma_0(omega_k * deltaT);
v_k1 = v_k + R_K * Gamma_1(omega_k * deltaT) * a_k * deltaT + g * deltaT;
p_k1 = p_k + v_k * deltaT + R_k * Gamma_2(omega_k * deltaT) * a_k *...
       deltaT^2 + 0.5 * g * deltaT^2;
   
f = [R_k1, v_k1, p_k1; 0 0 0 1 0; 0 0 0 0 1];

end