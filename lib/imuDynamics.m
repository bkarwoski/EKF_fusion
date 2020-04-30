function mu_pred = imuDynamics(mu, u, deltaT)
%mu 5x5 state matrix
%u 1x6, first accels, second gyros
R_k = mu(1:3, 1:3);
v_k = mu(1:3, 4);
p_k = mu(1:3, 5);
a_k = u(1:3)';
omega_k = u(4:6);
g = [0; 0; -9.81];

R_k1 = R_k * expm(skew(omega_k * deltaT));
v_k1 = v_k + R_k * Gamma_1(omega_k * deltaT) * a_k * deltaT + g * deltaT;
p_k1 = p_k + v_k * deltaT + R_k * Gamma_2(omega_k * deltaT) * a_k *...
       deltaT^2 + 0.5 * g * deltaT^2;
   
mu_pred = [R_k1, v_k1, p_k1; 0 0 0 1 0; 0 0 0 0 1];
end