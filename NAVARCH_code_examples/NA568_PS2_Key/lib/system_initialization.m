function sys = system_initialization(alphas, beta)
sys.gfun = @(mu, u) [...
    mu(1) + (-u(1) / u(2) * sin(mu(3)) + u(1) / u(2) * sin(mu(3) + u(2)));
    mu(2) + ( u(1) / u(2) * cos(mu(3)) - u(1) / u(2) * cos(mu(3) + u(2)));
    mu(3) + u(2) + u(3)];

sys.hfun = @(landmark_x, landmark_y, mu_pred) [...
    wrapToPi(atan2(landmark_y - mu_pred(2), landmark_x - mu_pred(1)) - mu_pred(3));
    sqrt((landmark_y - mu_pred(2))^2 + (landmark_x - mu_pred(1))^2)];

sys.M = @(u) [...
    alphas(1)*u(1)^2+alphas(2)*u(2)^2, 0, 0;
    0, alphas(3)*u(1)^2+alphas(4)*u(2)^2, 0;
    0, 0, alphas(5)*u(1)^2+alphas(6)*u(2)^2];

sys.Q = [...
        beta^2,    0;
        0,      25^2];
end