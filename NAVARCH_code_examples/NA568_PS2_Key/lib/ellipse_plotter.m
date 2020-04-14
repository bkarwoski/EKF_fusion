function ellipse_plotter(filter)
G1 = [0     0     1;...
      0     0     0;
      0     0     0];

G2 = [0     0     0;...
      0     0     1;
      0     0     0];

G3 = [0    -1     0;...
      1     0     0;
      0     0     0];
% create confidence ellipse
% first create points from a unit circle + angle (third dimension of so(3))
phi = (-pi:.01:pi)';
circle = [cos(phi), sin(phi), zeros(length(phi),1)];
% Chi-squared 3-DOF 95% percent confidence (0.05): 7.815
scale = sqrt(7.815);
ELLIPSE = zeros(size(circle,1),2);
for j = 1:size(circle,1)
     % sample covariance on SE(2)
     L = chol(filter.Sigma, 'lower');
     ell_se2_vec = scale * L * circle(j,:)';
     % retract and right-translate the ellipse on Lie algebra to SE(2) using Lie exp map
     temp = expm(G1 * ell_se2_vec(1) + G2 * ell_se2_vec(2) + G3 * ell_se2_vec(3)) * filter.mu;
     ELLIPSE(j,:) = [temp(1,3), temp(2,3)];    
end
plot(ELLIPSE(:,1), ELLIPSE(:,2));
plot(filter.mu(1,3), filter.mu(2,3), 'o')
drawnow limitrate
end

