function out = Gamma_2(phi)
n = norm(phi);
out = 0.5 * eye(size(phi,1)) + (n - sin(n))/(n^3) * skew(phi) + ...
      (n^2 + 2 * cos(n) - 2)/(2 * n^4) * skew(phi)^2;
% out = eye(3);
end