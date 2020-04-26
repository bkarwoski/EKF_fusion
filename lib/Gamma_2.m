function out = Gamma_2(phi)
n = norm(phi);
out = eye(size(phi,1)).* 0.5 + (n - sin(n))/(n^3) * skew(phi) + ...
      (n^2 + 2 * cos(n) - 2)/(2 * n^4)*skew(phi)^2;
% out = eye(3);
end