function out = Gamma_1(phi)
%     n = norm(phi);
%     out = eye(size(phi,1)) + (1 - cos(n)) / n^2 * skew(phi) +...
%           (n - sin(n)) / n^3 * skew(phi)^2;
out = eye(3);
end