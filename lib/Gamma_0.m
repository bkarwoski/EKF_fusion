function out = Gamma_0(phi)
    n = norm(phi);
    out = eye(size(phi,1)) + sin(n) / n * skew(phi) +...
             (1 - cos(n))/n^2 * skew(phi)^2;
% out = eye(3);
end