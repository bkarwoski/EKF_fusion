function [x,y] = calculateEllipseXY(mu, Sigma, k2, N)
%function [x,y] = calculateEllipseXY(mu, Sigma, k2, N)
%
% mu     is the [2x1] [x;y] mean vector
% Sigma  is the [2x2] covariance matrix
% k2     is the Chi-squared 2 DOF variable
% N      is the number of points to use (optional, default 20)
%
%-------------------------------------------------------
% created from draw_ellipse 10/03/2003 rme
% 11-04-2004     rme    Rewrote.
% 11-15-2004     rme    Fixed a bug, wasn't using k2 in the construction of A.
% 11-21-2004     rme    Fixed a bug in call to eig() when Sigma was sparse
% 05-25-2006     rme    Added optional N argument.
% 06-14-2006     rme    Modified the calculation of A to use D.^0.5 instead 
%                       of D^0.5, which is the matrix square root.  Both are the
%                       same when D is a diagonal matrix, but D.^0.5 is computed 
%                       elementwise and therefore O(5) faster.
% 06-15-2006     rme    When Sigma is symmetric positive semi-definite, numerical
%                       errors can lead to small negative diagonal elements in the 
%                       eigenvalue decomposition, therefore, set A=real(A).

if ~exist('N','var'); N = 20; end;
  
persistent CIRCLE;
if isempty(CIRCLE) || (size(CIRCLE,2) ~= N)
  theta  = linspace(0, 2*pi, N);
  CIRCLE = [cos(theta); 
	    sin(theta)];
end

% eigenvalue decompostion
[V,D] = eig(full((Sigma+Sigma')/2));

% compute linear system which transforms unit variance to Sigma
A = V*(k2*D).^0.5;
A = real(A);

% map unit circular covariance to k2 Sigma ellipse
Y = A*CIRCLE;

% shift origin to the mean
x = mu(1) + Y(1,:)';
y = mu(2) + Y(2,:)';
