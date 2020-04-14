function varargout = draw_ellipse(mu, Sigma, k2, varargin)
% h = draw_ellipse(mu, Sigma, K2)
% mu     is the [2x1] [x;y] mean vector
% Sigma  is the [2x2] covariance matrix
% K2     is the Chi-squared 2 DOF variable
% h      is the output plot handle
%
% Draws an ellipse centered at mu with covariance Sigma and
% confidence region k2, i.e.,
% K2 = 1; % 1-sigma
% K2 = 4; % 2-sigma
% K2 = 9; % 3-sigma
% K2 = chi2inv(.50, 2); % 50% probability contour
%
% h = draw_ellipse(mu, Sigma, K2, 'Npoints', N)
% Use the 'Npoints' option to specify the number of points used to define the
% contour, N=20 is the default.
%
% h = draw_ellipse(mu,Sigma,K2,varargin)
% Optional line plot commands can be passed via the varargin syntax, e.g.
% % 1-sigma contour with red dash-dot ellipse and line thickness of 1.5
% draw_ellipse([0;0], eye(2), 1, 'r-.-','linewidth',1.5); 
%
%-------------------------------------------------------
% 2004-11-15    rme    Rewrote to use calculateEllipseXY.m
% 2004-11-21    rme    Added varargin option.
% 2006-05-25    rme    Added optional 'points' argument.
% 2006-06-03    rme    Changed 'points' to 'Npoints' and fixed a bug 
%                      in setting it's default value.
% 2006-06-18    rme    Made plot handle, h, an optional output arg.
% 2009-09-09    rme    Made help description more verbose.

Npoints = 20;
if (nargin > 3)
  for ii=1:length(varargin) % check for 'Npoints', N pair option
    if ischar(varargin{ii}) && strcmpi(varargin{ii},'npoints');
      Npoints = varargin{ii+1};
      varargin = {varargin{1:ii-1},varargin{ii+2:end}};
      break;
    end
  end
end;
    
[x,y] = calculateEllipseXY(mu, Sigma, k2, Npoints);

if nargin > 3
  hp = plot(x,y,varargin{:},'linewidth',1.25);
else
  hp = plot(x,y,'linewidth',1.25);
end

if (nargout > 0)
  varargout{1} = hp;
end
