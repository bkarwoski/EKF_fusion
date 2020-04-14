function h = plotcov2d(centerX, centerY, cov, color, filled, fillColor, falpha, nSigma)
% PLOTCOV2D Hacked-together function that plots a basic representation of covariance onto a graph
% IN:    centerX - x coordinate of center point of the covariance
%        centerY - y coordinate of center point of the covariance
%        cov - covariance matrix, duh :)
%        color - Color to plot in (e.g. 'r', 'g', etc.)
% NOTES: The dog is always situated at the origin for the purposes of this model

% Some of this code is borrowed from sample code at:
% http://www.cs.columbia.edu/~jebara/6998-01/matlab.html

cov = [cov(1,1) cov(1,2); cov(2,1) cov(2,2)];

mu = [centerX; centerY];
[V,D]=eig(cov);
lam1 = D(1,1);
lam2 = D(2,2);
v1 = V(:,1);
v2 = V(:,2);
if v1(1)==0
    theta = deg2rad(90);
else
    theta = atan(v1(2)/v1(1));
end

if ~exist('nSigma','var')
    nSigma = 1;
end
a = nSigma*sqrt(lam1);
b = nSigma*sqrt(lam2);

% plot the ellipse
np = 500;
ang = [0:np]*2*pi/np;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
pts = [centerX;centerY]*ones(size(ang)) + R*[cos(ang)*a; sin(ang)*b];

h=plot( pts(1,:), pts(2,:), 'Color', color, 'LineWidth', 2);

if filled
    if falpha < 1
        h=fill(pts(1,:), pts(2,:), fillColor, 'FaceAlpha', falpha);
    else
        h=fill(pts(1,:), pts(2,:), fillColor);
    end
end

hold on;
minor1 = mu-a*v1;
minor2 = mu+a*v1;
line([minor1(1) minor2(1)], [minor1(2) minor2(2)], 'Color', color)
major1 = mu-b*v2; major2 = mu+b*v2;
hl=line([major1(1) major2(1)], [major1(2) major2(2)], 'Color', color);

