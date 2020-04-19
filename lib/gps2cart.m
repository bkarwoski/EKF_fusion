% modified on basis of Michael Kleder
% https://www.mathworks.com/matlabcentral/fileexchange/7942-covert-lat-lon-alt-to-ecef-cartesian

% check the following paper for a detailed explanation
% http://papers.cumincad.org/data/works/att/a38d.content.03652.pdf

function [x,y,z] = gps2cart(lat,lon,alt)

    % WGS84 ellipsoid constants:
    a = 6378137; % earth radius (semi-major axis of the ellipse)
    e = 8.1819190842622e-2; % first eccentricity
    
    % intermediate calculation
    % N is vertical radius of curvature
    N = a ./ sqrt(1 - e^2 .* sin(lat).^2);
    
    % results:
    x = (N + alt) .* cos(lat) .* cos(lon);
    y = (N + alt) .* cos(lat) .* sin(lon);
    z = ((1 - e^2) .* N + alt) .* sin(lat);
    
end