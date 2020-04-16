% modified on basis of Michael Kleder
% https://www.mathworks.com/matlabcentral/fileexchange/7942-covert-lat-lon-alt-to-ecef-cartesian


function [x,y,z] = gps2cart(lat,lon,alt)

    % WGS84 ellipsoid constants:
    a = 6378137;
    e = 8.1819190842622e-2;
    
    % intermediate calculation
    % (prime vertical radius of curvature)
    N = a ./ sqrt(1 - e^2 .* sin(lat).^2);
    
    % results:
    x = (N + alt) .* cos(lat) .* cos(lon);
    y = (N + alt) .* cos(lat) .* sin(lon);
    z = ((1 - e^2) .* N + alt) .* sin(lat);
    
end