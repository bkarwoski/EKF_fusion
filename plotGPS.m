clear
close

% load the data into matlab 
loadCartesianGPS

% plot ground truth positions
% plot3(x_gt, y_gt, z_gt, '.')
% grid on
% hold on
% plot gps positions
% plot3(x_gps, y_gps, z_gps, 'or')

% plot gps cartesian
plot3(Y(:,1), Y(:,2), Y(:,3), '.')

axis equal
axis vis3d