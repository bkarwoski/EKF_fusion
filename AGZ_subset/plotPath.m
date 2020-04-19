clear
close

% load the data into matlab 
loadGroundTruthAGL

% plot ground truth positions
plot3(x_gt, y_gt, z_gt, '.')
grid on
hold on
% plot gps positions
% plot3(x_gps, y_gps, z_gps, 'or')
axis equal
axis vis3d