function varargout = run()
%for testing
clc
clear
close all
pauseLen = 0;

%%Initializations
%TODO: load data here
data = load('lib/IMU_GPS_GT_data.mat');
IMUData = data.imu;
GPSData = data.gpsAGL;
gt = data.gt;

addpath([cd, filesep, 'lib'])
initialStateMean = eye(5);
initialStateCov = eye(9);
deltaT = 1 / 30; %hope this doesn't cause floating point problems
numSteps = 3000;%TODO largest timestamp in GPS file, divided by deltaT, cast to int

results = zeros(7, numSteps);
% time x y z Rx Ry Rz

% sys = system_initialization(deltaT);
Q = blkdiag(eye(3)*(0.35)^2, eye(3)*(0.015)^2, zeros(3));
%IMU noise characteristics
%Using default values from pixhawk px4 controller
%https://dev.px4.io/v1.9.0/en/advanced/parameter_reference.html
%accel: first three values, (m/s^2)^2
%gyro: next three values, (rad/s)^2 

filter = filter_initialization(initialStateMean, initialStateCov, Q);

%IMU noise? do in filter initialization

%% main loop
IMUIdx = 1;
GPSIdx = 1;
nextIMU = IMUData(IMUIdx, :); %first IMU measurement
nextGPS = GPSData(GPSIdx, :); %first GPS measurement

%plot ground truth, raw GPS data
dpr = 100; %data plot range
% plot ground truth positions
plot3(gt(1:dpr,2), gt(1:dpr,3), gt(1:dpr,4), '.g')
grid on
hold on
% plot gps positions
plot3(GPSData(1:dpr,2), GPSData(1:dpr,3), GPSData(1:dpr,4), '.b')
axis equal
axis vis3d

for t = 1:numSteps
    currT = t * deltaT;
%     if(currT >= nextIMU(1)) %if the next IMU measurement has happened
%         disp('prediction')
%         filter.prediction(nextIMU(2:7));
%         IMUIdx = IMUIdx + 1;
%         nextIMU = IMUData(IMUIdx, :);
%         plot3(filter.mu(1, 5), filter.mu(2, 5), filter.mu(3, 5), '.r');
%         plotPose(filter.mu(1:3, 1:3), filter.mu(1:3, 5));
%         
%     end
    if(currT >= nextGPS(1)) %if the next GPS measurement has happened
        disp('correction')
        filter.correction(nextGPS(2:4));
        GPSIdx = GPSIdx + 1;
        nextGPS = GPSData(GPSIdx, :);
        plot3(filter.mu(1, 5), filter.mu(2, 5), filter.mu(3, 5), '.k');
        plotPose(filter.mu(1:3, 1:3), filter.mu(1:3, 5));
    end
    results(2:4, t) = filter.mu(1:3, 5); %just position so far
%     plot3(results(2, t), results(3, t), results(4, t), 'or');
%     disp(filter.mu(1:3, 1:3));
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end
end

    function []= plotPose(R, t)
        x = t(1);
        y = t(2);
        z = t(3);
        x_vec = R * [1; 0; 0];
        y_vec = R * [0; 1; 0];
        z_vec = R * [0; 0; 1];
        quiver3(x, y, z, x_vec(1), x_vec(2), x_vec(3), 'r');
        quiver3(x, y, z, y_vec(1), y_vec(2), y_vec(3), 'g');
        quiver3(x, y, z, z_vec(1), z_vec(2), z_vec(3), 'b');
    end

end