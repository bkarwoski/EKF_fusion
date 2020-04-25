function varargout = run(pauseLen)
clear
close
%%Initializations
%TODO: load data here

addpath([cd, filesep, 'lib'])
initialStateMean = zeros(5);
initialStateMean(1:3, 1:3) = eye(3);
initialStateCov = eye(9);
deltaT = 1 / 30; %hope this doesn't cause floating point problems
numSteps = 100;%TODO largest timestamp in GPS file, divided by deltaT, cast to int

results = zeros(7, numSteps);
% time x y z Rx Ry Rz

% sys = system_initialization(deltaT);
Q = blkdiag(eye(3)*(0.35)^2, eye(3)*(0.015)^2, zeros(3));
%IMU noise characteristics
%Using default values from pixhawk px4 controller
%https://dev.px4.io/v1.9.0/en/advanced/parameter_reference.html
%accel: first three values, (m/s^2)^2
%gyro: next three values, (rad/s)^2 

filter = filter_initialization(initialStateMean, initialStateCov);

%IMU noise? do in filter initialization

%% main loop
IMUIdx = 1;
GPSIdx = 1;
nextIMU = IMUData(IMUIdx, :); %first IMU measurement
nextGPS = gps_measurement(GPSIdx, :); %first GPS measurement

%plot ground truth, raw GPS data
loadGroundTruthAGL
% plot ground truth positions
plot3(x_gt, y_gt, z_gt, '.')
grid on
hold on
% plot gps positions
plot3(x_gps, y_gps, z_gps, 'or')
axis equal
axis vis3d

for t = 1:numSteps
    currT = t * deltaT;
    if(currT >= nextIMU(1)) %if the next IMU measurement has happened
        filter.prediction(nextIMU)
        IMUIdx = IMUIdx + 1;
        nextIMU = IMUData(IMUIdx, :);
    end
    if(currT >= nextGPS(1)) %if the next GPS measurement has happened
        filter.correction(nextGPS);
        GPSIdx = IMUIdx + 1;
        nextGPS = gps_measurement(GPSIdx, :);
    end
    results(2:4, t) = filter.mu(5, 1:3); %just position so far
    plot3(results(2:4, t));
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end
end
%if there is new IMU data:
% filter.prediction(IMU data)

%if there is new GPS data:
    %filter.correction(GPS data)
    
%plot results
%pause for some period of time
end

end