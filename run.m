function varargout = run(pauseLen)
%for testing
clear
pauseLen = 0.1;

%%Initializations
%TODO: load data here
data = load('lib/IMU_GPS_GT_data.mat');
IMUData = data.imu;
GPSData = data.gps;
gt = data.gt;

addpath([cd, filesep, 'lib'])
initialStateMean = eye(5);
initialStateCov = eye(9);
deltaT = 1 / 30; %hope this doesn't cause floating point problems
numSteps = 20;%TODO largest timestamp in GPS file, divided by deltaT, cast to int

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

% plot ground truth positions
plot3(gt(:,1), gt(:,2), gt(:,3), '.')
grid on
hold on
% plot gps positions
plot3(GPSData(:,2), GPSData(:,3), GPSData(:,4), 'or')
axis equal
axis vis3d

for t = 1:numSteps
    currT = t * deltaT;
    if(currT >= nextIMU(1)) %if the next IMU measurement has happened
        filter.prediction(nextIMU(2:7));
        IMUIdx = IMUIdx + 1;
        nextIMU = IMUData(IMUIdx, :);
    end
    if(currT >= nextGPS(1)) %if the next GPS measurement has happened
        filter.correction(nextGPS(2:4));
        GPSIdx = IMUIdx + 1;
        nextGPS = GPSData(GPSIdx, :);
    end
    results(2:4, t) = filter.mu(5, 1:3); %just position so far
    plot3(results(2, t), results(3, t), results(4, t));
%     disp(filter.mu(:,:));
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