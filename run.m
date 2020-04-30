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
numSteps = 100000;%TODO largest timestamp in GPS file, divided by deltaT, cast to int

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

% % plot ground truth positions
% plot3(gt(:,2), gt(:,3), gt(:,4), '.g')
grid on
hold on
% % plot gps positions
% plot3(GPSData(:,2), GPSData(:,3), GPSData(:,4), '.b')
axis equal
axis vis3d

counter = 0;
MAXIGPS = 2708;
MAXIIMU = 27050;
isStart = false;

for t = 1:numSteps
    currT = t * deltaT;
    if(currT >= nextIMU(1)) %if the next IMU measurement has happened
        disp('prediction')
        filter.prediction(nextIMU(2:7));
        isStart = true;
        IMUIdx = IMUIdx + 1;
        nextIMU = IMUData(IMUIdx, :);
        plot3(filter.mu(1, 5), filter.mu(2, 5), filter.mu(3, 5), 'or');
    end
    if(currT >= nextGPS(1) & isStart) %if the next GPS measurement has happened
        disp('correction')
        counter = counter + 1;
        filter.correction(nextGPS(2:4));
        GPSIdx = GPSIdx + 1;
        nextGPS = GPSData(GPSIdx, :);
        plot3(nextGPS(2), nextGPS(3), nextGPS(4), '.g');
        plot3(filter.mu(1, 5), filter.mu(2, 5), filter.mu(3, 5), 'ok');
    end
    results(2:4, t) = filter.mu(1:3, 5); %just position so far
%     plot3(results(2, t), results(3, t), results(4, t), 'or');
%     disp(filter.mu(1:3, 1:3));
    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end
    if IMUIdx >= MAXIIMU || GPSIdx >= MAXIGPS
        break
    end
end
disp(counter)

end
