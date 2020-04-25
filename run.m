function varargout = run(pauseLen)

%%Initializations
%todo: load data here

addpath([cd, filesep, 'lib'])
initialStateMean = zeros(5);
initialStateMean(1:3, 1:3) = eye(3);
initialStateCov = eye(9);
deltaT = 1 / 30 %hope this doesn't cause floating point problems
numSteps = %largest timestamp in GPS file, divided by deltaT, cast to int
results = zeros(7, numSteps)
sys = system_initialization();
filter = filter_initialization(sys, initialStateMean, initialStateCov);

%IMU noise? do in filter initialization

%% main loop
IMUIdx = 1;
GPSIdx = 1;
nextIMU = IMUData(IMUIdx, :) %first IMU measurement
nextGPS = gps_measurement(GPSIdx, :) %first GPS measurement
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
    results(1:3, t) = filter.mu(5, 1:3); %just position so far
end
%if there is new IMU data:
% filter.prediction(IMU data)

%if there is new GPS data:
    %filter.correction(GPS data)
    
%plot results
%pause for some period of time
end

end