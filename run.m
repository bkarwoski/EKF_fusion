function varargout = run()

%%Initializations
%todo: load data here
initialStateMean = zeros(5);
% initialStateCov =

%IMU noise?

%sys = system_initialization()
%filter = filter_initialization()

%get numSteps, and deltaT, based on fastest measurement


%% main loop
for t = 1:numSteps
% read in sequential IMU, GPS data
% filter.prediction(IMU data)

%if there is new GPS data:
    %filter.correction(GPS data)
    
%plot results
%pause for some period of time
end

end