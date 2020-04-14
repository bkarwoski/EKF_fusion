%generateScript: simulates the trajectory of the robot using square
%                path given by generate motion
%
%data=generateScript(initialstatemean,numSteps,alphas,betas)
%     generates data of the form
%     [realObservation', noisefreeMotion', noisefreeObservation',
%     realRobot', noisefreeRobot']
%
%realObservation and noisefreeMotion is the only data available to
%filter.  All other data for debugging/display purposes.
%
%alphas are the 4-d noise for robot motion
%beta: noise for observations
%
%right now, observation ids not based on relationship between robot and marker
%
% TODO: update generateScript so that observations are for the
% closest marker

function data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT)

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------

motionDim = 3;
observationDim = 3; % observation size (range, bearing, marker ID)

realRobot = initialStateMean;
noisefreeRobot = initialStateMean;

% soccer field
global FIELDINFO;
FIELDINFO = getfieldinfo();

data = zeros(numSteps, 22);
for n = 1:numSteps
    % --------------------------------------------
    % Simulate motion
    % --------------------------------------------
    
    
    
    
    t=n*deltaT;
    noisefreeMotion = generateMotion(t,deltaT);

    % Noise-free robot
    prevNoisefreeRobot = noisefreeRobot;
    dx = cos(noisefreeRobot(3) + noisefreeMotion(1)) * noisefreeMotion(2);
    dy = sin(noisefreeRobot(3) + noisefreeMotion(1)) * noisefreeMotion(2);
    linear_vel = sqrt(dx^2 + dy^2);
    angular_vel = noisefreeMotion(1) + noisefreeMotion(3);
    noisefreeMotion = [linear_vel;angular_vel;0];
    
    noisefreeRobot = sampleOdometry(noisefreeMotion, noisefreeRobot, [0 0 0 0 0 0]);

    % Move robot
    [realRobot,noisymotion] = sampleOdometry(noisefreeMotion, realRobot, alphas);

    %--------------------------------------------------------------
    % Simulate observation
    %--------------------------------------------------------------

    % n / 2 causes each landmark to be viewed twice
    markerId = mod(floor(n / 2), FIELDINFO.NUM_MARKERS) + 1;
    markerId_2 = mod(floor(n / 2), FIELDINFO.NUM_MARKERS) + 2;
    if markerId_2 == 7
        markerId_2 = 1;
    end
    
    landmark_x = FIELDINFO.MARKER_X_POS(markerId);
    landmark_y = FIELDINFO.MARKER_Y_POS(markerId);
    landmark_x2 = FIELDINFO.MARKER_X_POS(markerId_2);
    landmark_y2 = FIELDINFO.MARKER_Y_POS(markerId_2);
    
    b = [landmark_x;landmark_y;1]; 
    b2 = [landmark_x2;landmark_y2;1];
    
    groundtruth = posemat(realRobot);
    
    N = diag([10^2; 10^2]);
    LN = chol(N, 'lower');

    Y = groundtruth \ b + [LN * randn(2,1); 0];
    Y2 = groundtruth \ b2 + [LN * randn(2,1); 0];

    noisefreeObservation = observation(realRobot, markerId);
    
    % Observation noise
    Q = [beta^2   0    0;
         0        10^2 0;
         0        0    0];
    
    observationNoise = mvnrnd(zeros(observationDim,1), Q)';
    realObservation = noisefreeObservation + observationNoise;


    % data = [bearing(noise), range(noise), ID, v(noise), w(noise), g(noise), bearing, range, ID, x(noise), y(noise), theta(noise),  x, y, theta,     Y,    Y2, markerID2]
    %      = [             1,            2,  3,        4,        5,        6,       7,     8,  9,       10,       11,           12, 13,14,    15, 16-18, 19-21,        22]
    data(n,:) = [realObservation', noisymotion, noisefreeObservation', realRobot', noisefreeRobot', Y', Y2', markerId_2];
end
end

function H = posemat(mu)
x = mu(1);
y = mu(2);
h = mu(3);
% construct a SE(2) matrix element
H = [cos(h) -sin(h) x;
     sin(h)  cos(h) y; 
     0 0 1];
end