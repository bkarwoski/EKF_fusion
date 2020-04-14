function varargout = run(stepsOrData, pauseLen, makeVideo, filter_name)
% RUN  PS2 Feature-Based Localization Simulator
%   RUN(ARG)
%   RUN(ARG, PAUSLEN, MAKEVIDEO)
%      ARG - is either the number of time steps, (e.g. 100 is a complete
%            circuit) or a data array from a previous run.
%      PAUSELEN - set to `inf`, to manually pause, o/w # of seconds to wait
%                 (e.g., 0.3 is the default)
%      MAKEVIDEO - boolean specifying whether to record a video or not
%
%   DATA = RUN(ARG,PAUSELEN)
%      DATA - is an optional output and contains the data array generated
%             and/or used during the simulation.

addpath([cd, filesep, 'lib'])
if ~exist('pauseLen','var') || isempty(pauseLen)
    pauseLen = 0.3; % seconds
end
if ~exist('makeVideo','var') || isempty(makeVideo)
    makeVideo = false;
end

%--------------------------------------------------------------
% Graphics
%--------------------------------------------------------------

NOISEFREE_PATH_COL = 'green';
ACTUAL_PATH_COL = 'blue';

NOISEFREE_BEARING_COLOR = 'cyan';
OBSERVED_BEARING_COLOR = 'red';

GLOBAL_FIGURE = 1;

if makeVideo
    try
        votype = 'avifile';
        vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    catch
        votype = 'VideoWriter';
        vo = VideoWriter('video', 'MPEG-4');
        set(vo, 'FrameRate', min(5, 1/pauseLen));
        open(vo);
    end
end

%--------------------------------------------------------------
% Initializations
%--------------------------------------------------------------
deltaT = 0.1;
initialStateMean = [180 50 0]';
initialStateCov = eye(3);

% Motion noise (in odometry space, see Table 5.5, p.134 in book).
alphas = [0.00025 0.00005 0.0025 0.0005 0.0025 0.0005].^2; % variance of noise proportional to alphas

% Standard deviation of Gaussian sensor noise (independent of distance)
beta = deg2rad(5);

persistent data numSteps;
if isempty(stepsOrData) % use dataset from last time
    if isempty(data)
        numSteps = 100;
        data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
    end
elseif isscalar(stepsOrData)
    % Generate a dataset of motion and sensor info consistent with
    % noise models.
    numSteps = stepsOrData;
    data = generateScript(initialStateMean, numSteps, alphas, beta, deltaT);
else
    % use a user supplied dataset from a previous run
    data = stepsOrData;
    numSteps = size(data, 1);
    global FIELDINFO;
    FIELDINFO = getfieldinfo;
end

results = zeros(7,numSteps);
sys = system_initialization(alphas, beta);

filter = filter_initialization(sys, initialStateMean, initialStateCov, filter_name);

for t = 1:numSteps
    %=================================================
    % data available to your filter at this time step
    %=================================================
    motionCommand = data(t,4:6)'; % [Trans_vel,Angular_vel]' noisefree control command
    observation = data(t,1:3)';   % [bearing, landmark_id]' noisy observation
    
    
    Y = data(t,16:18); % landmark 1 position relative to the robot
    Y2 = data(t, 19:21);% landmark 2 position relative to the robot
    landmark_ids = [data(t,3), data(t,22)];

    %=================================================
    % data *not* available to your filter, i.e., known
    % only by the simulator, useful for making error plots
    %=================================================
    % actual position (i.e., ground truth)
    x = data(t,10);
    y = data(t,11);
    theta = data(t,12);

    % noisefree observation
    noisefreeBearing = data(t, 7);

    %=================================================
    % graphics
    %=================================================
    figure(GLOBAL_FIGURE); clf; hold on; plotfield(observation(2));

    % draw actual path and path that would result if there was no noise in
    % executing the motion command
    plot([initialStateMean(1) data(1,10)], [initialStateMean(2) data(1,11)], 'Color', ACTUAL_PATH_COL);
    plot([initialStateMean(1) data(1,13)], [initialStateMean(2) data(1,14)], 'Color', NOISEFREE_PATH_COL);

    % draw actual path (i.e., ground truth)
    plot(data(1:t,10), data(1:t,11), 'Color', ACTUAL_PATH_COL, 'linewidth', 2);
    plotrobot(x, y, theta, 'black', 1, 'cyan');

    % draw noise free motion command path
    plot(data(1:t,13), data(1:t,14), 'Color', NOISEFREE_PATH_COL, 'linewidth', 2);
    plot(data(t,13), data(t,14), '*', 'Color', NOISEFREE_PATH_COL, 'linewidth', 2);

    % indicate observed angle relative to actual position
    plot([x x+cos(theta+observation(1))*100], [y y+sin(theta+observation(1))*100], 'Color', OBSERVED_BEARING_COLOR, 'linewidth', 2);

    % indicate ideal noise-free angle relative to actual position
    plot([x x+cos(theta+noisefreeBearing)*100], [y y+sin(theta+noisefreeBearing)*100], 'Color', NOISEFREE_BEARING_COLOR, 'linewidth', 2);
    set(gca, 'fontsize', 14)
    drawnow limitrate

    %=================================================
    %TODO: update your filter here based upon the
    %      motionCommand and observation
    %=================================================
    
    switch filter_name
        case {"EKF", "UKF"}
           filter.prediction(motionCommand);
           filter.correction(observation);
           draw_ellipse(filter.mu(1:2), filter.Sigma(1:2,1:2),9)
           results(:,t) = mahalanobis(filter,data(t,10:12));
        case "PF"
            hp = plot(filter.particles(1,:), filter.particles(2,:),'.','Color', [[0.2980 .6 0], .25]);
            filter.prediction(motionCommand);
            filter.correction(observation);
            set(hp,'XData',filter.particles(1,:),'YData', filter.particles(2,:));
            draw_ellipse(filter.mu(1:2), filter.Sigma(1:2,1:2), 9)
            results(:,t) = mahalanobis(filter,data(t,10:12));
        case "InEKF"
            filter.prediction(motionCommand)
            filter.correction(Y, Y2, landmark_ids);
            ellipse_plotter(filter);
            % Uncomment to run for bonus points
            lieTocartesian(filter);
            %Uncomment to run for bonus points
            results(:,t) = mahalanobis(filter,data(t,10:12));
    end
        

    if pauseLen == inf
        pause;
    elseif pauseLen > 0
        pause(pauseLen);
    end

    if makeVideo
        F = getframe(gcf);
        switch votype
          case 'avifile'
            vo = addframe(vo, F);
          case 'VideoWriter'
            writeVideo(vo, F);
          otherwise
            error('unrecognized votype');
        end
    end
end

figure; set(gca, 'fontsize', 14);
hold on; grid on
plot(results(1,:), 'linewidth', 2)
plot(7.185*ones(1,length(results)),'r', 'linewidth', 2)
legend('Chi-square Staistics','p = 0.05 w/ 3 DOF', 'fontsize', 14, 'location', 'best')

figure; set(gca, 'fontsize', 14)
subplot(3,1,1)
plot(results(2,:), 'linewidth', 2)
hold on; grid on
ylabel('X', 'fontsize', 14)
plot(results(5,:),'r', 'linewidth', 2)
plot(-1*results(5,:),'r', 'linewidth', 2)
legend('Deviation from Ground Truth','3rd Sigma Contour', 'fontsize', 14, 'location', 'best')
subplot(3,1,2)
plot(results(3,:), 'linewidth', 2)
hold on; grid on
plot(results(6,:),'r', 'linewidth', 2)
plot(-1*results(6,:),'r', 'linewidth', 2)
% plot(3*ones(1,length(results)),'r')
ylabel('Y', 'fontsize', 14)
subplot(3,1,3)
plot(results(4,:), 'linewidth', 2)
hold on; grid on
plot(results(7,:),'r', 'linewidth', 2)
plot(-1*results(7,:),'r', 'linewidth', 2)
% plot(3*ones(1,length(results)),'r')
ylabel('\theta', 'fontsize', 14)
xlabel('Iterations', 'fontsize', 14)

if nargout >= 1
    varargout{1} = data;
end
if nargout >= 2
    varargout{2} = results;
end

if makeVideo
    fprintf('Writing video...');
    switch votype
      case 'avifile'
        vo = close(vo);
      case 'VideoWriter'
        close(vo);
      otherwise
        error('unrecognized votype');
    end
    fprintf('done\n');
end
