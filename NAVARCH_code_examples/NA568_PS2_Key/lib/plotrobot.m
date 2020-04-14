function plotrobot(xpos, ypos, theta, color, filled, fillColor)
% PLOTROBOT

WAS_HOLD = ishold;

if ~WAS_HOLD
    hold on;
end

radius = 13;

h = plotcircle([xpos ypos], radius, 100, color, filled, fillColor);
if numel(h) > 1
    set(h(2), 'FaceAlpha', 0.25);
end

orientationLine = [xpos xpos+cos(theta)*(radius*1.5);
                   ypos ypos+sin(theta)*(radius*1.5)];

plot(orientationLine(1,:), orientationLine(2,:), ...
     'Color', 'black', 'LineWidth', 1.5);

if ~WAS_HOLD
    hold off;
end
