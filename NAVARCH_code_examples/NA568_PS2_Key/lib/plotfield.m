% highlights the detectedmarker id

function plotfield(detectedMarker)
% PLOTFIELD

global FIELDINFO;

WAS_HOLD = ishold;

if ~WAS_HOLD
    hold on;
end

margin = 200;

axis equal;
axis([-margin, FIELDINFO.COMPLETE_SIZE_X+margin, -margin, FIELDINFO.COMPLETE_SIZE_Y+margin]);

for k = 1:6
    if k == detectedMarker
        plotcircle([FIELDINFO.MARKER_X_POS(k), FIELDINFO.MARKER_Y_POS(k)], ...
                   15, 200, 'black',1, [0.8 0.8 0.8]);
    else
        plotcircle([FIELDINFO.MARKER_X_POS(k), FIELDINFO.MARKER_Y_POS(k)], ...
                   15, 200, 'black',1, 'white');
    end
    text(FIELDINFO.MARKER_X_POS(k)-2, FIELDINFO.MARKER_Y_POS(k), num2str(k));
end

if ~WAS_HOLD
    hold off;
end

