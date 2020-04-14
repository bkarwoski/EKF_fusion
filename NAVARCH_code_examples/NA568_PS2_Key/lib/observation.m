%-------------------------------------------------------
% returns the observation of the specified marker given the current state
%-------------------------------------------------------
function obs = observation(state, id)
global FIELDINFO;

% Compute expected observation.
dx = FIELDINFO.MARKER_X_POS(int32(id)) - state(1);
dy = FIELDINFO.MARKER_Y_POS(int32(id)) - state(2);

obs = [...
    wrapTo2Pi(atan2(dy, dx) - state(3));
    sqrt(dx^2 + dy^2)
    id ];
end
