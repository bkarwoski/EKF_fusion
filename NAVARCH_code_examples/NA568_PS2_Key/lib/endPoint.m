%-------------------------------------------------------
% endpoint of an observation
%-------------------------------------------------------
function pt = endPoint(pos, obs)

orientation = pos(3) + obs(2);
pt(1) = pos(1) + cos(orientation) * obs(1);
pt(2) = pos(2) + sin(orientation) * obs(1);
