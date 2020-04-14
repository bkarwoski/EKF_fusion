%-------------------------------------------------------
% predicts the new state given the current state and motion
% motion in form of [drot1,dtrans,drot2]
%-------------------------------------------------------
function state = prediction(state, motion)

% state(3)=state(3)+motion(1);
% state(1)=state(1)+motion(2)*cos(state(3));
% state(2)=state(2)+motion(2)*sin(state(3));
% state(3)=state(3)+motion(3);
% state(3)=minimizedAngle(state(3));
x = state(1);
y = state(2);
theta = state(3);
v = motion(1);
w = motion(2);
gamma = motion(3);
if w == 0
   x = x + cos(theta) * v;
   y = y + sin(theta) * v;
   theta = theta + gamma;
else
   x = x + (-v / w * sin(theta) + v / w * sin(theta + w));
   y = y + ( v / w * cos(theta) - v / w * cos(theta + w));
   theta = theta + w + gamma;
end
state = [x;y;theta];
state(3) = wrapTo2Pi(state(3));
end