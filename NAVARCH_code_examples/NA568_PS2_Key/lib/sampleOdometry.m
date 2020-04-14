%-------------------------------------------------------
% noisy version of prediction
% predicts the new state given the current state and motion
% motion in form of [drot1,dtrans,drot2]
%-------------------------------------------------------
function [state, noisymotion] = sampleOdometry(motion, state, alphas)

Trans_vel = motion(1);
Angular_vel = motion(2);


noisymotion(1)=mvnrnd(motion(1),alphas(1)*Trans_vel^2+alphas(2)*Angular_vel^2);
noisymotion(2)=mvnrnd(motion(2),alphas(3)*Trans_vel^2+alphas(4)*Angular_vel^2);
noisymotion(3)=mvnrnd(motion(3),alphas(5)*Trans_vel^2+alphas(6)*Angular_vel^2); 


state = prediction(state, noisymotion);
