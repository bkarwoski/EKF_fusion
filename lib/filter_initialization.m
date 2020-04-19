function filter = filter_initialization(sys, initialStateMean,initialStateCov)
%initialStateMean should be 5x5
%initialStateCov should be 9x9
%Note: This script exists for legacy reasons
init.mu = initialStateMean;
init.Sigma = initialStateCov;
filter = InEKF(sys, init);

%From SE(2) example:
%         init.mu = eye(3);
%         init.mu(1,3) = initialStateMean(1);
%         init.mu(2,3) = initialStateMean(2);
%         init.Sigma = initialStateCov;
%         filter = InEKF(sys, init);
end