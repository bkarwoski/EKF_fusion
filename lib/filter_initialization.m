function filter = filter_initialization(initialStateMean,initialStateCov, Q)
%initialStateMean should be 5x5
%initialStateCov should be 9x9
%Note: This script exists for legacy reasons
init.mu = initialStateMean;
init.Sigma = initialStateCov;
init.Q = Q;
filter = InEKF(init);
end