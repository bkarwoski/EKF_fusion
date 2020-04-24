classdef InEKF < handle   
    properties
        mu;                 % Pose Mean
        Sigma;              % Pose Sigma
        gfun;               % Motion model function
        mu_pred;             % Mean after prediction step
        Sigma_pred;          % Sigma after prediction step
        mu_cart;
        sigma_cart;
    end
    
    methods
        function obj = InEKF(sys, init)
            obj.gfun = sys.gfun;
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u) 
           %u is IMU data(?)
           %build state matrix from v, p, R mu values?
           %get state_pred from gfun(state, u)
           %how is H matrix different from state matrix?
           %u_se3 = logm(H_prev \ H_pred)
           state = obj.Sigma;
           state_pred = obj.gfun(state, u);
           
           %Adjoint function? Not necessary
           %Call adjoint anonymous function
           %call propagation(u_se3, AdjX)
        end
        
        function propagation(obj, u, AdjX)
           %u is in se(3)
           %what are the shapes of u, obj.mu?
            % u is square, mu's 2nd dim == u's first dim
           %propagate mean
           %obj.mu_pred = obj.mu * expm(u);
           %propagate covariance
        end
        
        function correction(obj, gps_measurement)
            H = zeros(5, 9);
            H(1:3, 7:9) = eye(3);
            % covaraince_v is 5*5 with the top left 3*3 block needed.
            N = inv(obj.mu) * covariance_v * (inv(obj.mu))';
            S = H * obj.Sigma * H' + N; % S: 5*5
            L = obj.Sigma * H' * inv(S); % L: 9*5
            b = [0 0 0 0 1];
            obj.mu = expm(L * (obj.mu_pred * gps_measurement - b')) * obj.mu_pred;
            obj.Sigma = (eye(9) - L * H) * obj.Sigma_pred * (eye(9) - L * H)' ...
                + L * N * L';    
        end
    end