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
            
        end
    end