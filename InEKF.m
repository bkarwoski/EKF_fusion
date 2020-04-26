classdef InEKF < handle   
    properties
        mu;                 % Pose Mean
        Sigma;              % Pose Sigma
        mu_cart;
        sigma_cart;
        Q;                  %IMU accel and gyro noise
    end
    
    methods
        function obj = InEKF(init)
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
            obj.Q = init.Q;
        end
        
        function prediction(obj, u) 
           %u_se3 = logm(H_prev \ H_pred)
%            state = obj.Sigma; 
%            state_pred = obj.gfun(state, u);
           pose_prev = obj.posemat(obj.mu);
           obj.mu = imuDynamics(obj.mu, u, 1/30); %TODO not hardcode dT
           pose_next = obj.posemat(obj.mu);
           %todo remove state_pred, just use state?
           u_se3 = logm(pose_prev \ pose_next);
           obj.propagation(u_se3);
        end
        
        function propagation(obj, u) %propagation not needed
            %u is in se(3)
            %what are the shapes of u, obj.mu?
                % u is square, mu's 2nd dim == u's first dim
            %propagate mean
            %obj.mu_pred = obj.mu * expm(u);
            %propagate covariance

            % from slide 36, u doesn't need to be se(3), using raw IMU here.
            
            omega = u(5:7);
            accel = u(2:4);
            obj.mu(1:3, 1:3) = obj.mu(1:3, 1:3) * skew(omega);
            obj.mu(1:3, 4) = obj.mu(1:3, 1:3) * accel' +[0 0 -9.81]';
            obj.mu(1:3, 5) = obj.mu(1:3, 5);
            
            A = zeros(9); 
            A(1:3, 1:3) = - twisted(omega);
            A(4:6, 1:3) = - twisted(accel);
            A(4:6, 4:6) = - twisted(omega);
            A(7:9, 4:6) = eye(3);
            A(7:9, 7:9) = -twisted(omega); 

            obj.Sigma_pred = A * obj.Sigma + obj.Sigma * A' + obj.Q;
            
        end
        
        function correction(obj, gps_measurement)
            gps = [gps_measurement, 0, 1]';
            H = zeros(5, 9);
            H(1:3, 7:9) = eye(3);
            % covaraince_v is 5*5 with the top left 3*3 block needed.
            gpsNoise = 0.5; %meters
            covariance_v = [eye(3).*gpsNoise, zeros(3,2); zeros(2,5)];
            %TODO- update GPS covariance every timestep
            N = inv(obj.mu) * covariance_v * (inv(obj.mu))';
            S = H * obj.Sigma * H' + N; % S: 5*5
            L = obj.Sigma * H' * inv(S); % L: 9*5
            b = [0 0 0 0 1];
            obj.mu = expm(L * (obj.mu * gps - b')) * obj.mu;
            obj.Sigma = (eye(9) - L * H) * obj.Sigma * (eye(9) - L * H)' ...
                + L * N * L';    
        end
        
        function Rt = posemat(mu)
            R = mu(1:3, 1:3);
            t = mu(1:3, 5);
            Rt = [R, t; 0 0 0 1];
        end
    end
end