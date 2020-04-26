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
            A(1:3, 1:3) = - skew(omega);
            A(4:6, 1:3) = - skew(accel);
            A(4:6, 4:6) = - skew(omega);
            A(7:9, 4:6) = eye(3);
            A(7:9, 7:9) = -skew(omega); 

            obj.Sigma = A * obj.Sigma + obj.Sigma * A' + obj.Q;
            
        end
        
        function correction(obj, gps_measurement)
            gps = [gps_measurement, 0, 1]';
            H = zeros(5, 9);
            H(1:3, 7:9) = eye(3);
            gpsNoise = 1; %meters, initially constant
            covariance_v = [eye(3).*gpsNoise^2, zeros(3,2); zeros(2,5)];
            covariance_v(4,4) = 1;
            covariance_v(5,5) = 1;
            N = inv(obj.mu) * covariance_v * (inv(obj.mu))';
            % N = N(1:3, 1:3);
            S = H * obj.Sigma * H' + N;
            L = obj.Sigma * H' * inv(S);
            b = [0 0 0 0 1]';

            % unsolved expm(mu), should map mu 1 by 9 to lie group, 9 by 9 ?


            % check slide 69
            zai_hat = zeros(5);
            % zai 9 by 9
            zai = L * (obj.mu * gps - b);
            phi = zai(1:3); 
            rho1 = zai(4:6);
            rho2 = zai(7:9);
            % check slide 66, assume theta in sphererical coordinate
            % deal with special condition, 1e-9 a threshold for small value, changeable
            jacobian_phi = eye(3);
            if phi(3) > 1e-9
                theta = atan(sqrt(phi(1)^2 + phi(2)^2) / phi(3));
                if theta < 1e-9
                    jacobian_phi = jacobian_phi + skew(phi);
                else
                    jacobian_phi = jacobian_phi + (1 - cos(theta)) / theta^2 * skew(phi) ...
                    + (theta - sin(theta)) / theta^3 * (skew(phi)^2);
                end
            end
           
            zai_hat(1:3, 1:3) = expm(skew(phi));
            zai_hat(1:3, 4) = jacobian_phi * rho1;
            zai_hat(1:3, 5) = jacobian_phi * rho2;
            zai_hat(4:5, 4:5) = eye(2);
            
            obj.mu = zai_hat * obj.mu;

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