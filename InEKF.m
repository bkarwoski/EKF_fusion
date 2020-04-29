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
           pose_prev = obj.mu
           obj.mu = imuDynamics(obj.mu, u, 1/30); %TODO not hardcode dT
           pose_next = obj.mu
           %todo remove state_pred, just use state?
           invprev_next = pose_prev \ pose_next
           u_se3 = logm(invprev_next)
           obj.propagation(wedge(u_se3));
        end
        
        function propagation(obj, u) %propagation not needed
            %u is in se(3)
            %what are the shapes of u, obj.mu?
                % u is square, mu's 2nd dim == u's first dim
            %propagate meanS
            %obj.mu_pred = obj.mu * expm(u);
            %propagate covariance

            % from slide 36, u doesn't need to be se(3), using raw IMU here.
            
            omega = u(4:6);
            accel = u(1:3);
            prev_mu=obj.mu;
            obj.mu(1:3, 1:3) = obj.mu(1:3, 1:3) * skew(omega);
            obj.mu(1:3, 4) = obj.mu(1:3, 1:3) * accel' +[0 0 -9.81]';
            obj.mu(1:3, 5) = obj.mu(1:3, 5);
            obj.mu=prev_mu+obj.mu*1/30;
            
            A = zeros(9); 
            A(1:3, 1:3) = - skew(omega);
            A(4:6, 1:3) = - skew(accel);
            A(4:6, 4:6) = - skew(omega);
            A(7:9, 4:6) = eye(3);
            A(7:9, 7:9) = -skew(omega); 

            obj.Sigma = (A * obj.Sigma + obj.Sigma * A' + obj.Q) * 1/30 + obj.Sigma;
            
        end
        
        function correction(obj, gps_measurement)
            gps = [gps_measurement, 0, 1]'; 
            
            H = zeros(5, 9);
            H(1:3, 7:9) = eye(3);
            gpsNoise = 1; %meters, initially constant
            % covariance should be 5 instead of 3 so that N, hence L could match 
            % the dimension when calculating mu and sigma
            covariance_v = [eye(3).*gpsNoise^2, zeros(3,2); zeros(2,5)];
            covariance_v(4,4) = 1;
            covariance_v(5,5) = 1;
            N = inv(obj.mu) * covariance_v * (inv(obj.mu))';
            % N = N(1:3, 1:3);
            S = H * obj.Sigma * H' + N;
            L = obj.Sigma * H' * inv(S);
            b = [0 0 0 0 1]';

            % map mu 1 by 9 to lie group, 5 by 5
            % check slide 69
            % zai 3(K+1) vector, hence K is 2, and zai_hat should be 5 by 5 since mu 5 by 5
            zai_hat = zeros(5);
            % zai 9 by 1
            zai = L * (obj.mu * gps - b);
            phi = zai(1:3); 
            rho1 = zai(4:6);
            rho2 = zai(7:9);

            % check slide 66
            % put phi_hat into so(3), and calculate the left jacobian
            jacobian_phi = eye(3);
            % deal with special condition, 1e-9 a threshold for small value, could change to smaller values
            e = 1e-9;
            if phi(3) > e
                % assume theta in spherical coordinate
                theta = atan(sqrt(phi(1)^2 + phi(2)^2) / phi(3));
                if theta < e
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
            
            obj.mu = obj.mu * zai_hat; %order seems wrong

            obj.Sigma = (eye(9) - L * H) * obj.Sigma * (eye(9) - L * H)' ...
                + L * N * L';    
        end
        
%         function Rt = posemat(obj, mu) %Not sure if this function makes sense
%             Rt =  zeros(5);
%             Rt(1:3, 1:3) = mu(1:3, 1:3);
%             Rt( = mu(1:3, 5);
%             Rt = [R, t; 0 0 0 1];
%         end
    end
end
