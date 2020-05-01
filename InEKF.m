classdef InEKF < handle   
    properties
        mu;                 % Pose Mean
        Sigma;              % Pose Sigma
        mu_pred;             % Mean after prediction step
        Sigma_pred;          % Sigma after prediction step
    end
    
    methods
        function obj = InEKF(init)
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u)
            g = [0; 0; 9.81];
            dt = 1 / 10;
            R_k = obj.mu(1:3, 1:3);
            v_k = obj.mu(1:3, 4);
            p_k = obj.mu(1:3, 5);
            a_k = u(1:3);
            omega_k = u(4:6);

            R_pred = R_k * expm(skew(omega_k * dt));
            v_pred = v_k + (R_k * Gamma_1(omega_k * dt) * a_k' + g) *dt;
            p_pred = p_k +  v_k * dt + 0.5 * (2 * R_k *  Gamma_2(omega_k * dt) * a_k' + g) * dt ^2;
            
            H_pred = [R_pred, v_pred, p_pred;
                        zeros(1,3), 1, 0;
                        zeros(1,3), 0, 1];
                       
            obj.propagation(H_pred, a_k, omega_k);
        end

        function propagation(obj, H_pred, a_k, omega_k)
            dt = 1 / 10;
            obj.mu_pred = H_pred;

            % log linear
            A = zeros(9); 
            A(1:3, 1:3) = - skew(omega_k);
            A(4:6, 1:3) = - skew(a_k);
            A(4:6, 4:6) = - skew(omega_k);
            A(7:9, 4:6) = eye(3);
            A(7:9, 7:9) = -skew(omega_k); 

            phi = expm(A * dt);
            obj.Sigma_pred = obj.Sigma + phi * eye(9) * phi';
        end
        
        function correction(obj, gps_measurement)
            b = [0; 0; 0; 0; 1];
            H = [zeros(3), zeros(3), eye(3)];

            % N just a covariance, so instead of doing the covariance stuff, just a 3 by 3
            N = eye(3).*0.5;             
            Y = [gps_measurement'; 0; 1];
            nu = obj.mu_pred \ Y - b; 
            S = H * obj.Sigma_pred * H' + N;
            K = obj.Sigma_pred * H' * (S \ eye(size(S)));
            
            % calculate zai
            zai = K * nu(1:3);
            zai_hat = zeros(5);
            phi = zai(1:3); 
            rho1 = zai(4:6);
            rho2 = zai(7:9);
            jacobian_phi = eye(3);
            theta = norm(phi);
            jacobian_phi = jacobian_phi + (1 - cos(theta)) / theta^2 * skew(phi) ...
                + (theta - sin(theta)) / theta^3 * (skew(phi)^2);
            zai_hat(1:3, 1:3) = expm(skew(phi));
            zai_hat(1:3, 4) = jacobian_phi * rho1;
            zai_hat(1:3, 5) = jacobian_phi * rho2;
            zai_hat(4:5, 4:5) = eye(2);

            obj.mu = obj.mu_pred * zai_hat;

            obj.Sigma = (eye(9) - K * H) * obj.Sigma_pred * (eye(9) - K * H)' + K * N * K';
        end
    end
end