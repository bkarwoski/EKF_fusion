classdef UKF < handle
    properties
        mu;             % Pose Mean
        Sigma;          % Pose Covariance
        gfun;           % Motion Model Function
        hfun;           % Measruement Model Function
        M;              % Motion model noise(dynamical and function of input)
        Q;              % Sensor Noise
        kappa_g;        
        mu_pred;
        Sigma_pred;
        n;
        X;
        w;
        Y;
    end
    
    methods
        function obj = UKF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            obj.kappa_g = init.kappa_g;
            % initial mean and covariance
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
        end
        
        function prediction(obj, u)
            %Create Sigma Points
            Mean_Aug = [obj.mu;0;0;0;0;0];
            Sigma_Aug(1:3,1:3) = obj.Sigma;
            Sigma_Aug(4:6,4:6) = obj.M(u);
            Sigma_Aug(7:8,7:8) = 2*obj.Q;
            obj.sigma_point(Mean_Aug, Sigma_Aug, obj.kappa_g);
            %Propagate Sigma Points through Non-linear map
            obj.mu_pred = zeros(3,1);
            obj.Y = zeros(3,2*obj.n+1);
            for j = 1:2*obj.n+1
                obj.Y(:,j) = obj.gfun(obj.X(1:3,j), u + obj.X(4:6,j));

                obj.mu_pred = obj.mu_pred + obj.w(j) * obj.Y(:,j);
            end
            %Update Covariance
            obj.Sigma_pred = (obj.Y - obj.mu_pred) * diag(obj.w) * (obj.Y - obj.mu_pred)';
        end
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
            z_hat = [0;0];
            Z = zeros(2, 2*obj.n+1);
            for j = 1:2*obj.n+1
                Z(:,j) = obj.hfun(landmark_x, landmark_y, obj.Y(:,j)) + obj.X(7:8,j);
                z_hat = z_hat + obj.w(j) * Z(:,j);
            end
            %Calculate Innovation
            S = (Z-z_hat) * diag(obj.w) * (Z-z_hat)';
            Sigma_xz = (obj.Y - obj.mu_pred) * diag(obj.w) * (Z - z_hat)';
            %Calculate Kalman Gain
            K = Sigma_xz*S^-1;
            diff = [...
                wrapToPi(z(1) - z_hat(1));
                z(2) - z_hat(2)];
            obj.mu = obj.mu_pred + K*diff;
            obj.Sigma = obj.Sigma_pred - K * S * K';
        end
        
        function sigma_point(obj, mean, cov, kappa)
            obj.n = numel(mean);
            L = sqrt(obj.n + kappa) * chol(cov,'lower');
            obj.Y = mean(:,ones(1, numel(mean)));
            obj.X = [mean,obj.Y + L, obj.Y - L];
            obj.w = zeros(2 * obj.n + 1,1);
            obj.w(1) = kappa / (obj.n + kappa);
            obj.w(2:end) = 0.5 / (obj.n + kappa);
        end
    end
end