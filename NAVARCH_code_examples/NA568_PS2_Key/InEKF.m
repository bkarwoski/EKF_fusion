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
            
            state(1) = obj.mu(1,3);
            state(2) = obj.mu(2,3);
            state(3) = atan2(obj.mu(2,1), obj.mu(1,1));
            H_prev = obj.posemat(state);
            state_pred = obj.gfun(state, u);
            H_pred = obj.posemat(state_pred);
            
            u_se2 = logm(H_prev \ H_pred);

            Adjoint = @(X) [X(1:2,1:2), [X(2,3); -X(1,3)]; 0 0 1];
            AdjX = Adjoint(H_prev);
            
            obj.propagation(u_se2, AdjX);
        end
        
        function propagation(obj, u, AdjX)
            % SE(2) propagation model; the input is u \in se(2) plus noise
            % propagate mean
            obj.mu_pred = obj.mu * expm(u);
            % propagate covariance
            obj.Sigma_pred = obj.Sigma + AdjX * diag([0.05^2, 0.1^2, 0.1^2]) * AdjX';
        end
        
        function correction(obj, Y, Y2, landmark_ids)
            global FIELDINFO;        
            landmark_x = FIELDINFO.MARKER_X_POS(landmark_ids(1));
            landmark_y = FIELDINFO.MARKER_Y_POS(landmark_ids(1));       
            landmark_x2 = FIELDINFO.MARKER_X_POS(landmark_ids(2));
            landmark_y2 = FIELDINFO.MARKER_Y_POS(landmark_ids(2));
            
            G1 = [...
                0     0     1;
                0     0     0;
                0     0     0];
            
            G2 = [...
                0     0     0;
                0     0     1;
                0     0     0];
            
            G3 = [...
                0    -1     0;
                1     0     0;
                0     0     0];
            
            b = [landmark_x;landmark_y;1];
            b2 = [landmark_x2;landmark_y2;1];
            
            H = [...
                -1  0 landmark_y;
                0 -1 -landmark_x];
            H2 = [...
                -1  0 landmark_y2;
                0 -1 -landmark_x2];
            
            H = [H;H2];
            
            
            N = blkdiag([10000 0 ;0 10000],[10000 0 ;0 10000]);
            
            R = obj.mu_pred(1:2,1:2);
            R = blkdiag(R, R);
            nu1 = obj.mu_pred*Y' - b ;
            nu2 = obj.mu_pred*Y2' - b2;
                       
            N = R * N * R';
            
            
            S = H * obj.Sigma_pred * H' + N;
            K = obj.Sigma_pred * H' * (S \ eye(size(S)));
            
            delta1 = K(1:3,1:2) *[eye(2),zeros(2,1)]* nu1;
            delta2 = K(1:3,3:4) *[eye(2),zeros(2,1)]* nu2;
            
            delta = delta1 + delta2;
            obj.mu = expm(delta(1) * G1 + delta(2) * G2 + delta(3) * G3)*obj.mu_pred;
            
            obj.Sigma = (eye(3) - K * H) * obj.Sigma_pred * (eye(3) - K * H)' + K * N * K';
        end
        
        function H = posemat(obj,state)
            x = state(1);
            y = state(2);
            h = state(3);
            % construct a SE(2) matrix element
            H = [...
                cos(h) -sin(h) x;
                sin(h)  cos(h) y;
                     0       0 1];
        end
    end
end
