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
        
%         function prediction(obj, u)
%             
%         end

%         function out = Gamma_0(phi)
%             %     n = norm(phi);
%             %     out = eye(size(phi,1)) + sin(n) / n .* skew(phi) +...
%             %              (1 - cos(n)) ./ skew(phi)^2;
%             out = eye(3);
%         end
        
%         function out = Gamma_1(phi)
%             %     n = norm(phi);
%             %     out = eye(size(phi,1)) + (1 - cos(n)) / n^2 * skew(phi) +...
%             %           (n - sin(n)) / n^3 * skew(phi)^2;
%             out = eye(3);
%         end
        
%         function out = Gamma_2(phi)
%             %todo: implement this
%             out = eye(3);
%         end
        
        
        % define SE(3) methods
%         function X = skew(x)
%             % vector to skew R^3 -> so(3)
%             X = [   0,  -x(3),  x(2);
%                 x(3),      0,  -x(1);
%                 -x(2), x(1),   0];
%         end
        
%         function x = unskew(X)
%             % so(3) -> R^3
%             x = [X(3,2); X(1,3); X(2,1)];
%         end
        
%         function X = hat(x)
%             % hat: R^6 -> se(3)
%             X = [skew(x(4:6)), x(1:3); 0 0 0 0];
%         end
%         
%         function x = wedge(X)
%             % wedge: se(3) -> R^6
%             x = [X(1:3,4); unskew(X(1:3,1:3))];
%         end

    end