classdef PF < handle
    properties
        gfun;               %Motion model function
        hfun;               %Measurement Model Function
        Q;                  %Sensor Noise
        M;                  %Motion Model Noise (dynamical and function of input)
        n;                  %Number of Particles
        particles;          %Pose of particle
        particle_weight;    %Particle Weight
        mu;
        Sigma;
    end
    
    methods
        function obj = PF(sys, init)
            % motion model
            obj.gfun = sys.gfun;
            % measurement model
            obj.hfun = sys.hfun;
            % motion noise covariance
            obj.M = sys.M;
            % measurement noise covariance
            obj.Q = sys.Q;
            % PF parameters
            obj.mu = init.mu;
            obj.Sigma = init.Sigma;
            obj.n = init.n;
            obj.particles = init.particles;
            obj.particle_weight = init.particle_weight;
        end
        function prediction(obj, u)
            u_noise_std = chol(obj.M(u), 'lower');
            for j = 1:obj.n
                %Propagate Particles through Motion Model
                obj.particles(:,j) = obj.gfun(obj.particles(:,j),u_noise_std*randn(3,1)+u);
            end
            
        end
        
        
        function correction(obj, z)
            global FIELDINFO;
            landmark_x = FIELDINFO.MARKER_X_POS(z(3));
            landmark_y = FIELDINFO.MARKER_Y_POS(z(3));
%             u_noise_std = chol(obj.M(u), 'lower');
            weight = zeros(obj.n,1);
            for j = 1:obj.n
                %Propagate Particles through Motion Model
%                 obj.particles(:,j) = obj.gfun(obj.particles(:,j),u_noise_std*randn(3,1)+u);
                %Calculate measurement and difference in measurements
                z_hat = obj.hfun(landmark_x, landmark_y, obj.particles(:,j));
                diff = [...
                    wrapToPi(z(1) - z_hat(1));
                    z(2) - z_hat(2)];
                %Use mvnpdf to get weight probability of difference in
                %measurement
                weight(j) = mvnpdf(diff, 0, 1.5*obj.Q);
            end
            %Update Weights
            obj.particle_weight = obj.particle_weight.*weight;
            obj.particle_weight = obj.particle_weight./sum(obj.particle_weight);
            Neff = 1 / sum(obj.particle_weight.^2);
            if Neff < obj.n /5
                obj.resample();
            end
            obj.meanAndVariance();
        end 
         
        function resample(obj)
            newSamples = zeros(size(obj.particles));
            newWeight = zeros(size(obj.particle_weight));
            W = cumsum(obj.particle_weight);
            r = rand/obj.n;
            count = 1;
            for j = 1:obj.n
                u = r+(j-1)/obj.n;
                while u > W(count)
                    count = count+1;
                end
                newSamples(:,j) = obj.particles(:,count);
                newWeight(j) = 1/obj.n;
            end
            obj.particles = newSamples;
            obj.particle_weight = newWeight;
        end
        
        function meanAndVariance(obj)
            obj.mu = mean(obj.particles, 2); 
            % orientation is a bit more tricky.
            sinSum = 0;
            cosSum = 0;
            for s = 1:obj.n
                cosSum = cosSum + cos(obj.particles(3,s));
                sinSum = sinSum + sin(obj.particles(3,s));
            end
            obj.mu(3) = atan2(sinSum, cosSum);     
            % Compute covariance.
            zeroMean = obj.particles - repmat(obj.mu, 1, obj.n);
            for s = 1:obj.n
                zeroMean(3,s) = wrapTo2Pi(zeroMean(3,s));
            end
            
            obj.Sigma = zeroMean * zeroMean' / obj.n;
        end
    end
end