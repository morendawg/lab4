classdef ParticleFilter < handle
    %PARTICLEFILTER 
    %omkar and delaney
    
    properties (Constant)
        dof = 3;
        type = 'pf';
    end
    
    properties
        state               % 3x1, (x, y, theta)'
        state_hist          % 3xT, history of all states (T is # of steps)

        num_particles       % 1x2, number of particles
        weights             % Nx1, weights for each pariticles
        particles           % 3xN, all particles
        
        process_noise       % 2x2, process noise covariance
        measurement_noise   % 2x2, measurement noise covariance
        initialized         % 1x1, indicates whether filter is initialized or not
        
        % Used for visualizing uncertainty
        state_cov           % 3x3, state covariance
        state_var_hist      % 3xT, history of all state variances
    end
    
    properties (Dependent)
        quivers             % used for visualizing particles
    end
    
    properties (Constant)
        quiver_length = 0.3 % used for visualizing particles
    end
    
    methods
        function obj = ParticleFilter(num_particles)
            %PARTICLEFILTER Constructor
            % INPUTS:
            %   num_particles: number of particles, 100 is a good number to
            %   start with
            obj.num_particles = num_particles;
            obj.initialized = false;
        end
        
        function initialize(obj, init_state, init_state_noise, process_noise, measurement_noise)
            % INITIALIZE initialize particle filter
            % INPUTS:
            %   init_state: 3x1, initial pose
            %   init_state_noise: 3x1, initial state noise std
            %   process_noise: 2x1, process noise std
            %   measurement_noise: 2x1, measurement noise std 
            obj.state = init_state;
            obj.state_cov = diag(init_state_noise.^2);
            obj.process_noise = process_noise(:);
            obj.measurement_noise = measurement_noise(:);
            
            % ===== YOUR CODE STARTS HERE =====
            % You need to do the following
            % 1. Generate num_particles particles according to
            % init_state and init_state_noise (gaussian distribution).
            
            %Multivariate normal distribution
            obj.particles = mvnrnd(obj.state, obj.state_cov, obj.num_particles)';
            
            % 2. Generate equal weights for each particle, all weights sum
            % to 1.
            
            %POSSIBLE
            obj.weights = ones(obj.num_particles, 1)./(obj.num_particles); 
            
            obj.state_hist = [obj.state_hist, obj.state];
            obj.state_var_hist = [obj.state_var_hist, diag(obj.state_cov)];
            obj.initialized = true;
        end
        
        function step(obj, U, Z, L, pose_BC, dt)
            % STEP run filter one step
            % INPUTS:
            %   U: 2x1, vel_cmd [v; w];
            %   Z: 2xM, range_bearing of observed landmarks
            %   L: 2xM, position of observed landmarks
            %   pose_BC: 3x1, transformation from camera to robot base
            %   dt: delta t
            
            if ~obj.initialized
                warning('Filter not initialized');
                return;
            end
            
            % main filter step
            obj.predict(U, dt);
            obj.correct(Z, L, pose_BC);
            
            % save hist
            obj.state_hist = [obj.state_hist, obj.state];
            obj.state_var_hist = [obj.state_var_hist, diag(obj.state_cov)];
        end
        
        function predict(obj, U, dt)
            %PREDICT prediction with motion model
            
            % ===== YOUR CODE STARTS HERE =====
            % Here you need to update all particles according to the robot
            % motion model, remeber to add random noise to the input U.
            
            %iterate through all particles
            
            od_noise = mvnrnd(U, diag(obj.process_noise).^2, obj.num_particles)';
            
            obj.particles(1, :) = obj.particles(1, :)  + od_noise(1, :) .* cos(obj.particles(3, :)) * dt;
            obj.particles(2, :) = obj.particles(2, :) + od_noise(1, :) .* sin(obj.particles(3, :)) * dt;
            obj.particles(3, :) = wrapToPi( obj.particles(3, :)  + od_noise(2, :) * dt );
            % ===== YOUR CODE ENDS HERE =====
            
            % reassign
            obj.state = weighted_state_mean(obj.particles, obj.weights);
            obj.state_cov = weighted_state_cov(obj.state, obj.particles, obj.weights);
        end
        
        function correct(obj, Z, L, pose_BC)
            % CORRECT, correction with measurements
            if isempty(Z)
                return;
            end
            
            % ===== YOUR CODE STARTS HERE =====
            % Here you need to weight each particles according the the
            % measurement and then resample the particles based on their
            % new weights. Essentially you need to compute p(z_i | x_j)
            % where i is the index of measurements and j is the index of
            % particles. 
            % Hint: You can compute one set of weights for each landmark
            % measurement and then multiply them together to get the final
            % weights.
            
            %1. Compute weights for particles

            
            %%END OD
            transformed_particle = se2_oplus(obj.particles, pose_BC);
            for i=1:obj.num_particles
                weight = [];
                for j=1:size(L,2)
                    part_x = transformed_particle(1,i);
                    part_y = transformed_particle(2,i);
                    part_theta = transformed_particle(3,i);
                    land_x = L(1,j);
                    land_y = L(2,j);
                    h1 = sqrt((land_x - part_x)^2 + (land_y - part_y)^2);
                    h2 = wrapToPi(atan2((land_y - part_y), land_x - part_x)-part_theta);
                    h = [h1;h2];
                    weight = [weight,mvnpdf(h, Z(:,j), diag(obj.measurement_noise).^2)];
                end
                obj.weights(i) = prod(weight);
            end
            %%END OD
            
            
            % 2. Resample particles and reset weights to be all equal
          
            particleIndeces = datasample([1:obj.num_particles], obj.num_particles, 'Weights', obj.weights(:,1));
            obj.particles(1,:) = obj.particles(1, particleIndeces);
            obj.particles(2,:) = obj.particles(2, particleIndeces);
            obj.particles(3,:) = obj.particles(3, particleIndeces);
            
            obj.weights = ones(obj.num_particles, 1)./(obj.num_particles);
            
            % ===== YOUR CODE ENDS HERE =====
            
            obj.state = weighted_state_mean(obj.particles, obj.weights);
            obj.state_cov = weighted_state_cov(obj.state, obj.particles, obj.weights);
        end
        
        function quivers = get.quivers(obj)
            U = cos(obj.particles(3, :)) * obj.quiver_length;
            V = sin(obj.particles(3, :)) * obj.quiver_length;
            quivers = [obj.particles(1:2, :); U; V];
        end
        
    end
end
