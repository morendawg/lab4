classdef ParticleFilter < handle
    %PARTICLEFILTER Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        dof = 3;
        type = 'pf';
    end
    
    properties
        state               % 3x1, (x, y, theta)'
        state_cov           % 3x3, state covariance
        state_hist          % 3xT, history of all states (T is # of steps)
        state_var_hist      % 3xT, history of all state variances
        
        num_particles       % 1x2, number of particles
        weights             % Nx1, weights for each pariticles
        particles           % 3xN, all particles
        
        process_noise       % 2x2, process noise covariance
        measurement_noise   % 2x2, measurement noise covariance
        initialized         % 1x1, indicates whether filter is initialized or not
    end
    
    properties (Dependent)
        quivers
    end
    
    properties (Constant)
        quiver_length = 0.3
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
            obj.particles = create_normal_particles(init_state, init_state_noise, obj.num_particles);
            obj.weights = ones(obj.num_particles, 1) / obj.num_particles;
            obj.process_noise = process_noise(:);
            obj.measurement_noise = measurement_noise(:);
            obj.initialized = true;
            
            obj.state_hist = [obj.state_hist, obj.state];
            obj.state_var_hist = [obj.state_var_hist, diag(obj.state_cov)];
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
            n = obj.num_particles;
            Xs = obj.particles;
            Q = obj.process_noise;
            W = obj.weights;
            
            % Preterb input
            Us = U + diag(Q) * randn(2, n);
            % Update particles
            Ys = process_model(Xs, Us, dt);
            % Compute new state and state_cov
            Xn = weighted_state_mean(Ys, W);
            Pn = weighted_state_cov(Ys, Xn, W);
            
            % reassign
            obj.state = Xn;
            obj.state_cov = Pn;
            obj.particles = Ys;
        end
        
        function correct(obj, Z, L, pose_BC)
            % CORRECT, correction with measurements
            if isempty(Z)
                return;
            end
            
            [~, m] = size(L);
            
            Z = reshape(Z, [], 1);
            
            % rename for readability
            Xs = obj.particles;
            W = obj.weights;
            R = obj.measurement_noise;
            
            % apply measurement model to each particle
            Zp = measurement_model(Xs, L, pose_BC);
            % calculate p(z | x)
            Ws = normpdf(Zp, Z, repmat(R, m, 1));
            % assume independence
            W = W .* prod(Ws)';
            W = W / sum(W); % normalize

            % resample
            if neff(obj.weights) < obj.num_particles / 2
                indices = obj.resample(W);
                Xs = Xs(:, indices);
                W = ones(obj.num_particles, 1) / obj.num_particles;
            end
            
            % Compute new state and state_cov
            Xn = weighted_state_mean(Xs, W);
            Pn = weighted_state_cov(Xs, Xn, W);
            
            % reassign
            obj.state = Xn;
            obj.state_cov = Pn;
            obj.particles = Xs;
            obj.weights = W;
        end
        
        function indices = resample(obj, W)
            W_sum = [0; cumsum(W)];
            indices = discretize(rand(obj.num_particles, 1), W_sum);
        end
        
        function quivers = get.quivers(obj)
            U = cos(obj.particles(3, :)) * obj.quiver_length;
            V = sin(obj.particles(3, :)) * obj.quiver_length;
            quivers = [obj.particles(1:2, :); U; V];
        end
    end
end

function Xs = create_normal_particles(X, X_std, N)
Xs = randn(3, N);
Xs = Xs .* X_std(:) + X(:);
end

function n = neff(W)
n = 1 ./ sum(W.^2);
end
