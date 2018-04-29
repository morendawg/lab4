classdef SimInterface < handle
    % SIMINTERFACE
    
    properties (Constant)
        dt = 0.1           % rate of simulation
        k = 1
        pause_t = 1e-4     % time to pause for drawing
    end
    
    properties
        time       % current time
        n_frame    % number of frames
        max_time   % max time to simulate
        vel_cmd    % previous vel_cmd
        
        robot      % Robot (with camera)
        world      % World
    end
    
    properties (Dependent)
        done    % simulation done
    end
    
    methods
        function obj = SimInterface(max_time, robot, world)
            % Constructor
            obj.time = 0.0;
            obj.n_frame = 0;
            obj.max_time = max_time;
            assert(max_time > 0, 'max_time must be > 0');
            obj.vel_cmd = [0; 0];
            
            obj.robot = robot;
            obj.world = world;
        end
        
        function meas = measurement(obj)
            meas.time = obj.time;
            % ground truth pose of the robot, only used for simulation
            meas.pose = obj.robot.pose(:);
            % odometry from the robot
            meas.odom = obj.robot.pose(:);
            % transformation from camera to robot
            meas.pose_BC = obj.robot.pose_BC(:);
            % the vel_cmd that is sent previously
            meas.vel_cmd = obj.vel_cmd(:);
            
            % Do measurement here
            % Landmark measurement
            % transform landmarks in world frame to camera frame
            % T_CW = (T_WB * T_BC)^-1
            pose_CW = se2_inverse(se2_oplus(obj.robot.pose, obj.robot.pose_BC));
            landmarks_C = se2_transform(pose_CW, obj.world.landmarks);
            [ids, range_bearings] = obj.robot.camera.measure_landmarks(landmarks_C);
            
            % ids of observed landmarks, Mx1
            meas.landmark_ids = ids(:);
            % range and bearing measurements, 2xM
            meas.range_bearings = range_bearings;
            % position of all landmarks in the world, 2XN
            meas.landmarks = obj.world.landmarks;
            
            % Obstacle measurement
            % transform all obstacles in world frame to camera frame
            line_segs_C = [se2_transform(pose_CW, obj.world.line_segs(1:2, :));
                           se2_transform(pose_CW, obj.world.line_segs(3:4, :))];
	        meas.scan = obj.robot.camera.measure_obstacles(line_segs_C);
            meas.max_range = obj.robot.camera.max_range;
        end
        
        function update(obj, vel_cmd)
            % check_vel_cmd(vel_cmd)
            
            % increment frame number and time
            obj.n_frame = obj.n_frame + 1;
            obj.time = obj.time + obj.dt;
            
            % early termination
            if obj.done
                return;
            end
            
            % update robot pose and traj and save vel_cmd
            obj.vel_cmd = check_vel_cmd(vel_cmd);
            for i = 1:obj.k
                obj.robot.update(obj.vel_cmd, obj.dt / obj.k);
            end
            
            pause(obj.pause_t);
        end
        
        function string = title_str(obj)
            string = sprintf('frame: %d, time: %.2f', obj.n_frame, obj.time);
        end
        
        function done = get.done(obj)
            % Checks whether simulation is done
            done = obj.time >= obj.max_time;
        end
    end
end