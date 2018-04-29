classdef Robot < handle
    % ROBOT

    properties
        pose      % pose of the robot [x; y; theta]
        pose_BC   % pose of camera in base frame  
        camera    % camera
        noise     % noise in vel_cmd (std, not cov)
    end  % properties
    
    properties (Dependent, SetAccess = private)
        x 
        y
        theta
    end
    
    properties (Constant)
        radius = 0.3;
    end
    
    methods
        
        function obj = Robot(init_pose, noise)
            obj.pose = init_pose(:);
            obj.camera = [];
            obj.pose_BC = [];
            obj.noise = noise;
        end
        
        function update(obj, vel_cmd, dt)
            % make noise proportional to the actual command
            vx = vel_cmd(1) + randn * obj.noise(1) * vel_cmd(1);
            wz = vel_cmd(2) + randn * obj.noise(2) * vel_cmd(2);
            obj.x = obj.x + dt * vx * cos(obj.theta);
            obj.y = obj.y + dt * vx * sin(obj.theta);
            obj.theta = wrapToPi(obj.theta + dt * wz);
        end
        
        function add_camera(obj, camera, pose_BC)
            obj.camera = camera;
            obj.pose_BC = pose_BC;
        end
        
        % Getter/Setter
        function x = get.x(obj)
            x = obj.pose(1);
        end
        
        function set.x(obj, x)
            obj.pose(1) = x;
        end
        
        function y = get.y(obj)
            y = obj.pose(2);
        end
        
        function set.y(obj, y)
            obj.pose(2) = y;
        end
        
        function theta = get.theta(obj)
            theta = obj.pose(3);
        end
        
        function set.theta(obj, theta)
            obj.pose(3) = wrapToPi(theta);
        end
        
    end  % methods
    
end % classdef Robot 