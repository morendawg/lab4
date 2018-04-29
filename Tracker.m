classdef Tracker < handle
    %Tracker 
    
    properties
        controller
    end
    
    properties (Dependent)
        waypoints
    end
    
    methods
        function obj = Tracker(des_linear_vel, max_angular_vel, look_ahead_dist)
            %Tracker Uses matlab ROS toolbox's PurePursuit class to track
            %waypoints
            obj.controller = robotics.PurePursuit;
            obj.controller.MaxAngularVelocity = max_angular_vel;
            obj.controller.LookaheadDistance = look_ahead_dist;
            obj.controller.DesiredLinearVelocity = des_linear_vel;
        end
        
        function vel_cmd = generate_vel_cmd(obj, pose)
            % Generate velocity command give current robot pose
            [v, w] = obj.controller(pose);
            vel_cmd = [v; w];
        end
        
        function set.waypoints(obj, waypoints)
            % Set waypoints for internal controller
            % WAYPOINTS: 2xN waypoints to follow
            obj.controller.Waypoints = waypoints';
        end
        
        function waypoints = get.waypoints(obj)
            waypoints = obj.controller.Waypoints';
        end
    end
end

