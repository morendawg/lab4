% Author: Dan Moreno

classdef RobotControl < handle
    % ROBOTCONTROL This class is for student
    
    properties (Constant)
        k_rho = 0.1;
        k_beta = -0.1;
        k_alpha = 0.2;
        
        dt = 0.1;
    end
    
    properties
        filter
        gridmap
        tracker
        planner
        curr_ij
        old_ij
    end
    
    methods
        function obj = RobotControl(filter, gridmap, planner, tracker)
            % Constructor of RobotControl
            obj.filter = filter;
            obj.gridmap = gridmap;
            obj.planner = planner;
            obj.tracker = tracker;
            obj.curr_ij = [1; 1]; % subscript of the cell that the robot is currently in
        end
        
        function vel_cmd = process(obj, measurement)
            % Update filter
            obj.filter.step(...
                measurement.vel_cmd, ...
                measurement.range_bearings, ...
                measurement.landmarks(:, measurement.landmark_ids), ...
                measurement.pose_BC, ...
                obj.dt);
            
            pose = obj.filter.state;
            
            % Update gridmap
            pose_C = se2_oplus(pose, measurement.pose_BC);
            obj.gridmap.add_scan(pose_C, measurement.scan, measurement.max_range);
            
            % Check if we are close to goal
            if obj.planner.goal_reached(pose)
                vel_cmd = [0; 0];
                return;
            end
            
            % Check if we need to replan
            if obj.should_replan(pose)
                waypoints = obj.planner.plan(obj.gridmap, pose(1:2));
                obj.tracker.waypoints = waypoints;
            end
            
            vel_cmd = obj.tracker.generate_vel_cmd(pose);
            
            
        end
        
        function replan = should_replan(obj, pose)
            % SHOULD_REPLAN, check whether the planner should replan based
            % on whether the robot has moved from one cell to another
            % INPUTS:
            %   POSE: 3x1, robot pose in world frame
            % OUTPUTS:
            %   REPLAN: bool, true if needs to replan
            
            % YOUR CODE STARTS HERE =======================================
            obj.old_ij = obj.curr_ij;
            obj.curr_ij(1) = round(pose(1));
            obj.curr_ij(2) = round(pose(2));
            
            s = size(obj.gridmap.map);
            
            if(sub2ind(s, obj.curr_ij(1),obj.curr_ij(2)) ~= sub2ind(s,obj.old_ij(1),obj.old_ij(2)))
                replan = true;
                return
            end
            replan = false;
                
            % YOUR CODE ENDS HERE =========================================
        end
    end
end