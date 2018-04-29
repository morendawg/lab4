classdef RobotViz < handle
    % ROBOTVIZ Visualizer for robot
    
    properties (Constant)
        color_robot = 'y'       % color of robot
        color_direction = 'r'   % color of direction
        color_traj = 'b'        % color of trajector
        color_camera = 'g'      % color of camera
        color_scan = 'b'        % color of scan
        alpha = 0.5             % alpha of color
    end  % properties constant
    
    properties
        hg_ax            % axis
        tf_WB            % transform base to world
        tf_BC            % transform camera to base
        hg_robot         % handle of shape of robot
        hg_direction     % handle of direction of robot
        hg_traj          % handle of traj of robot
        hg_camera        % patch of camera
        hg_scan
        traj            % trajectory of robot
        radius
    end  % properties
    
    methods
        
        function obj = RobotViz(ax, robot)
            % Constructor of RobotViz
            init_pose = robot.pose(:);
            obj.traj = init_pose;
            obj.radius = robot.radius;
            
            hold on;
            obj.hg_ax = ax;
            obj.tf_WB = hgtransform;
            obj.tf_WB.Matrix = makehgtform(...
                'translate', [init_pose(1), init_pose(2), 0], ...
                'zrotate', init_pose(3));
            
            % draw a circle that represents the robot
            t = linspace(0, 2 * pi);
            x = obj.radius * cos(t);
            y = obj.radius * sin(t);
            obj.hg_robot = patch(obj.hg_ax, ...
                'XData', x, 'YData', y, ...
                'FaceColor', obj.color_robot, ...
                'FaceAlpha', obj.alpha, ...
                'Parent', obj.tf_WB);
            
            % draw a triangle that represents the direction of robot
            half_r = obj.radius / 2;
            x = [0, 0, obj.radius];
            y = [-half_r, half_r, 0];
            obj.hg_direction = patch(obj.hg_ax, ...,
                'XData', x, 'YData', y, ...
                'FaceColor', obj.color_direction, ...
                'FaceAlpha', obj.alpha, ...
                'Parent', obj.tf_WB);
            
            % draw a line that represents the trajectory of robot
            obj.hg_traj = plot(obj.hg_ax, init_pose(1), init_pose(2), ...
                'Color', obj.color_traj);
            
            % draw camera if it exists
            if ~isempty(robot.camera)
                obj.tf_BC = hgtransform('Parent', obj.tf_WB);
                obj.tf_BC.Matrix = makehgtform(...
                    'translate', [robot.pose_BC(1), robot.pose_BC(2), 0], ...
                    'zrotate', robot.pose_BC(3));
                x = [0, 1, 1] * robot.camera.max_range;
                r = robot.camera.w / robot.camera.f;
                y = [0, r / 2, -r / 2] * robot.camera.max_range;
                obj.hg_camera = patch(obj.hg_ax, ...,
                    'XData', x, 'YData', y, ...
                    'FaceColor', obj.color_camera, ...
                    'FaceAlpha', obj.alpha, ...
                    'EdgeColor', 'none', ...
                    'Parent', obj.tf_BC);
                
                % scan plot
                scan_lines = robot.camera.scan_lines;
                
                obj.hg_scan = quiver(obj.hg_ax, ...
                    scan_lines(1, :), scan_lines(2, :), ...
                    scan_lines(3, :), scan_lines(4, :), ...
                    0, ...
                    'Color', obj.color_scan, ...
                    'ShowArrowHead', 'off', ...
                    'Parent', obj.tf_BC);
            end
            hold off;
        end
        
        function draw_pose(obj, pose)
            obj.traj = [obj.traj, pose];
            obj.tf_WB.Matrix = makehgtform(...
                'translate', [pose(1), pose(2), 0], 'zrotate', pose(3));
            obj.draw_traj(obj.traj);
        end
        
        function draw_scan(obj, scan)
            % Check how many scans are valid
            status = ~isnan(scan(1, :));
            n_valid = sum(status);
            
            if n_valid == 0
                obj.hg_scan.set('XData', 0, 'YData', 0, ...
                    'UData', 0, 'VData', 0);
                return;
            end
            
            % Only visualize valid scans
            ranges = scan(1, status);
            bearings = scan(2, status);
            
            u = ranges .* cos(bearings);
            v = ranges .* sin(bearings);
            obj.hg_scan.set('XData', zeros(1, n_valid), ...
                'YData', zeros(1, n_valid), ...
                'UData', u, 'VData', v);
        end
        
        function draw_traj(obj, traj)
            obj.hg_traj.set('XData', traj(1, :), 'YData', traj(2, :));
        end
    end  % methods
    
end  % classdef RobotViz
