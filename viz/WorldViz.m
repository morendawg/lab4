classdef WorldViz < handle
    %WORLDVIZ Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        landmark_size = 15;
        landmark_color = 'b';
        obstacle_color = 'm';
    end
    
    properties
        hg_ax
        hg_landmarks
        hg_landmarks_obs
        hg_obstacles
        world
    end
    
    methods
        function obj = WorldViz(ax, world)
            %WORLDVIZ Construct an instance of this class
            %   Detailed explanation goes here
            obj.world = world;
            obj.hg_ax = ax;
            
            hold on;
            obj.hg_landmarks = plot(obj.hg_ax, obj.world.landmarks_X, obj.world.landmarks_Y, ...
                '.', 'MarkerSize', obj.landmark_size);
            obj.hg_landmarks_obs = plot(obj.hg_ax, obj.world.landmarks_X, obj.world.landmarks_Y, ...
                'o', 'LineWidth', 2);
            obj.hg_landmarks_obs.set('XData', []', 'YData', []);
            
            
            if ~isempty(obj.world.obstacles)
                obj.hg_obstacles = fill(obj.world.obstacles_X, obj.world.obstacles_Y, ...
                    obj.obstacle_color, 'FaceAlpha', 0.25);
            end
            
            hold off;
        end
        
        function draw(obj, ids)
            if isempty(ids)
                obj.hg_landmarks_obs.set('XData', [], 'YData', []);
            else
                observed_landmarks = obj.world.landmarks(:, ids);
                obj.hg_landmarks_obs.set('XData', observed_landmarks(1, :), ...
                    'YData', observed_landmarks(2, :));
            end
        end
    end
end

