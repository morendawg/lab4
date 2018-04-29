classdef PlannerViz < handle
    %PLANNERVIZ 
    properties
        hg_ax
        planner
        hg_goal
        hg_path
    end
    
    methods
        function obj = PlannerViz(ax, planner)
            %PLANNERVIZ 
            obj.hg_ax = ax;
            obj.planner = planner;
            
            hold on
            obj.hg_goal = plot(obj.hg_ax, planner.goal_pos(1), planner.goal_pos(2), 'r+', 'LineWidth', 2, 'MarkerSize', 10);
            obj.hg_path = plot(obj.hg_ax, 0, 0);
            hold off
        end
        
        function draw(obj)
            set(obj.hg_path, 'XData', obj.planner.path(1, :), 'YData', obj.planner.path(2, :));
        end
    end
end

