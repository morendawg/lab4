classdef FilterViz < handle
    %FILTERVIZ 
    
    properties (Constant)
        cov_scale = 5;
        color_traj = 'r';
    end
    
    properties
        hg_ax
        hg_traj
        hg_cov
        filter
        hg_quivers
    end
    
    methods
        function obj = FilterViz(ax, filter)
            %FILTERVIZ
            obj.hg_ax = ax;
            obj.filter = filter;
            assert(filter.initialized, 'Filter is not initialized');            
            
            hold on
            obj.hg_traj = plot(filter.state(1), filter.state(2), 'Color', obj.color_traj);
            if strcmp(obj.filter.type, 'pf')
                % Draw all particles
                quivers = obj.filter.quivers;
                obj.hg_quivers = quiver(quivers(1,:), quivers(2, :), quivers(3, :), quivers(4, :), 0);
            end
            % Draw covariance
            ellipse = cov_ellipse(filter.state(1:2), filter.state_cov(1:2, 1:2), obj.cov_scale);
            obj.hg_cov = plot(ellipse(1, :), ellipse(2, :));
            hold off
        end
        
        function draw(obj)
            obj.hg_traj.set('XData', obj.filter.state_hist(1, :), ...
                'YData', obj.filter.state_hist(2, :));
            
            if strcmp(obj.filter.type, 'pf')
                quivers = obj.filter.quivers;
                obj.hg_quivers.set(...
                    'XData', quivers(1, :), 'YData', quivers(2, :), ...
                    'UData', quivers(3, :), 'VData', quivers(4, :));
            end
            ellipse = cov_ellipse(obj.filter.state(1:2), obj.filter.state_cov(1:2, 1:2), obj.cov_scale);
            ellipse = real(ellipse);
            obj.hg_cov.set('XData', ellipse(1, :), 'YData', ellipse(2, :));
        end
    end
end

