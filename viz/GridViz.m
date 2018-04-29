classdef GridViz < handle
    
    properties
        hg_ax
        hg_grid
        grid
    end
    
    methods
        function obj = GridViz(ax, grid)
            %GRIDVIZ
            obj.hg_ax = ax;
            obj.grid = grid;
            
            obj.hg_ax.Visible = 'off';
            % Use gray colormap for correct visualization
            cmap = colormap(obj.hg_ax, 'gray');
            % Flip color map to make unoccupied (free) cells white
            cmap = flip(cmap);
            
            obj.hg_grid = imshow(obj.grid.map, 'Parent', obj.hg_ax, 'InitialMagnification', 'fit');
            colormap(obj.hg_ax, cmap);
            
            xlimits = obj.grid.x_world_lim;
            ylimits = obj.grid.y_world_lim;
            correction = 1 / (2*obj.grid.resolution);
            
            % Set XData and YData
            if (abs(xlimits(1) - xlimits(2) + 2 * correction) < eps)
                % Special case when there is only one cell
                obj.hg_grid.XData = [xlimits(1), xlimits(2)];
            else
                obj.hg_grid.XData = [xlimits(1) + correction, xlimits(2) - correction];
            end
            
            if (abs(ylimits(1) - ylimits(2) + 2 * correction) < eps)
                obj.hg_grid.YData = [ylimits(2), ylimits(1)];
            else
                obj.hg_grid.YData = [ylimits(2) - correction, ylimits(1) + correction];
            end
            
            % Set the axes
            set(obj.hg_ax, 'YDir','normal');
            
            % Set XLim and YLim
            obj.hg_ax.XLim = xlimits;
            obj.hg_ax.YLim = ylimits;
            
            % Make axes visible
            obj.hg_ax.Visible = 'on';
            
        end
        
        function draw(obj)
            obj.hg_grid.set('CData', obj.grid.map);
        end
    end
end

