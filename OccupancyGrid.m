classdef OccupancyGrid < handle
    %OCCUPANCYGRID
    
    properties
        grid  % robotics.OccupancyGrid
    end
    
    properties (Dependent)
        size
        map
        resolution
        x_world_lim
        y_world_lim
    end
    
    methods
        function obj = OccupancyGrid(grid_size, resolution)
            %OCCUPANCYGRID, wraps a robotics.OccupancyGrid class
            % https://www.mathworks.com/help/robotics/ref/robotics.occupancygrid-class.html
            % GRID_SIZE: 1x2, [width, height] of the grid in meters
            % RESOLUTION: number of grid cells per meter 
            
            % NOTE: You are welcome to implement your own occupancy grid
            % algorithm, but we strongly recommend you start with this one.
            % If everything works and you have extra time, then feel free
            % to replace this with your own implementation.
            obj.grid = robotics.OccupancyGrid(grid_size(1), grid_size(2), resolution);
        end
        
        function add_scan(obj, pose, scan, max_range)
            % ADD_SCAN, update grid map with a scan measurement
            % INPUTS:
            %   POSE: 3x1, pose of the camera in world frame
            %   SCAN: 2xN, 2d scan measurement from the camear, 1st row is
            %   range, 2nd row is bearing. Note that if no obstacle
            %   detected, then range returns NaN.
            %   MAX_RANGE: maximum range of the scan measurement
            % HINT: use `insertRay` function from the robotics toolbox
            % https://www.mathworks.com/help/robotics/ref/robotics.occupancygrid.insertray.html
            
            % YOUR CODE STARTS HERE =======================================
            pose = pose';
            ranges = scan(1,:);
            angles = scan(2,:);
            show(obj.grid)
            insertRay(obj.grid,pose,ranges,angles,max_range);
            show(obj.grid)
            % YOUR CODE ENDS HERE =========================================
        end
        
        function map = inflated_map(obj, radius)
            % INFLATED_MAP, inflate occupancy grid map by radius
            % INPUTS:
            %   RADIUS: inflation radius
            % OUTPUTS:
            %   map: MxN, a matrix that represents the free, 
            %   unobserved, and occupied space of the grid map. This matrix
            %   should only have 3 distinct values. 
            %   M = grid_size(1), N = grid_size(2)
            % HINT: Use `inflate` function from the robotics toolbox
            % https://www.mathworks.com/help/robotics/ref/robotics.occupancymap3d.inflate.html
            
            % get a copy of the grid, don't want to inflate original grid
            copy_grid = obj.grid.copy();
            
            % YOUR CODE STARTS HERE =======================================
            inflate(copy_grid, radius);
            map = occupancyMatrix(copy_grid, 'ternary');
            % YOUR CODE ENDS HERE =========================================
        end
        
        % Getters
        function map = get.map(obj)
            map = obj.grid.occupancyMatrix;
        end
        
        function x_world_lim = get.x_world_lim(obj)
            x_world_lim = obj.grid.XWorldLimits;
        end
        
        function y_world_lim = get.y_world_lim(obj)
            y_world_lim = obj.grid.YWorldLimits;
        end
        
        function resolution = get.resolution(obj)
            resolution = obj.grid.Resolution;
        end
        
        function sz = get.size(obj)
            sz = obj.grid.GridSize;
        end
    end
end

