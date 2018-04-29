classdef Planner < handle
  properties (Constant)
    goal_radius = 0.1;
end

properties
    goal_pos  % 2x1, position of the goal in world frame
    path      % 2xN, for visualization purpose only
end

methods
    function obj = Planner(grid_size, grid_resolution, goal_pose)
        %PLANNER
        % INPUTS:
        %   GRID_SIZE: 2x1, [width, height] of the grid
        %   GRID_RESOLUTION: number of cells per meter
        %   GOAL_POSE: 3x1, goal pose in world frame
        obj.goal_pos = goal_pose(1:2);            
    end
    
    function waypoints = plan(obj, gridmap, curr_pos)
        % PLAN, plan a path (of waypoints) from curr_pos to goal_pos in
        % gridmap
        % INPUTS:
        %   GRIDMAP: object, an occupancy gridmap
        %   CURR_POS: 2x1, current position of the robot in world frame
        % OUTPUTS:
        %   WAYPOINTS: 2xN, a collision free path that goes from
        %   curr_pos to goal_pos
        
        % YOUR CODE STARTS HERE =======================================
        % NOTE: the following is only a recommended implementation.
        % However, you are not restricted to it. You can implment any
        % algorithm you want, as long as the final result is a 
        % collision free path.
        % For example, instead of creating a graph from the map, one
        % could plan directly in the occupancy grid.
        % If you choose the way less traveled, then you don't need to
        % implement the following functions:
        % `inflated_map` and `map2adjmatrix`.
        
        % Inflate map and then convert to ternary representation
        map = gridmap.inflated_map(0.3); % DO NOT MODIFY
            
        % Create an adjacency matrix from map
        A = obj.map2adjmatrix(map); % DO NOT MODIFY
            
        % Create a digraph from the adjacency matrix
        G = digraph(A); % DO NOT MODIFY
        
        plot(G, 'Layout', 'force', 'EdgeLabel', G.Edges.Weight);
        
        % Call matlab's `shortestpath` on G, by default this uses
        % dijkstra's algorithm, if you implement A* if you want
        
        s = size(map);
        
        startNode = sub2ind(s,round(curr_pos(1)),round(curr_pos(2)));
        endNode = sub2ind(s,round(obj.goal_pos(1)), round(obj.goal_pos(2)));
        
        waypoints = shortestpath(G,startNode,endNode);
        waypoints = cell2mat({waypoints});
        
        w_s = zeros(2, length(waypoints));
        
        for i=1:length(waypoints)
            [I,J] = ind2sub(s,waypoints(i));
            w_s(:,i) = [I;J];
        end
        waypoints = w_s;
        % save for visualization
        obj.path = waypoints;
    end
    
    function A = map2adjmatrix(obj, map)
        % MAP2DIGRAPH convert map from occupancy grid to digraph
        % matrix 
        % INPUTS:
        %   MAP:
        % OUTPUTS:
        %   A: NxN, where N is the number of nodeso the graph
        %   represented by A. see:
        %   https://www.mathworks.com/help/matlab/ref/digraph.html
        
        % YOUR CODE STARTS HERE =======================================
        G = digraph;
        [n,m] = size(map);
        s = size(map);
        
        for i = 1:n
            for j = 1:m
                c = sub2ind(s,i,j);
                 for a = -1:1
                     for b = -1:1
                         if (i+a<1||j+b<1||i+a>n||j+b>m||(a==0 && b==0))
                             continue;
                         end                       
                         adj = sub2ind(s,i+a,j+b);
                         if(map(ind2sub(s,adj))==1)
                             G = addedge(G,c,adj,0);
                         elseif(map(ind2sub(s,adj))==0)
                             G = addedge(G,c,adj,1);
                         else
                             G = addedge(G,c,adj,2);
                         end
                         
                     end
                 end
             end
        end

        A = adjacency(G);
       
        % YOUR CODE ENDS HERE =========================================
        end
    
    function reached = goal_reached(obj, pose)
        % GOAL_REACHED, check if pose is within some distance of
        % goal_pos
        % INPUTS:
        %   POSE: 3x1, current pose of the robot in world frame
        % OUTPUTS:
        %   REACHED: bool, true if pose is close to goal_pose
        
        % YOUR CODE STARTS HERE =======================================
        
        pose_xy = [pose(1) pose(2)];
        if(norm(obj.goal_pos - pose_xy) < obj.goal_radius)
            reached = true;
            return
        end
        reached = false;
        % YOUR CODE ENDS HERE =========================================
    end
end
end

