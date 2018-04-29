classdef World < handle
    %WORLD Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        landmarks
        obstacles
        line_segs
    end
    
    properties (Dependent)
        landmarks_X   % all landmarks x coords
        landmarks_Y   % all landmarks y coords
        n_landmarks
        
        obstacles_X
        obstacles_Y
        n_obstacles
    end
    
    methods
        function obj = World(landmarks_file, obstacles_file)
            % Load from mat file
            load(landmarks_file);
            obj.landmarks = landmarks; % 2xN
            
            if nargin == 2
                load(obstacles_file)
                obj.obstacles = obstacles;
                obj.line_segs = make_line_segments(obj.obstacles.X, obj.obstacles.Y);
            end
        end
        
        % Getter/Setter
        function X = get.landmarks_X(obj)
            X = obj.landmarks(1, :);
        end
        
        function Y = get.landmarks_Y(obj)
            Y = obj.landmarks(2, :);
        end
        
        function n = get.n_landmarks(obj)
            n = length(obj.landmarks);
        end
        
        function X = get.obstacles_X(obj)
            if isempty(obj.obstacles)
                X = [];
            else
                X = obj.obstacles.X;
            end
        end
        
        function Y = get.obstacles_Y(obj)
            if isempty(obj.obstacles)
                Y = [];
            else
                Y = obj.obstacles.Y;
            end
        end
        
        function n = get.n_obstacles(obj)
            [~, n] = size(obj.obstacles.X);
        end
        
    end
end

function line_segs = make_line_segments(X, Y)
X = [X; X(1, :)];
Y = [Y; Y(1, :)];

X1 = X(1:end-1, :);
Y1 = Y(1:end-1, :);
X2 = X(2:end, :);
Y2 = Y(2:end, :);

line_segs = [X1(:), Y1(:), X2(:), Y2(:)]';
end

