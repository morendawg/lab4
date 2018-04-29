classdef Camera < handle
    %CAMERA A 2D camera, x along optical axis
    
    properties (Constant)
        max_range = 3       % max detection range
        min_range = 0.5     % min detection range
        angular_resolution = deg2rad(4);
    end
    
    properties
        f   % focal length in pixels
        w   % image width in pixels
        noise
        
        scan_lines
        n_scan_lines
        thetas
        ranges 
    end
    
    methods
        function obj = Camera(f, w, noise)
            %CAMERA constructor
            % INPUTS:
            %   f: focal length
            %   w: image width
            obj.f = f;
            obj.w = w;
            obj.noise = noise;
            
            theta = abs(atan(obj.w / obj.f / 2));
            obj.thetas = linspace(-theta, theta, 2 * theta / obj.angular_resolution);
            obj.n_scan_lines = length(obj.thetas);
            u = ones(1, obj.n_scan_lines) * obj.max_range;
            v = u .* tan(obj.thetas);
            obj.scan_lines = [zeros(1, obj.n_scan_lines);
                              zeros(1, obj.n_scan_lines);
                              u; v];
            obj.ranges = hypot(u, v);
            obj.ranges(:) = nan;
        end
        
        function mask = check_landmarks(obj, points)
            x = points(1, :);
            y = points(2, :);
            % check if points is within the range of the camera
            range_check = (x >= obj.min_range) & (x <= obj.max_range);
            % check if points fall within camera field of view
            fov_check = abs(y ./ x) < (obj.w / obj.f / 2);
            mask = range_check & fov_check;
        end
        
        function scan = measure_obstacles(obj, line_segs_C)
            % MEASURE_OBSTACLES obstacles measurement model for camera
            scan = [obj.ranges; obj.thetas];
            
            if isempty(line_segs_C)
                % return [] when there is no obstacles
                return;
            end
            
            % check intersection
            out = line_segment_intersect(obj.scan_lines', line_segs_C'); % input is Nx4
            
            % find min for each scan line
            
            % make 0 -> Inf
            int_matrix_X = out.intMatrixX;
            int_matrix_X(int_matrix_X == 0) = inf;
            % Find line segments that has the closest intersection with
            % scan lines
            [min_X, line_seg_ind] = min(int_matrix_X, [], 2);
            
            % find the scan line id that is within max range
            % these are the valid scans
            scan_line_ind = find(min_X <= obj.max_range & min_X > 0);
            line_seg_ind = line_seg_ind(scan_line_ind);
            
            % convert subscripts to linear indices so that we can get the Y
            % value of these intersections
            linear_ind = sub2ind(size(out.intMatrixY), scan_line_ind, line_seg_ind);
            min_Y = out.intMatrixY(linear_ind);
            min_X = min_X(scan_line_ind);
            
            % Update scan
            scan(1, scan_line_ind) = hypot(min_X, min_Y);
        end
        
        function [ids, range_bearings] = measure_landmarks(obj, landmarks_C)
            % MEASURE_LANDMARKS landmarks measurement model for camera
            % INPUTS:
            %   points_C: (2xN) coordinates of landmarks in camera frame
            % OUTPUTS:
            %   cam_meas: (3xN) id, range and bearing of observed landmarks
            %     * id is a unique identifier
            %     * range is the distance between the camera and the landmark
            %     * bearing is the angle between the x-axis of the camera
            %       which points forward and the landmark
            
            ids = [];
            range_bearings = [];
            if isempty(landmarks_C)
                % return [] when map is empty
                return;
            end
            
            % Check points that can be seen by camera
            valid = obj.check_landmarks(landmarks_C);
            [~, n] = size(landmarks_C);
            inds = 1:n;
            % ids of valid points
            valid_ids = inds(valid);
            
            if isempty(valid_ids)
                % return [] when no valid observation
                return;
            end
            
            % camera coords of valid points
            valid_landmarks = landmarks_C(:, valid);
            
            % converts points to bearing and range
            ranges = hypot(valid_landmarks(1, :), valid_landmarks(2, :));
            bearings = atan2(valid_landmarks(2, :), valid_landmarks(1, :));
            ids = valid_ids;
            range_bearings = [ranges; bearings];
            
            % inject noise to camera measurement
            m = length(ids);
            range_noise = randn(1, m) * obj.noise(1);
            bearing_noise = randn(1, m) * obj.noise(2);
            range_bearings = range_bearings + [range_noise; bearing_noise];
        end
    end
end

