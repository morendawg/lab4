%% Cleanup
clear; clc; close all; init;

%% Init
max_time = 80;  % max simulation time

start_pose = [2; 1; pi/3];
goal_pose = [8; 8; 0];     % final orientation doesn't matter

robot_noise = [0.05; deg2rad(2)];
robot = Robot(start_pose, robot_noise);  % robot kinematic model
camera_noise = [0.1; deg2rad(5)];
camera = Camera(1, 1.25, camera_noise);    % camera
robot.add_camera(camera, [0.3; 0; 0]);  % attach camera to robot
world = World('maps/landmarks_grid.mat', 'maps/obstacles_simple.mat');
interface = SimInterface(max_time, robot, world);

%% 
% PARTICLE_FILTER, use your lab3 code
pose_noise = ones(3, 1) * 0.2;
filter = ParticleFilter(200);
filter.initialize(start_pose, pose_noise, robot_noise, camera_noise);

% OCCUPANCY_GRID_MAP, your code in this class
grid_size = [10; 10];   % width, height in meters, assume orgin is [0, 0]
grid_resolution = 2.5;  % number of cells per meter in both direction
gridmap = OccupancyGrid(grid_size, grid_resolution);

% PLANNER, your code in this class
planner = Planner(gridmap.size, gridmap.resolution, goal_pose);

% TRACKER des_vel, max_ang_vel, look_ahead_dist
tracker = Tracker(0.2, 0.5, 0.5);

% Everything goes into control
control = RobotControl(filter, gridmap, planner, tracker);

%% Setup visualization
hg_ax = prepare_ax([0, grid_size(1)], [0, grid_size(2)]);
grid_viz = GridViz(hg_ax, gridmap);
robot_viz = RobotViz(hg_ax, robot);
world_viz = WorldViz(hg_ax, world);
filter_viz = FilterViz(hg_ax, filter);
planner_viz = PlannerViz(hg_ax, planner);
h_title = title(interface.title_str());

%% Main loop
fprintf('Running in SIMULATION\n');
while ~interface.done
    meas = interface.measurement();
    
    t = tic;
    vel_cmd = control.process(meas);
    t = toc(t) * 1000;
    
    interface.update(vel_cmd);
    
    % visualize observed landmarks
    world_viz.draw(meas.landmark_ids);
    robot_viz.draw_pose(meas.pose);
    robot_viz.draw_scan(meas.scan);
    planner_viz.draw();
    filter_viz.draw();
    grid_viz.draw();
    drawnow;
    
    h_title.String = sprintf('%s, elapsed: %0.2fms', interface.title_str(), t);
end

