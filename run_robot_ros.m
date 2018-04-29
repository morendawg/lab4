%%
clear; clc; close all;

%% Init
robot_ip = '192.168.129.182'; 
host_ip = '192.168.129.208';

start_pose = [0; 2.7; -pi/2];
robot_noise = [0.05; deg2rad(2)];
camera_noise = [0.1; deg2rad(5)];
pose_noise = ones(3, 1) * 0.2;

%% 
robot = Robot(start_pose, [0; 0]);  % set 0 noise only for visualization
camera = Camera(630, 635, [0; 0]); 
robot.add_camera(camera, [0.125; -0.025; 0]);
world = World('landmarks.mat');

interface = RosInterface(robot_ip, host_ip, robot, world);

filter = ParticleFilter(256);
filter.initialize(start_pose, pose_noise, robot_noise, camera_noise);
control = RobotControl(filter);

%% Setup visualization
hg_ax = prepare_ax([-5, 5], [-5, 5]);
h_title = title('ROS');
robot_viz = RobotViz(hg_ax, robot);
world_viz = WorldViz(hg_ax, world);
filter_viz = FilterViz(hg_ax, filter);

%% 
fprintf('Running in ROS\n');
while ~interface.done
%     meas = interface.measurement();
    
    t = tic;
%     vel_cmd = control.process(meas);
    vel_cmd = [0.1; 0];
    t = toc(t) * 1000;

    interface.update(vel_cmd);
    
    robot_viz.draw(control.filter.state);
%     world_viz.draw(meas.landmark_ids);
    filter_viz.draw();
    h_title.String = sprintf('elapsed: %0.2fms', t);
    drawnow;
end
