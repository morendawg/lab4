% Author: cis390

classdef RosInterface < handle
    %ROSINTERFACE Ros interface to the robot
    %    This class is intended to replace SimInterface seamleassly
    
    properties (Constant)
        hz = 10     % the control rate is at 10hz
        dt = 1/10
    end
    
    properties
        done
        vel_cmd
        sub_odom        % subscribe to odom
        sub_pose        % subscribe to pose
        sub_cloud       % subscribe to pointcloud -> (id, range, bearing)
        pub_start_pose  % publish init pose
        pub_twist       % publish twist
        rate            % ros rate object
        twist_msg       % twist msg
        init_odom       % first odom [x, y, theta]
        robot           % Robot
        world           % World
    end
    
    methods
        function obj = RosInterface(ros_ip, host_ip, robot, world)
            %ROSINTERFACE Construct an instance of this class
            %   The constructor takes in an ip and calls rosinit to connect
            %   to ros master and creates relevant publisher/subscriber
            
            % shutdown and restart ros
            rosshutdown;
            rosinit(ros_ip, 'NodeHost', host_ip);
            
            obj.done = 0;
            obj.robot = robot;
            obj.world = world;
            
            % subscriber
            obj.sub_odom = rossubscriber('/odom', 'nav_msgs/Odometry');
            obj.sub_cloud = rossubscriber('/RangeAndBearing', 'sensor_msgs/PointCloud');
            
            % publisher
            obj.pub_twist = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
            
            % ros rate
            obj.rate = rosrate(obj.hz);
            reset(obj.rate);
            
            % vel_cmd
            obj.vel_cmd = [0; 0];
            obj.twist_msg = rosmessage('geometry_msgs/Twist');
            
            obj.init_odom = [];
        end
        
        function meas = measurement(obj)
            % get measurement from robot
            % NOTE: we use time of odom as time for any measurments (image)
            % although different measurements are sampeld at different time
            % our rate is so low that this should not matter to much.
            
            % get measurement from robot
            odom_msg = receive(obj.sub_odom, obj.dt * 10);
            
            % decode point cloud
            cloud_msg = receive(obj.sub_cloud, obj.dt * 10);
            range_bearings, ids = decode_cloud(cloud_msg);
            
            % ids of observed landmarks, Mx1
            meas.landmark_ids = ids(:);
            % range and bearing measurements, 2xM
            meas.range_bearings = range_bearings;
            
            % convert time to double
            meas.time = rostime2double(odom_msg.Header.Stamp);
            meas.pose_BC = obj.robot.pose_BC;
            meas.landmarks = obj.world.landmarks;
            meas.vel_cmd = obj.vel_cmd;
            
            odom = se2__pose_msg(odom_msg.Pose.Pose);
            
            % set init_pose
            if isempty(obj.init_odom)
                obj.init_odom = odom;
            end
            
            % for every new pose, inv compose with init_pose
            meas.odom = se2_ominus(odom, obj.init_odom);
        end
        
        function update_twist_msg(obj, vel_cmd)
            % check_vel_cmd(vel_cmd)
            obj.twist_msg.Linear.X = vel_cmd(1);
            obj.twist_msg.Angular.Z = vel_cmd(2);
        end
        
        function update(obj, vel_cmd)
            % update Twist msg from vel_cmd
            obj.vel_cmd = vel_cmd;
            obj.update_twist_msg(vel_cmd);
            
            % publish and wait
            obj.pub_twist.send(obj.twist_msg);
            waitfor(obj.rate);
        end
    end
end

function [range_bearings, landmark_ids] = decode_cloud(cloud_msg)

% range_bearings: range and bearing matrix (2 x N) where N is the number of tags
%         visible
% landmarks_ids: N x 1 matrix 

num_points = length(cloud_msg.points);
range_bearings = zeros(2, num_points);
landmark_ids = zeros(num_points, 1);

for i=1:num_points
    landmark_ids(i) = cloud_msg.points.x;
    range_bearings(1, i) = cloud_msg.points.y; 
    range_bearings(2, i) = cloud_msg.points.z;
end
end