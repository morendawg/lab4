function pose_msg = pose_msg__se2(se2)
%POSE_MSG__SE2 
pose_msg = rosmessage('geometry_msgs/Pose');
pose_msg.Position.X = se2(1);
pose_msg.Position.Y = se2(2);
pose_msg.Orientation.W = cos(se2(3));
pose_msg.Orientation.Z = sin(se2(3));
end

