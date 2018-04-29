function se2 = se2__pose_msg(pose_msg)
%SE2__POSE_MSG se2 from ros::Pose
se2 = zeros(3, 1);
se2(1) = pose_msg.Position.X;
se2(2) = pose_msg.Position.Y;
se2(3) = atan2(pose_msg.Orientation.Z, pose_msg.Orientation.W);
end

