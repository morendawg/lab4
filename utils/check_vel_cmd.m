function vel_cmd = check_vel_cmd(vel_cmd)
%CHECK_VEL_CMD 
max_lin_vel = 0.5;
max_ang_vel = 4;
if abs(vel_cmd(1)) > max_lin_vel
    warning('linear velocity too big [%.4f]', vel_cmd(1));
    vel_cmd(1) = sign(vel_cmd(1)) * max_lin_vel;
end

if abs(vel_cmd(2)) > max_ang_vel
    warning('angular velocity too big [%.4f]', vel_cmd(2));
    vel_cmd(2) = sign(vel_cmd(2)) * max_ang_vel;
end
end

