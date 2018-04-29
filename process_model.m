function Xn = process_model(X, U, dt)
%PROCESS_MODEL implements process model of a 2d differential driver robot
%   INPUTS:
%       X: pose at time k-1
%       u: vel_cmd at time k
%       dt: time step
%   OUTPUTS:
%       Xn: pose at time k

v = U(1, :);
w = U(2, :);

x = X(1, :) + dt * v .* cos(X(3, :));
y = X(2, :) + dt * v .* sin(X(3, :));
theta = X(3, :) + w * dt;

Xn = [x; y; wrapToPi(theta)];

end

