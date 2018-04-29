function Z = measurement_model(X, L, pose_BC)
%MEASUREMENT_MODEL
[~, m] = size(L);
[~, n] = size(X);

% Get camera in world frame
X_WC = se2_oplus(X, pose_BC);
Z = zeros(2*m, n);

if n < m
    % loop over X
    for i = 1:n
        diff = L - X_WC(1:2, i);
        range = hypot(diff(1, :), diff(2, :));
        bearing = wrapToPi(atan2(diff(2, :), diff(1, :)) - X_WC(3, i));
        rb = [range; bearing];
        Z(:, i) = reshape(rb, [], 1); % reshape to 2mx1
    end
else
    % loop over L, since X can be big for particle filter
    for i = 1:m
        diff = L(:, i) - X_WC(1:2, :);
        range = hypot(diff(1, :), diff(2, :));
        bearing = wrapToPi(atan2(diff(2, :), diff(1, :)) - X_WC(3, :));
        rb = [range; bearing];
        Z(2*i-1:2*i, :) = rb;
    end
end

end

