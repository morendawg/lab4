function X = weighted_state_mean(Xs, W)
%STATE_MEAN Calculate weighted mean of multiple states
W = W(:);
mu_xy = Xs(1:2, :) * W;
 % mean of heading is the heading of the weighted unit vectors
sum_sin = sin(Xs(3, :)) * W;
sum_cos = cos(Xs(3, :)) * W;
mu_theta = atan2(sum_sin, sum_cos);
X = [mu_xy; mu_theta];
end

