function P = weighted_state_cov(Xs, Xm, Wc)
% Computes weighted state covariance
res = Xs - Xm;
res(3, :) = wrapToPi(res(3, :));
% covariance
P = res * diag(Wc) * res';
end