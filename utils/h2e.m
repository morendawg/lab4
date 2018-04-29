function e = h2e(h)
%H2E Homogeneous -> Euclidean
[d, ~] = size(h);
e = h(1:d-1, :) ./ h(d, :); % broadcast
end

