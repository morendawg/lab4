function h = e2h(e)
%E2H Euclidean -> Homogeneous
[~, n] = size(e);
pad = ones(1, n);
h = [e; pad];
end

