function points_new = se2_transform(se2, points)
%SE2_TRANSFORM transform points using se2, vectorized
c = cos(se2(3, :));
s = sin(se2(3, :));
px = points(1, :);
py = points(2, :);
points_new = se2(1:2, :) + [c .* px - s .* py;
                            s .* px + c .* py];
end

