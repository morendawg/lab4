function v = se2_oplus(v1, v2)
%SE2_OPLUS v = v1 o+ v2, vectorized
%    v = v1.compose(v2)
c1 = cos(v1(3, :));
s1 = sin(v1(3, :));
t2x = v2(1, :);
t2y = v2(2, :);

t = v1(1:2, :) + ...
    [c1 .* t2x - s1 .* t2y;
    s1 .* t2x + c1 .* t2y];
theta = wrapToPi(v1(3, :) + v2(3, :));

v = [t; theta];
end

