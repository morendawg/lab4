function se2_inv = se2_inverse(se2)
%SE2_INVERSE inverse of se2

theta = se2(3, :);
tx = se2(1, :);
ty = se2(2, :);
c = cos(theta);
s = sin(theta);
se2_inv = [-c .* tx - s .* ty;
           s .* tx - c .* ty;
           -theta];
end

