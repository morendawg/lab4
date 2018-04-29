function X = matrixfy(x)
%MATRIXFY Create matrix with x along the diagonal if x is a vector
if iscolumn(x) || isrow(x)
    X = diag(x);
else
    X = x;
end

