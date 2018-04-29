%% Grid
ncol = 10;
nrow = 10;
space = 1;
x = 0:ncol;
y = 0:nrow;

[X, Y] = meshgrid(x, y);
landmarks = [X(:) Y(:)]' * space;
