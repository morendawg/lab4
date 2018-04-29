%%
x = 1:2:9;
y = 1:2:9;
d = 1;

X = [x - d/2; x + d/2; x + d/2; x - d/2];
Y = [y - d/2; y - d/2; y + d/2; y + d/2];

obstacles.X = X;
obstacles.Y = Y;

save('obstacles_simple.mat', 'obstacles');

%%

x = [3:10, 0:7, 3:10];
y = [3*ones(1,8), 6*ones(1, 8), 9*ones(1,8)];

X = [x - d/2; x + d/2; x + d/2; x - d/2];
Y = [y - d/2; y - d/2; y + d/2; y + d/2];

obstacles.X = X;
obstacles.Y = Y;

save('obstacles_complex.mat', 'obstacles');
