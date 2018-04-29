function plot_filter(state_hist, state_var_hist, sigma)
%PLOT_FILTER plot estimated pose with 3 sigma std

[~, l] = size(state_hist);
times = 1:l;

state_std_hist = sqrt(state_var_hist);

figure();
for i = 1:3
    subplot(3, 1, i);
    axis tight;
    grid on;
    hold on;
    plot(times, state_hist(i, :), 'r');
    plot(times, state_hist(i, :) + sigma * state_std_hist(i, :), 'b');
    plot(times, state_hist(i, :) - sigma * state_std_hist(i, :), 'b');
    hold off;
end

end

