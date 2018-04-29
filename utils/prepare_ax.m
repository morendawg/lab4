function hg_ax = prepare_ax(xlim, ylim)
%PREPARE_AX
hg_ax = gca; grid on; axis equal;
hg_ax.XLim = xlim;
hg_ax.YLim = ylim;
hg_ax.XLimMode = 'manual';
hg_ax.YLimMode = 'manual';
hg_ax.set('Box', 'On');
end

