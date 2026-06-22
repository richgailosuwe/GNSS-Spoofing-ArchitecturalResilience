function plot_hardware_wls_scatter()
% PLOT_HARDWARE_WLS_SCATTER  Real-hardware WLS ENU scatter about session mean.
%
% Loads saved hardware validation .mat only and writes one PNG to results/figures.
% This is a precision/stability plot, not an absolute-accuracy plot.

    config;
    fig_dir = fullfile(cfg.root, 'results', 'figures');
    if ~isfolder(fig_dir), mkdir(fig_dir); end

    fp = fullfile(cfg.root, 'results', 'hardware', 'june18footballpitchdomnesti2_validation.mat');
    S = load(fp);
    R = S.results;
    wls = R.wls_pos;
    valid = all(isfinite(wls), 2);
    wls = wls(valid, :);

    ref = mean(wls, 1, 'omitnan')';
    enu = nan(size(wls));
    for i = 1:size(wls, 1)
        [e, n, u] = coord_convert('ecef2enu', wls(i,:)', ref);
        enu(i,:) = [e n u];
    end

    horiz = vecnorm(enu(:,1:2), 2, 2);
    med_scatter = median(horiz, 'omitnan');
    p95_scatter = prctile(horiz(~isnan(horiz)), 95);

    lim = max(10, ceil(max(abs(enu(:,1:2)), [], 'all', 'omitnan') / 5) * 5);

    f = figure('Color','w','Position',[100 100 620 580],'Visible','off');
    hold on; grid on; box on; axis equal;
    plot(enu(:,1), enu(:,2), '.', 'Color', [0.10 0.45 0.70], 'MarkerSize', 7);
    plot(0, 0, 'kx', 'LineWidth', 1.5, 'MarkerSize', 8);
    xlim([-lim lim]); ylim([-lim lim]);
    xlabel('East about WLS session mean [m]');
    ylabel('North about WLS session mean [m]');
    title({'Real hardware WLS horizontal scatter — Domnesti pitch', ...
        sprintf('Precision about session mean only: median %.2f m, p95 %.2f m; no surveyed truth', ...
        med_scatter, p95_scatter)}, 'FontWeight','normal', 'FontSize', 11);

    out = fullfile(fig_dir, 'hardware_wls_scatter_domnesti2.png');
    exportgraphics(f, out, 'Resolution', 300);
    close(f);
    fprintf('Saved: %s\n', out);
end
