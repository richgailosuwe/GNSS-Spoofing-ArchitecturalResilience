function plot_ablation_zoom(scenario_name, zoom_epochs)
% PLOT_ABLATION_ZOOM  Zoomed A/B/C mitigation comparison from saved evidence.
%
% Loads saved ablation .mat only and writes one PNG to results/figures.

    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'scenario_4_gps_glonass';
    end
    if nargin < 2 || isempty(zoom_epochs)
        zoom_epochs = [100 300];
    end

    config;
    fig_dir = fullfile(cfg.root, 'results', 'figures');
    if ~isfolder(fig_dir), mkdir(fig_dir); end

    fp = fullfile(cfg.root, 'results', 'ablation', sprintf('%s_ablation.mat', scenario_name));
    S = load(fp);
    R = S.results;

    n = numel(R.peA);
    x = (1:n)';
    idx = x >= zoom_epochs(1) & x <= zoom_epochs(2);
    ymax = max([R.peA(idx); R.peB(idx); R.peC(idx)], [], 'omitnan');
    ymax = max(20, ceil(1.10 * ymax / 10) * 10);

    f = figure('Color','w','Position',[100 100 920 430],'Visible','off');
    hold on; grid on; box on;
    pC = plot(x, R.peC, '-', 'LineWidth', 1.2, 'Color', [0.75 0.10 0.10]);
    pB = plot(x, R.peB, '-', 'LineWidth', 1.7, 'Color', [0.20 0.60 0.20]);
    pA = plot(x, R.peA, '-', 'LineWidth', 1.3, 'Color', [0.10 0.45 0.70]);
    if isfield(R, 'start_epoch')
        xline(R.start_epoch, ':', 'attack onset', 'Color', [0.35 0.35 0.35], ...
            'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
    end
    xlim(zoom_epochs);
    ylim([0 ymax]);
    xlabel('Epoch');
    ylabel('3D reference-relative position error [m]');
    title(sprintf('Attack-onset zoom: mitigation ablation — %s', strrep(scenario_name,'_','\_')), ...
        'FontWeight','normal');
    legend([pA pB pC], {'A: full pipeline','B: gate-only','C: no mitigation'}, ...
        'Location','northwest');

    out = fullfile(fig_dir, sprintf('ablation_zoom_%s.png', scenario_name));
    exportgraphics(f, out, 'Resolution', 300);
    close(f);
    fprintf('Saved: %s\n', out);
end
