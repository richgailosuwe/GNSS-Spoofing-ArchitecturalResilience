function plot_classification_counts(scenario_name)
% PLOT_CLASSIFICATION_COUNTS  Trusted/suspect/spoofed counts from saved evidence.
%
% Loads saved pipeline .mat only and writes one PNG to results/figures.

    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'scenario_1_gps';
    end

    config;
    fig_dir = fullfile(cfg.root, 'results', 'figures');
    if ~isfolder(fig_dir), mkdir(fig_dir); end

    fp = fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', scenario_name));
    S = load(fp);
    P = S.pipeline_result;
    C = P.classification;
    n_epochs = numel(C);

    trusted = nan(n_epochs, 1);
    suspect = nan(n_epochs, 1);
    spoofed = nan(n_epochs, 1);
    for e = 1:n_epochs
        if isempty(C{e}), continue; end
        cr = C{e};
        if isfield(cr, 'n_trusted'), trusted(e) = cr.n_trusted; end
        if isfield(cr, 'n_suspect'), suspect(e) = cr.n_suspect; end
        if isfield(cr, 'n_spoofed'), spoofed(e) = cr.n_spoofed; end
    end

    attack_epoch = get_attack_epoch(P, cfg);

    f = figure('Color','w','Position',[100 100 920 430],'Visible','off');
    hold on; grid on; box on;
    plot(trusted, '-', 'LineWidth', 1.3, 'Color', [0.18 0.62 0.25]);
    plot(suspect, '-', 'LineWidth', 1.3, 'Color', [0.95 0.65 0.15]);
    plot(spoofed, '-', 'LineWidth', 1.5, 'Color', [0.82 0.15 0.12]);
    xline(attack_epoch, ':', 'attack onset', 'Color', [0.35 0.35 0.35], ...
        'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
    xlabel('Epoch');
    ylabel('Number of satellites');
    title(sprintf('Stage 2 classification counts — %s', strrep(scenario_name,'_','\_')), ...
        'FontWeight','normal');
    legend({'trusted','suspect','spoofed'}, 'Location','northwest');

    out = fullfile(fig_dir, sprintf('classification_counts_%s.png', scenario_name));
    exportgraphics(f, out, 'Resolution', 300);
    close(f);
    fprintf('Saved: %s\n', out);
end

function attack_epoch = get_attack_epoch(P, cfg)
    attack_epoch = 120;
    if isfield(P, 'scenario_idx') && P.scenario_idx <= numel(cfg.scenarios)
        if isfield(cfg.scenarios{P.scenario_idx}, 'start_epoch')
            attack_epoch = cfg.scenarios{P.scenario_idx}.start_epoch;
        end
    end
end
