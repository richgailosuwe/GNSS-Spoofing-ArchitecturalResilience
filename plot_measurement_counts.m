function plot_measurement_counts(scenario_name)
% PLOT_MEASUREMENT_COUNTS  Accepted/rejected EKF measurement counts.
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
    ekf = P.ekf;

    accepted = ekf.n_accepted(:);
    if isfield(ekf, 'n_rejected')
        rejected = ekf.n_rejected(:);
    else
        rejected = nan(size(accepted));
    end
    total = accepted + rejected;
    attack_epoch = get_attack_epoch(P, cfg);

    f = figure('Color','w','Position',[100 100 920 430],'Visible','off');
    hold on; grid on; box on;
    plot(total, '-', 'LineWidth', 1.1, 'Color', [0.45 0.45 0.45]);
    plot(accepted, '-', 'LineWidth', 1.4, 'Color', [0.10 0.45 0.70]);
    if any(~isnan(rejected))
        plot(rejected, '-', 'LineWidth', 1.4, 'Color', [0.82 0.15 0.12]);
        leg = {'available after mask/gate assembly','accepted','rejected'};
    else
        leg = {'available after mask/gate assembly','accepted'};
    end
    xline(attack_epoch, ':', 'attack onset', 'Color', [0.35 0.35 0.35], ...
        'LabelVerticalAlignment','bottom', 'LabelHorizontalAlignment','right');
    xlabel('Epoch');
    ylabel('Measurement count');
    title(sprintf('EKF measurement acceptance — %s', strrep(scenario_name,'_','\_')), ...
        'FontWeight','normal');
    legend(leg, 'Location','northwest');

    out = fullfile(fig_dir, sprintf('measurement_counts_%s.png', scenario_name));
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
