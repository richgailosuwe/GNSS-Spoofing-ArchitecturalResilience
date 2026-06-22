%% plot_chi2_timeline.m
% Plot the GPS-only chi-squared consistency statistic for authentic data and
% the final saved Scenario 1 observations (GPS PRNs 14, 22, and 31 spoofed).
%
% The calculation uses corrected_pseudorange, the final transmit-time and
% group-delay measurement model. The threshold varies with the number of
% valid GPS measurements because dof = n_sats - 4.
%
% OUTPUT:
%   results/figures/chi2_timeline.png
%
% This is a diagnostic figure, not proof that chi-squared always detects or
% always absorbs a coordinated attack. The printed exceedance counts report
% what occurs in the saved Scenario 1 evidence.

config;
cfg.verbose = 0;

obs_path = fullfile(cfg.paths.obs, 'authentic.obs');
nav_path = fullfile(cfg.paths.nav, 'authentic.nav');
spoof_path = fullfile(cfg.root, 'results', 'simulated_scenarios', ...
    'scenario_1_gps', 'spoofed_obs.mat');

assert(isfile(obs_path), 'Authentic observation file not found: %s', obs_path);
assert(isfile(nav_path), 'Navigation file not found: %s', nav_path);
assert(isfile(spoof_path), 'Saved Scenario 1 observations not found: %s', spoof_path);

obs_auth = rinex_read_obs(obs_path, cfg);
nav = rinex_read_nav(nav_path, cfg);
loaded = load(spoof_path, 'obs_spoofed');
obs_spoof = loaded.obs_spoofed;

epochs = unique(obs_auth.GPS.time);
n_epochs = numel(epochs);
start_epoch = cfg.scenarios{1}.start_epoch;
spoofed_prns = cfg.scenarios{1}.spoofed_PRNs.GPS;

chi2_auth = nan(n_epochs, 1);
chi2_spoof = nan(n_epochs, 1);
threshold = nan(n_epochs, 1);

fprintf('\nComputing final-model GPS chi-squared timeline (%d epochs)...\n', n_epochs);
for ei = 1:n_epochs
    t_e = epochs(ei);
    [pr_auth, sp_auth] = gps_epoch_set(obs_auth, nav, cfg, t_e);
    [pr_spoof, sp_spoof] = gps_epoch_set(obs_spoof, nav, cfg, t_e);

    if numel(pr_auth) >= cfg.identify.min_sats
        [chi2_auth(ei), threshold(ei)] = chi2_from_set(pr_auth, sp_auth, cfg);
    end
    if numel(pr_spoof) >= cfg.identify.min_sats
        chi2_spoof(ei) = chi2_from_set(pr_spoof, sp_spoof, cfg);
    end

    if mod(ei, 500) == 0
        fprintf('  epoch %d / %d\n', ei, n_epochs);
    end
end

attack_mask = (1:n_epochs)' >= start_epoch;
valid_attack = attack_mask & isfinite(threshold);
auth_exceed = valid_attack & (chi2_auth > threshold);
spoof_exceed = valid_attack & (chi2_spoof > threshold);

fprintf('\n=== Scenario 1 chi-squared summary ===\n');
fprintf('Spoofed GPS PRNs: %s\n', strjoin("G" + string(spoofed_prns), ', '));
fprintf('Attack-period valid epochs: %d\n', sum(valid_attack));
fprintf('Authentic threshold exceedances: %d (%.2f%%)\n', ...
    sum(auth_exceed), 100 * sum(auth_exceed) / sum(valid_attack));
fprintf('Scenario 1 threshold exceedances: %d (%.2f%%)\n', ...
    sum(spoof_exceed), 100 * sum(spoof_exceed) / sum(valid_attack));

e_quote = min(500, n_epochs);
fprintf('Epoch %d: authentic %.2f, Scenario 1 %.2f, threshold %.2f\n\n', ...
    e_quote, chi2_auth(e_quote), chi2_spoof(e_quote), threshold(e_quote));

%% Plot the complete, unclipped statistic on a logarithmic y-axis.
fig = figure('Color', 'w', 'Position', [100 100 1000 520]);
ax = axes(fig);
set(ax, 'YScale', 'log');
hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
plot_floor = 1e-2;

ph_a = plot(ax, 1:n_epochs, max(chi2_auth, plot_floor), '-', ...
    'LineWidth', 1.1, 'Color', [0.15 0.45 0.20]);
ph_s = plot(ax, 1:n_epochs, max(chi2_spoof, plot_floor), '-', ...
    'LineWidth', 1.1, 'Color', [0.78 0.12 0.10]);
ph_t = plot(ax, 1:n_epochs, threshold, '--', ...
    'LineWidth', 1.8, 'Color', [0.10 0.15 0.45]);

xline(start_epoch, ':', 'Attack onset', 'Color', [0.35 0.35 0.35], ...
    'LabelVerticalAlignment', 'bottom', 'FontSize', 9);

xlabel('Epoch');
ylabel('Chi-squared statistic \lambda (log scale)');
title('GPS-only chi-squared consistency: authentic vs Scenario 1');
legend([ph_a ph_s ph_t], ...
    {'Authentic', 'Scenario 1: G14, G22, G31', 'DOF-dependent threshold'}, ...
    'Location', 'northwest');
xlim([1 n_epochs]);

finite_values = [chi2_auth(isfinite(chi2_auth)); ...
                 chi2_spoof(isfinite(chi2_spoof)); ...
                 threshold(isfinite(threshold))];
if ~isempty(finite_values)
    positive_values = finite_values(finite_values > 0);
    ylim(ax, [plot_floor, 1.2 * max(positive_values)]);
end

subtitle(sprintf(['Final transmit-time model; configured attack onset epoch %d; ' ...
    'threshold exceedances during attack: %d/%d'], ...
    start_epoch, sum(spoof_exceed), sum(valid_attack)));

out_dir = fullfile(cfg.root, 'results', 'figures');
if ~isfolder(out_dir), mkdir(out_dir); end
out_path = fullfile(out_dir, 'chi2_timeline.png');
exportgraphics(fig, out_path, 'Resolution', 300);
fprintf('Figure saved: %s\n', out_path);

%% Local helpers
function [pr_corr, sat_pos] = gps_epoch_set(obs, nav, cfg, t_e)
    mask = (obs.GPS.time == t_e);
    prns = obs.GPS.prn(mask);
    raw_pr = obs.GPS.pseudorange_L1(mask);

    pr_corr = zeros(0, 1);
    sat_pos = zeros(0, 3);
    for k = 1:numel(prns)
        if ~isfinite(raw_pr(k)) || raw_pr(k) <= 0
            continue;
        end
        try
            [pr_k, sp_k] = corrected_pseudorange(raw_pr(k), prns(k), 'GPS', ...
                t_e, cfg.ref_pos(:), nav, cfg);
        catch
            continue;
        end
        if ~isfinite(pr_k) || isempty(sp_k) || any(~isfinite(sp_k))
            continue;
        end
        pr_corr(end+1, 1) = pr_k; %#ok<AGROW>
        sat_pos(end+1, :) = sp_k(:)'; %#ok<AGROW>
    end
end

function [lambda, threshold] = chi2_from_set(pr_corr, sat_pos, cfg)
    weights = ones(numel(pr_corr), 1) / cfg.ekf.meas_noise_GPS;
    [~, ~, residuals] = wls_solver(pr_corr, sat_pos, weights, cfg.ref_pos);
    result = chi_squared_test(residuals, weights, 4, cfg);
    lambda = result.test_stat;
    threshold = result.threshold;
end
