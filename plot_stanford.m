%% plot_stanford.m
% Driver to produce the Stanford integrity diagram for Chapter 6.
%
% Plots the horizontal protection level (HPL) against the horizontal
% position error for every epoch of an ekf_runner run, on the standard
% Stanford integrity diagram axes. Each epoch is one point.
%
% The diagram partitions the (error, HPL) plane:
%   - Above the HPL = error diagonal : error is BOUNDED (HPL >= error),
%     the receiver correctly contains its own error. This is the safe
%     region.
%   - Below the diagonal             : error EXCEEDS the protection level,
%     i.e. misleading information (the protection level failed to bound
%     the true error). No epoch should fall here.
%   - The limit lines mark the RNP-0.1 lateral accuracy scale (185.2 m)
%     and twice that scale (370.4 m). Both are descriptive reference lines,
%     NOT certified alert limits.
%
% A correct, available solution sits ABOVE the diagonal and LEFT of /
% BELOW the limit lines: error is small and bounded, HPL is within limit.
%
% HOW TO RUN (from project root):
%   config
%   % run (or load) an authentic EKF result:
%   obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
%   nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);
%   ekf_out = ekf_runner(obs, nav, {}, cfg);     % or load('ekf_authentic_full.mat')
%   run('plot_stanford.m')
%
% To overlay spoofed scenarios, populate the optional struct array
% `scenario_runs` BEFORE running this script, e.g.:
%   scenario_runs(1).label = 'Scenario 1 (GPS)';   scenario_runs(1).ekf = ekf_s1;
%   scenario_runs(2).label = 'Scenario 2 (Galileo)'; scenario_runs(2).ekf = ekf_s2;
% If `scenario_runs` does not exist, only the authentic run is plotted.
%
% INTEGRITY LIMITS (sourced):
%   RNP-0.1 lateral accuracy : 0.1 NM = 185.2 m  (95% accuracy reference)
%   2 x reference scale      : 0.2 NM = 370.4 m (descriptive, not certified)
%   Reported as experimental RAIM-style integrity context, NOT certified
%   RNP AR / SBAS / GBAS compliance.
%
% OUTPUT:
%   - Figure saved to results/figures/stanford_diagram.png (300 dpi)
%   - Bounding statistics printed to the command window for the thesis.
%

%% --- Integrity limits ---------------------------------------------------
RNP_ACCURACY = 185.2;    % 0.1 NM lateral accuracy scale [m]
REF_2X       = 370.4;    % 2 x RNP-0.1 reference scale [m] (descriptive, not certified)

%% --- Collect (error, HPL) point sets ------------------------------------
sets = struct('label', {}, 'err', {}, 'hpl', {}, 'color', {}, 'marker', {});

% Authentic run (required: ekf_out must exist in the workspace).
if ~exist('ekf_out', 'var')
    error('plot_stanford: variable ekf_out not found. Run ekf_runner first.');
end
sets(end+1) = struct('label', 'Authentic', ...
    'err', horizontal_error(ekf_out.pos, cfg.ref_pos), 'hpl', ekf_out.hpl(:), ...
    'color', [0.10 0.45 0.70], 'marker', '.');

% Optional spoofed scenarios.
if exist('scenario_runs', 'var') && ~isempty(scenario_runs)
    cmap = lines(numel(scenario_runs) + 2);
    for k = 1:numel(scenario_runs)
        sets(end+1) = struct('label', scenario_runs(k).label, ...
            'err', horizontal_error(scenario_runs(k).ekf.pos, cfg.ref_pos), ...
            'hpl', scenario_runs(k).ekf.hpl(:), ...
            'color', cmap(k+1,:), 'marker', '.'); %#ok<SAGROW>
    end
end

%% --- Axis range ---------------------------------------------------------
all_err = []; all_hpl = [];
for s = 1:numel(sets)
    all_err = [all_err; sets(s).err]; %#ok<AGROW>
    all_hpl = [all_hpl; sets(s).hpl]; %#ok<AGROW>
end
ax_max = max([REF_2X*1.05, max(all_hpl,[],'omitnan')*1.10, max(all_err,[],'omitnan')*1.10]);
ax_max = ceil(ax_max/50)*50;

%% --- Plot ---------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [100 100 640 600]);
hold on; box on;

% HPL = error diagonal (boundary of the misleading-information region).
plot([0 ax_max], [0 ax_max], 'k-', 'LineWidth', 1.0);
text(ax_max*0.62, ax_max*0.70, 'HPL = error', 'Rotation', 45, ...
     'FontSize', 9, 'Color', [0.3 0.3 0.3]);

% Limit lines.
plot([0 ax_max], [REF_2X REF_2X], '--', 'Color', [0.75 0.1 0.1], 'LineWidth', 1.0);
text(ax_max*0.02, REF_2X+ax_max*0.012, sprintf('2 \\times RNP-0.1 ref = %.1f m', REF_2X), ...
     'FontSize', 9, 'Color', [0.75 0.1 0.1]);
plot([0 ax_max], [RNP_ACCURACY RNP_ACCURACY], ':', 'Color', [0.85 0.45 0.0], 'LineWidth', 1.0);
text(ax_max*0.02, RNP_ACCURACY+ax_max*0.012, ...
     sprintf('RNP-0.1 accuracy scale = %.1f m', RNP_ACCURACY), ...
     'FontSize', 9, 'Color', [0.85 0.45 0.0]);

% Epoch points.
h = gobjects(numel(sets),1);
for s = 1:numel(sets)
    h(s) = plot(sets(s).err, sets(s).hpl, sets(s).marker, ...
        'Color', sets(s).color, 'MarkerSize', 8, 'DisplayName', sets(s).label);
end

xlabel('Horizontal position error [m]');
ylabel('Horizontal protection level (HPL) [m]');
title('Stanford integrity diagram');
xlim([0 ax_max]); ylim([0 ax_max]);
axis square;
legend(h, 'Location', 'southeast');
set(gca, 'FontSize', 10);

%% --- Save ---------------------------------------------------------------
out_dir = fullfile(cfg.root, 'results', 'figures');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
out_path = fullfile(out_dir, 'stanford_diagram.png');
print(fig, out_path, '-dpng', '-r300');
fprintf('Stanford diagram saved to: %s\n', out_path);

%% --- Bounding statistics for the thesis ---------------------------------
fprintf('\n--- Stanford / integrity statistics ---\n');
for s = 1:numel(sets)
    e = sets(s).err; p = sets(s).hpl;
    valid = ~isnan(e) & ~isnan(p);
    e = e(valid); p = p(valid);
    frac_bounded = mean(p >= e) * 100;
    frac_below_ref2x = mean(p < REF_2X) * 100;
    n_mi         = sum(e > p);          % misleading-information epochs
    fprintf('  %-22s  n=%4d  HPL bounds err: %.1f%%  HPL<2xRNP: %.1f%%  MI epochs: %d\n', ...
        sets(s).label, numel(e), frac_bounded, frac_below_ref2x, n_mi);
    fprintf('      err mean=%.1f max=%.1f m   HPL mean=%.1f max=%.1f m\n', ...
        mean(e), max(e), mean(p), max(p));
end
fprintf('---------------------------------------\n\n');
