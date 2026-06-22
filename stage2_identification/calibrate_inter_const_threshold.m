% calibrate_inter_const_threshold.m
%
% Calibrates cfg.identify.inter_const_threshold from authentic BUCU00ROU data.
%
% METHOD:
%   Runs inter_constellation across all authentic epochs, collects every
%   pairwise distance between constellation solutions, then sets the threshold
%   at the 99.9th percentile of that distribution.
%
%   This is the same methodology used for Stage 1 thresholds:
%     - cfg.detect.residual_threshold          = 99.9th pct of authentic residuals
%     - cfg.detect.clock_consistency_threshold = max authentic clock drift
%   Source: empirical calibration from BUCU00ROU_R_20261370000_01D, 17-May-2026
%
% OUTPUT:
%   Prints recommended cfg.identify.inter_const_threshold value.
%   Also saves full distribution to results/logs/inter_const_calibration.mat
%
% Run from project root:
%   run('stage2_identification/calibrate_inter_const_threshold.m')

clear; clc;

PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
addpath(PROJECT_ROOT);
addpath(fullfile(PROJECT_ROOT, 'utils'));
addpath(fullfile(PROJECT_ROOT, 'stage1_detection'));
addpath(fullfile(PROJECT_ROOT, 'stage2_identification'));
cd(PROJECT_ROOT);
config;

% Temporarily silence verbose output from inter_constellation
cfg.verbose = 0;

fprintf('=======================================================\n');
fprintf('  INTER-CONSTELLATION THRESHOLD CALIBRATION\n');
fprintf('  Source: BUCU00ROU authentic data, 17-May-2026\n');
fprintf('  %s\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
fprintf('=======================================================\n\n');

% -------------------------------------------------------------------------
% Load data
% -------------------------------------------------------------------------
fprintf('[1/4] Loading RINEX files...\n');
obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);

all_times = unique(obs.GPS.time);
n_epochs  = length(all_times);
fprintf('      %d epochs to process (30s interval, 24h)\n\n', n_epochs);

% -------------------------------------------------------------------------
% Loop all epochs — collect pairwise distances
% -------------------------------------------------------------------------
fprintf('[2/4] Processing epochs...\n');

all_pairwise_dists = [];   % accumulates every pairwise distance (m)
epochs_solved      = 0;    % epochs where >= 2 constellations solved
epochs_skipped     = 0;    % epochs where < 2 constellations solved

PRINT_INTERVAL = 100;      % progress update every N epochs

for ep = 1:n_epochs

    t = all_times(ep);

    % Build obs_epoch for this epoch
    obs_epoch = build_obs_epoch(obs, t);
    if length(obs_epoch.prn) < cfg.identify.min_sats
        epochs_skipped = epochs_skipped + 1;
        continue;
    end

    % Run inter_constellation (verbose=0 suppresses output)
    try
        result = inter_constellation(obs_epoch, nav, cfg, t);
    catch
        epochs_skipped = epochs_skipped + 1;
        continue;
    end

    if result.n_constellations_solved < 2
        epochs_skipped = epochs_skipped + 1;
        continue;
    end

    % Collect upper triangle of pairwise distance matrix
    n_c = result.n_constellations_solved;
    for i = 1:n_c
        for j = i+1:n_c
            d = result.pairwise_distances(i,j);
            if ~isnan(d) && d >= 0
                all_pairwise_dists(end+1) = d; %#ok<AGROW>
            end
        end
    end

    epochs_solved = epochs_solved + 1;

    if mod(ep, PRINT_INTERVAL) == 0
        fprintf('      Epoch %4d / %4d  (collected %d distances so far)\n', ...
            ep, n_epochs, length(all_pairwise_dists));
    end

end

fprintf('\n      Done. %d epochs solved, %d skipped.\n', epochs_solved, epochs_skipped);
fprintf('      Total pairwise distances collected: %d\n\n', length(all_pairwise_dists));

% -------------------------------------------------------------------------
% Compute percentile distribution
% -------------------------------------------------------------------------
fprintf('[3/4] Computing distribution...\n');

if isempty(all_pairwise_dists)
    error('No pairwise distances collected — check data and path setup.');
end

pct_50   = prctile(all_pairwise_dists, 50);
pct_90   = prctile(all_pairwise_dists, 90);
pct_95   = prctile(all_pairwise_dists, 95);
pct_99   = prctile(all_pairwise_dists, 99);
pct_99_9 = prctile(all_pairwise_dists, 99.9);
pct_max  = max(all_pairwise_dists);

fprintf('\n  Authentic pairwise distance distribution:\n');
fprintf('    50th  percentile : %7.2f m\n', pct_50);
fprintf('    90th  percentile : %7.2f m\n', pct_90);
fprintf('    95th  percentile : %7.2f m\n', pct_95);
fprintf('    99th  percentile : %7.2f m\n', pct_99);
fprintf('    99.9th percentile: %7.2f m  <-- recommended threshold\n', pct_99_9);
fprintf('    Maximum          : %7.2f m\n', pct_max);

% Round up to nearest 5m for a clean config value
recommended = ceil(pct_99_9 / 5) * 5;
fprintf('\n  Recommended cfg.identify.inter_const_threshold = %.1f m\n', recommended);
fprintf('  (99.9th percentile = %.2f m, rounded up to nearest 5 m)\n', pct_99_9);

% -------------------------------------------------------------------------
% Save results
% -------------------------------------------------------------------------
fprintf('\n[4/4] Saving calibration data...\n');

save_path = fullfile(cfg.paths.logs, 'inter_const_calibration.mat');
save(save_path, 'all_pairwise_dists', 'pct_99_9', 'recommended', ...
    'epochs_solved', 'epochs_skipped', 'n_epochs');
fprintf('      Saved to: %s\n', save_path);

fprintf('\n=======================================================\n');
fprintf('  CALIBRATION COMPLETE\n');
fprintf('  Update config.m:\n');
fprintf('  cfg.identify.inter_const_threshold = %.1f;\n', recommended);
fprintf('  %% 99.9th pct of authentic pairwise distances\n');
fprintf('  %% Source: BUCU00ROU_R_20261370000_01D, 17-May-2026\n');
fprintf('=======================================================\n\n');


%% =========================================================================
%  LOCAL HELPER: build_obs_epoch
% =========================================================================
function obs_epoch = build_obs_epoch(obs, t)

prn_all   = [];
const_all = {};
pr_all    = [];
cn0_all   = [];

PR_FIELD  = 'pseudorange_L1';
CN0_FIELD = 'cn0';
constellations = {'GPS','Galileo','BeiDou','GLONASS'};

for c = 1:length(constellations)
    cname = constellations{c};
    if ~isfield(obs, cname), continue; end
    obs_c = obs.(cname);

    time_mask = (obs_c.time == t);
    if ~any(time_mask), continue; end
    prns_t = obs_c.prn(time_mask);

    for k = 1:length(prns_t)
        prn_k = prns_t(k);

        mask = (obs_c.time == t) & (obs_c.prn == prn_k);
        idx  = find(mask, 1, 'first');
        if isempty(idx), continue; end

        pr_val  = NaN;
        cn0_val = NaN;
        if isfield(obs_c, PR_FIELD),  pr_val  = obs_c.(PR_FIELD)(idx);  end
        if isfield(obs_c, CN0_FIELD), cn0_val = obs_c.(CN0_FIELD)(idx); end
        if isnan(pr_val) || pr_val <= 0, continue; end

        prn_all(end+1)   = prn_k;   %#ok<AGROW>
        const_all{end+1} = cname;   %#ok<AGROW>
        pr_all(end+1)    = pr_val;  %#ok<AGROW>
        cn0_all(end+1)   = cn0_val; %#ok<AGROW>
    end
end

obs_epoch.prn           = prn_all;
obs_epoch.constellation = const_all;
obs_epoch.pseudorange   = pr_all;
obs_epoch.cn0           = cn0_all;

end % build_obs_epoch

%% =========================================================================
%  CALIBRATION RESULTS — BUCU00ROU_R_20261370000_01D, 17-May-2026
%  Run date: 2026-06-05
% =========================================================================
%
%  Dataset: 2880 epochs (30s interval, 24h), 17280 pairwise distances
%
%  Authentic inter-constellation pairwise distance distribution:
%    50th  percentile :   66.11 m
%    90th  percentile :  110.99 m
%    95th  percentile :  124.76 m
%    99th  percentile :  151.30 m
%    99.9th percentile:  176.97 m  <-- threshold basis
%    Maximum          :  181.29 m
%
%  Recommended threshold: 180.0 m
%  (99.9th percentile = 176.97 m, rounded up to nearest 5 m)
%
%  INTERPRETATION:
%    The wide distribution (50th pct = 66 m, max = 181 m) is driven by
%    elevated BeiDou and GLONASS measurement noise at this station:
%      GPS     sigma = 18.3 m  (sigma^2 = 333  m^2)
%      Galileo sigma = 17.3 m  (sigma^2 = 301  m^2)
%      BeiDou  sigma = 70.5 m  (sigma^2 = 4972 m^2) — edge of coverage
%      GLONASS sigma = 58.9 m  (sigma^2 = 3476 m^2) — FDMA inter-freq biases
%
%    The tightest authentic pair is GPS<->Galileo; the widest is
%    BeiDou<->Galileo and GLONASS<->Galileo.
%
%  DETECTION SENSITIVITY LIMIT:
%    With threshold = 180 m and Humphreys drag-off at 5 m/epoch starting
%    at epoch 120, inter_constellation can only detect spoofing from
%    epoch 156 onward (120 + 180/5 = 156). Below that offset, the spoofed
%    constellation's position error is within authentic noise bounds and
%    is indistinguishable from genuine inter-constellation disagreement.
%    This limit must be documented in Chapter 6 and cross-referenced with
%    Stage 1 detection (which fires at epoch 120) and RAIM-FDE.
%
%  METHODOLOGY NOTE (for Chapter 6 / examiner defence):
%    Threshold set at 99.9th percentile of authentic pairwise distances,
%    consistent with the methodology used for Stage 1 thresholds
%    (residual_threshold, clock_consistency_threshold). This gives a
%    false alarm rate of 0.1% per epoch per constellation pair on
%    authentic data, which is acceptable for the RNP-0.1 application.