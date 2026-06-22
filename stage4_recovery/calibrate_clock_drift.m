% calibrate_clock_drift.m
%
% Calibrates cfg.ekf.clk_drift_init as the EFFECTIVE GPS receiver clock-bias
% drift observed in the FINAL transmit-time corrected pseudorange model.
%
% IMPORTANT FRAMING (thesis-safe):
%   This is the effective clock-bias drift seen by the estimator after
%   transmit-time + group-delay correction, NOT an intrinsic Leica GR50
%   oscillator-stability measurement. The previous value (-0.02454 m/s,
%   i.e. -0.7362 m/epoch) was calibrated BEFORE the transmit-time fix and was
%   most likely an estimator artefact: reception-time satellite-position error
%   leaking into the jointly-estimated clock state. Receiver clock bias is
%   estimated jointly with position from pseudoranges, so measurement-model
%   error biases the clock estimate (standard SPP result).
%
% METHOD (independent measurement first, EKF cross-check second):
%   PRIMARY  — GPS-only snapshot WLS per epoch, using corrected_pseudorange
%              (transmit-time). GPS-only is used deliberately: the EKF clock
%              state x(7) is the GPS master clock, and a multi-constellation
%              WLS without ISB would re-contaminate the clock with inter-system
%              bias. Build the clock-bias series, take jump-rejected epoch-to-
%              epoch deltas, report mean/median/MAD/std + linear-fit slope.
%   CROSS-CHECK — EKF authentic clock-bias drift over the same window. If the
%              WLS and EKF drifts agree in sign/order, the value is trustworthy.
%              If they disagree, investigate before changing config.
%
%   The mean-delta vs linear-fit-slope agreement is the internal trust check:
%   a clean linear drift has matching slope and mean-delta; a transient- or
%   noise-dominated series does not.
%
% WINDOWS: 100 (comparability with original), 200, and full day (robustness).
%
% OUTPUT: prints recommended clk_drift_init and matching Test 2 bound.
%         DOES NOT modify config.m — recommendation only.
%
% Run from project root:
%   run('stage4_recovery/calibrate_clock_drift.m')

%clear; 
clc;

PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
addpath(PROJECT_ROOT);
addpath(fullfile(PROJECT_ROOT, 'utils'));
addpath(fullfile(PROJECT_ROOT, 'stage1_detection'));
addpath(fullfile(PROJECT_ROOT, 'stage2_identification'));
addpath(fullfile(PROJECT_ROOT, 'stage4_recovery'));
cd(PROJECT_ROOT);
config;
cfg.verbose = 0;

fprintf('=======================================================\n');
fprintf('  CLOCK-DRIFT CALIBRATION (transmit-time model)\n');
fprintf('  Effective GPS clock-bias drift, NOT oscillator stability\n');
fprintf('  %s\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
fprintf('=======================================================\n\n');

obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);

epochs_all = unique(obs.GPS.time);
n_all      = numel(epochs_all);
rec        = cfg.ref_pos(:);
JUMP_M     = 3.0;          % jump-rejection threshold (matches Test 2)
C_LIGHT    = 299792458.0;

% ---------------------------------------------------------------------------
% PRIMARY: build GPS-only snapshot-WLS clock-bias series (transmit-time)
% ---------------------------------------------------------------------------
fprintf('[1/3] Building GPS-only snapshot-WLS clock series (transmit-time)...\n');

clk_wls = nan(n_all, 1);
for ei = 1:n_all
    t_e  = epochs_all(ei);
    m    = (obs.GPS.time == t_e);
    prns = obs.GPS.prn(m);
    prs  = obs.GPS.pseudorange_L1(m);

    sp_list = []; pr_list = []; w_list = [];
    for k = 1:numel(prns)
        pr_raw = prs(k);
        if isnan(pr_raw) || pr_raw <= 0, continue; end
        try
            [pr_corr, sp_tx] = corrected_pseudorange(pr_raw, prns(k), 'GPS', t_e, rec, nav, cfg);
        catch
            continue
        end
        if isnan(pr_corr), continue; end
        sp_list(end+1,:) = sp_tx(:)';                 %#ok<AGROW>
        pr_list(end+1,1) = pr_corr;                   %#ok<AGROW>
        w_list(end+1,1)  = 1/cfg.ekf.meas_noise_GPS;  %#ok<AGROW>
    end

    if numel(pr_list) < cfg.identify.min_sats, continue; end
    [~, clk_b] = wls_solver(pr_list, sp_list, w_list, rec);
    clk_wls(ei) = clk_b;
end

n_valid_wls = sum(~isnan(clk_wls));
fprintf('      WLS clock series: %d / %d epochs solved.\n\n', n_valid_wls, n_all);

% ---------------------------------------------------------------------------
% CROSS-CHECK: EKF authentic clock-bias series
% ---------------------------------------------------------------------------
fprintf('[2/3] Running authentic EKF for cross-check...\n');
ekf = ekf_runner(obs, nav, {}, cfg);
clk_ekf = ekf.clk_bias(:);
fprintf('      EKF clock series: %d epochs.\n\n', numel(clk_ekf));

% ---------------------------------------------------------------------------
% Statistics per window
% ---------------------------------------------------------------------------
fprintf('[3/3] Computing drift statistics per window...\n\n');

windows = {1:100, 1:200, 1:n_all};
wnames  = {'1:100 (orig window)', '1:200', sprintf('1:%d (full day)', n_all)};

stats = @(series, win) drift_stats(series, win, JUMP_M);

fprintf('%-22s | %-28s | %-22s\n', 'Window', 'GPS-only WLS (PRIMARY)', 'EKF (cross-check)');
fprintf('%s\n', repmat('-', 1, 78));
for wi = 1:numel(windows)
    sw = stats(clk_wls,  windows{wi});
    se = stats(clk_ekf,  windows{wi});
    fprintf('%-22s | mean=%+.4f slope=%+.4f | mean=%+.4f slope=%+.4f\n', ...
        wnames{wi}, sw.mean_d, sw.slope, se.mean_d, se.slope);
end
fprintf('%s\n\n', repmat('-', 1, 78));

% Detailed report on the comparability window (1:100), the primary basis.
sw100 = stats(clk_wls, 1:100);
se100 = stats(clk_ekf, 1:100);

fprintf('PRIMARY RESULT — GPS-only WLS, window 1:100:\n');
fprintf('  epochs used (after jump reject): %d   (jumps rejected: %d)\n', sw100.n_used, sw100.n_jumps);
fprintf('  mean delta:    %+.4f m/epoch\n', sw100.mean_d);
fprintf('  median delta:  %+.4f m/epoch\n', sw100.median_d);
fprintf('  MAD sigma:      %.4f m/epoch\n', sw100.mad_sigma);
fprintf('  std (rejected): %.4f m/epoch\n', sw100.std_d);
fprintf('  linear slope:  %+.4f m/epoch\n', sw100.slope);
fprintf('  slope vs mean: %+.4f m/epoch  (agreement check; ~0 = clean drift)\n', sw100.slope - sw100.mean_d);

fprintf('\nCROSS-CHECK — EKF, window 1:100:\n');
fprintf('  mean delta:    %+.4f m/epoch   slope: %+.4f m/epoch\n', se100.mean_d, se100.slope);

% Agreement verdict
agree_sign  = sign(sw100.mean_d) == sign(se100.mean_d) || abs(sw100.mean_d) < 0.01;
agree_order = abs(sw100.mean_d - se100.mean_d) < 0.05;

fprintf('\n--- VERDICT ---\n');
if agree_sign && agree_order
    rec_delta = sw100.mean_d;                 % use the independent WLS value
    rec_init  = rec_delta / cfg.ekf.dt;       % m/s for config
    sigma3    = 3 * max(sw100.mad_sigma, sw100.std_d);  % conservative 3-sigma
    fprintf('  WLS and EKF AGREE (sign + order). Recommendation is trustworthy.\n\n');
    fprintf('  RECOMMENDED config.m:\n');
    fprintf('    cfg.ekf.clk_drift_init = %+.6f;   %% m/s  (= %+.4f m/epoch / %g s)\n', ...
        rec_init, rec_delta, cfg.ekf.dt);
    fprintf('    %% Effective GPS clock-bias drift, transmit-time corrected model.\n');
    fprintf('    %% NOT oscillator stability. Supersedes -0.02454 (pre-transmit-time,\n');
    fprintf('    %% contaminated by reception-time satellite-position error).\n');
    fprintf('    %% Calibrated GPS-only snapshot WLS, BUCU 17-May-2026, window 1:100.\n\n');
    fprintf('  RECOMMENDED test_stage4_integration.m Test 2:\n');
    fprintf('    expected_delta = %+.4f m/epoch  (clk_drift_init * dt)\n', rec_init*cfg.ekf.dt);
    fprintf('    3-sigma bound  = %.4f m/epoch  (3 * %.4f)\n', sigma3, sigma3/3);
else
    fprintf('  WLS and EKF DISAGREE — DO NOT change config yet.\n');
    fprintf('    WLS mean delta = %+.4f m/epoch\n', sw100.mean_d);
    fprintf('    EKF mean delta = %+.4f m/epoch\n', se100.mean_d);
    fprintf('    Investigate: possible state absorption or transient contamination.\n');
end

fprintf('\n=======================================================\n');


%% =========================================================================
%  LOCAL HELPER: drift_stats
% =========================================================================
function s = drift_stats(series, win, jump_m)
    win = win(win >= 1 & win <= numel(series));
    x   = series(win);
    ep  = (1:numel(x))';
    valid = ~isnan(x);

    d      = diff(x);
    d      = d(~isnan(d));
    smooth = d(abs(d) < jump_m);

    s.n_used    = numel(smooth);
    s.n_jumps   = sum(abs(d) >= jump_m);
    if isempty(smooth)
        s.mean_d=NaN; s.median_d=NaN; s.mad_sigma=NaN; s.std_d=NaN;
    else
        s.mean_d    = mean(smooth);
        s.median_d  = median(smooth);
        s.mad_sigma = 1.4826 * median(abs(smooth - median(smooth)));  % MAD->sigma
        s.std_d     = std(smooth);
    end

    if sum(valid) >= 2
        p = polyfit(ep(valid), x(valid), 1);
        s.slope = p(1);
    else
        s.slope = NaN;
    end
end