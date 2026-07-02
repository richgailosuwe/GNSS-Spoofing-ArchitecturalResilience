%% TEST_SAT_BIAS_CALIBRATION  Calibrate/validate split for per-satellite bias correction.
%
% Run from project root after config:
%   config
%   obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
%   nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);
%   run('stage4_recovery/calibration/test_sat_bias_calibration.m')
%
% Expected result: PASS (2/2), plus one informational (non-pass/fail) report.
%
% METHODOLOGY (DGNSS-style, train/validate split — NOT overfitting):
%
%   Test 1 — Calibrate on epochs 1-100, validate on epochs 101-200.
%     The bias table is computed ONLY from epochs 1-100.
%     It is then APPLIED to epochs 101-200 (never seen during calibration).
%     Pass criterion: mean |position error| over epochs 101-200 WITH bias
%     correction < mean |position error| over epochs 101-200 WITHOUT
%     correction.  This proves the bias is a stable satellite property
%     that generalises to new epochs, not an artefact of the calibration
%     window.
%
%   Test 2 — Bias stability across two independent windows.
%     Compute bias_table_A from epochs 1-100 and bias_table_B from epochs
%     101-200 INDEPENDENTLY.  Compare the per-PRN bias values between A and B.
%     Pass criterion: correlation(bias_A, bias_B) > 0.5 for PRNs present in
%     both.  A strong positive correlation proves the "bias" is a repeatable
%     per-satellite property (ionospheric pierce point + ephemeris error at
%     this location), not epoch-specific noise that happens to average out.
%
%   Informational — Long-gap stability (epochs 1-100 vs epochs 901-1000,
%     approximately 7.5 hours later at 30s interval).
%     Reports correlation(bias_A, bias_long_gap).  NOT a pass/fail criterion
%     — this characterises HOW OFTEN the bias table needs recalibration,
%     which is a thesis finding (Chapter 6) regardless of the result.
%
% STAGE:    4 — EKF Position Recovery (calibration validation)

fprintf('\n=== test_sat_bias_calibration.m ===\n');

n_tests = 0;
n_pass  = 0;

%% ---- Test 1: Calibrate ep1-100, validate ep101-200 ------------------------
fprintf('\nTest 1: Calibrate on ep1-100, validate on ep101-200 (held-out window)\n');
n_tests = n_tests + 1;

calib_range    = 1:100;
validate_range = 101:200;

bias_table = calibrate_sat_bias(obs, nav, cfg, calib_range, 'GPS');

% Position error over validation window WITHOUT bias correction.
err_no_corr  = compute_pos_errors(obs, nav, cfg, validate_range, []);

% Position error over validation window WITH bias correction.
err_with_corr = compute_pos_errors(obs, nav, cfg, validate_range, bias_table);

mean_no_corr   = mean(err_no_corr);
mean_with_corr = mean(err_with_corr);

ok1 = mean_with_corr < mean_no_corr;

fprintf('  Calibration window:  epochs %d-%d\n', calib_range(1), calib_range(end));
fprintf('  Validation window:   epochs %d-%d (held out — not used for calibration)\n', ...
    validate_range(1), validate_range(end));
fprintf('  Satellites calibrated: %d\n', numel(bias_table.prn));
fprintf('  Mean pos error WITHOUT correction: %.2f m\n', mean_no_corr);
fprintf('  Mean pos error WITH correction:    %.2f m\n', mean_with_corr);
fprintf('  Improvement: %.2f m (%.1f%%)\n', mean_no_corr - mean_with_corr, ...
    100*(mean_no_corr - mean_with_corr)/mean_no_corr);

if ok1
    fprintf('  PASS — bias correction reduces position error on held-out epochs\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — bias correction did not improve held-out position error\n');
    fprintf('         (mean_with_corr=%.2f >= mean_no_corr=%.2f)\n', mean_with_corr, mean_no_corr);
end

%% ---- Test 2: Bias stability across independent windows --------------------
fprintf('\nTest 2: Bias stability — independent calibration on ep1-100 vs ep101-200\n');
n_tests = n_tests + 1;

bias_table_A = bias_table;  % already computed: ep1-100
bias_table_B = calibrate_sat_bias(obs, nav, cfg, 101:200, 'GPS');

% Match PRNs present in both tables.
common_prns = intersect(bias_table_A.prn, bias_table_B.prn);

bias_A_common = zeros(numel(common_prns),1);
bias_B_common = zeros(numel(common_prns),1);
for k = 1:numel(common_prns)
    bias_A_common(k) = bias_table_A.bias(bias_table_A.prn == common_prns(k));
    bias_B_common(k) = bias_table_B.bias(bias_table_B.prn == common_prns(k));
end

if numel(common_prns) < 3
    fprintf('  FAIL — insufficient common PRNs (%d) for correlation\n', numel(common_prns));
    ok2 = false;
else
    corr_AB = corr(bias_A_common, bias_B_common);
    ok2 = corr_AB > 0.5;

    fprintf('  Common PRNs: %d\n', numel(common_prns));
    fprintf('  Correlation(bias_ep1-100, bias_ep101-200): %.3f\n', corr_AB);
    fprintf('  PRN | bias_A (ep1-100) | bias_B (ep101-200)\n');
    for k = 1:numel(common_prns)
        fprintf('  %3d | %16.3f | %18.3f\n', common_prns(k), bias_A_common(k), bias_B_common(k));
    end

    if ok2
        fprintf('  PASS — correlation %.3f > 0.5: bias is a repeatable satellite property\n', corr_AB);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL — correlation %.3f <= 0.5: bias may be epoch-specific noise\n', corr_AB);
    end
end

%% ---- Informational: Long-gap stability (ep1-100 vs ep901-1000) -------------
fprintf('\nInformational: Long-gap bias stability — ep1-100 vs ep901-1000 (~7.5h later)\n');
fprintf('(NOT a pass/fail criterion — characterises recalibration frequency for Chapter 6)\n');

bias_table_longgap = calibrate_sat_bias(obs, nav, cfg, 901:1000, 'GPS');
common_prns_lg = intersect(bias_table_A.prn, bias_table_longgap.prn);

if numel(common_prns_lg) < 3
    fprintf('  Insufficient common PRNs (%d) for correlation\n', numel(common_prns_lg));
else
    bias_A_lg  = zeros(numel(common_prns_lg),1);
    bias_lg    = zeros(numel(common_prns_lg),1);
    for k = 1:numel(common_prns_lg)
        bias_A_lg(k) = bias_table_A.bias(bias_table_A.prn == common_prns_lg(k));
        bias_lg(k)   = bias_table_longgap.bias(bias_table_longgap.prn == common_prns_lg(k));
    end
    corr_lg = corr(bias_A_lg, bias_lg);

    fprintf('  Common PRNs: %d\n', numel(common_prns_lg));
    fprintf('  Correlation(bias_ep1-100, bias_ep901-1000): %.3f\n', corr_lg);
    if corr_lg > 0.5
        fprintf('  -> Bias remains correlated after ~7.5h: recalibration interval likely > 7.5h\n');
    else
        fprintf('  -> Bias correlation degrades by ~7.5h: recalibration interval likely < 7.5h\n');
        fprintf('     (Expected: GPS constellation geometry repeats ~ sidereal day = 23h56m,\n');
        fprintf('      so ionospheric pierce points for a given PRN shift over hours.)\n');
    end
end

%% ---- Save calibration table for use by ekf_runner --------------------------
output_dir = fullfile(cfg.root, 'results', 'calibration');
if ~isfolder(output_dir)
    mkdir(output_dir);
end
output_path = fullfile(output_dir, 'sat_bias_BUCU_GPS.mat');
save(output_path, 'bias_table');
fprintf('\nSaved calibration table (ep1-100): %s\n', output_path);
fprintf('NOTE: this table was calibrated on ep1-100. For production use with\n');
fprintf('ekf_runner on epochs >= 101, consider recalibrating on a window closer\n');
fprintf('to the epochs being processed (see long-gap result above).\n');

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS (+ 1 informational) ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_sat_bias_calibration: ALL PASS ✓\n\n');
else
    fprintf('test_sat_bias_calibration: %d FAILURE(S) — review output above\n\n', ...
        n_tests - n_pass);
end

%% ============================================================================
%  LOCAL HELPER
%% ============================================================================

function errs = compute_pos_errors(obs, nav, cfg, epoch_range, bias_table)
% COMPUTE_POS_ERRORS  WLS position error for each epoch in epoch_range,
% optionally applying a per-satellite bias correction (GPS only).
%
% bias_table = [] -> no correction applied (baseline).

    noise_map = struct( ...
        'GPS',     cfg.ekf.meas_noise_GPS, ...
        'Galileo', cfg.ekf.meas_noise_Galileo, ...
        'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
        'GLONASS', cfg.ekf.meas_noise_GLONASS);

    epochs_all = unique(obs.GPS.time);
    errs = [];

    apply_bias = ~isempty(bias_table);

    for ii = 1:numel(epoch_range)
        ei = epoch_range(ii);
        if ei < 1 || ei > numel(epochs_all), continue; end
        t_e = epochs_all(ei);

        all_pr = []; all_sp = []; all_w = [];
        constellations = {'GPS','Galileo','BeiDou','GLONASS'};
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs, cname), continue; end
            mask_t = (obs.(cname).time == t_e);
            if ~any(mask_t), continue; end
            prns_e = obs.(cname).prn(mask_t);
            prs_e  = obs.(cname).pseudorange_L1(mask_t);

            for k = 1:numel(prns_e)
                try
                    [sp, sc] = sat_position(nav, prns_e(k), cname, t_e);
                    pr_c     = pseudorange_correct(prs_e(k), sp, sc, cfg.ref_pos, t_e, nav, cname, cfg);
                    if isnan(pr_c), continue; end

                    if apply_bias && strcmp(cname, 'GPS')
                        pr_c = apply_sat_bias(pr_c, prns_e(k), bias_table);
                    end

                    all_pr(end+1)  = pr_c;        %#ok<AGROW>
                    all_sp(end+1,:) = sp';         %#ok<AGROW>
                    all_w(end+1)   = 1 / noise_map.(cname); %#ok<AGROW>
                catch
                    continue
                end
            end
        end

        if numel(all_pr) < cfg.identify.min_sats, continue; end

        [pos_wls, ~] = wls_solver(all_pr(:), all_sp, all_w(:), cfg.ref_pos);
        errs(end+1) = norm(pos_wls - cfg.ref_pos); %#ok<AGROW>
    end

end
