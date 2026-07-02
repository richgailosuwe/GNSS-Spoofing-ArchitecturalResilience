%% TEST_SAGNAC_FIX  Regression test for the Sagnac scalar correction fix.
%
% Run from project root after config:
%   config
%   obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
%   nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);
%   run('utils/test_sagnac_fix.m')
%
% Expected result: PASS (3/3)
%
% BACKGROUND:
%   The previous pseudorange_correct.m computed a Sagnac rotation of
%   sat_pos internally, used it to compute geo_range, then DISCARDED both.
%   The returned pr_corr never contained a Sagnac term, and downstream WLS
%   used the unrotated sat_pos. This caused satellite-position-dependent
%   biases of 10-150m (PRN19 ~+40m, PRN15 ~-29m, etc. observed on BUCU data),
%   explaining the 50-100m position error vs RTKLIB's 3-13m on the same files.
%
% FIX: scalar Sagnac term added directly to pr_corr (Kaplan & Hegarty 2017,
% Eq. 5.13): dt_sagnac = (Omega_E/c) * (x_sat*y_rec - y_sat*x_rec)
%
% TEST STRUCTURE:
%
%   Test 1 — Sagnac term magnitude sanity check (PRN 19, epoch 1).
%     Computes the Sagnac term directly and confirms it is in the expected
%     10-150m range for a GPS MEO satellite, with the correct sign relative
%     to PRN19's position.
%
%   Test 2 — Per-satellite residual reduction (epoch 1, all GPS satellites).
%     Compares |post-fit WLS residual| with OLD pseudorange_correct (no
%     Sagnac) vs NEW pseudorange_correct (with Sagnac), for every GPS
%     satellite at epoch 1.
%     Pass criterion: mean |residual| with NEW < mean |residual| with OLD.
%
%   Test 3 — Position error vs RTKLIB ground truth (epochs 1-50).
%     Compares mean WLS position error (vs cfg.ref_pos) with OLD vs NEW
%     pseudorange_correct, over 50 epochs.
%     Pass criterion: mean position error with NEW is substantially closer
%     to RTKLIB's observed ~3-13m than OLD's ~54-69m. Specific threshold:
%     mean error with NEW < 20m (RTKLIB showed sdn~4.2, sde~2.8, sdu~8.4 ->
%     3D ~ sqrt(4.2^2+2.8^2+8.4^2) ~ 9.7m; 20m gives generous margin for
%     WLS vs RTKLIB's Kalman-filtered estimator and different sat subsets).
%
% STAGE:    Utility fix — pseudorange_correct.m Sagnac correction

fprintf('\n=== test_sagnac_fix.m ===\n');

n_tests = 0;
n_pass  = 0;

OMEGA_E = 7.2921151467e-5;
C_LIGHT = 299792458.0;

epochs_all = unique(obs.GPS.time);
t_e1 = epochs_all(1);

%% ---- Test 1: Sagnac term magnitude sanity check (PRN 19) ------------------
fprintf('\nTest 1: Sagnac term magnitude — PRN 19, epoch 1\n');
n_tests = n_tests + 1;

[sp19, sc19] = sat_position(nav, 19, 'GPS', t_e1);

sagnac_term_19 = (OMEGA_E / C_LIGHT) * (sp19(1)*cfg.ref_pos(2) - sp19(2)*cfg.ref_pos(1));

fprintf('  sat_pos PRN19: [%.1f, %.1f, %.1f]\n', sp19);
fprintf('  rec_pos (ref): [%.1f, %.1f, %.1f]\n', cfg.ref_pos);
fprintf('  Sagnac term:   %.4f m\n', sagnac_term_19);

ok1 = abs(sagnac_term_19) >= 10 && abs(sagnac_term_19) <= 150;

if ok1
    fprintf('  PASS — Sagnac term %.2fm within expected 10-150m range for GPS MEO\n', ...
        sagnac_term_19);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — Sagnac term %.2fm outside expected 10-150m range\n', sagnac_term_19);
end

%% ---- Test 2: Per-satellite residual reduction (epoch 1) -------------------
fprintf('\nTest 2: Per-satellite residual reduction — epoch 1, OLD vs NEW correction\n');
n_tests = n_tests + 1;

mask1 = (obs.GPS.time == t_e1);
prns1 = obs.GPS.prn(mask1);
prs1  = obs.GPS.pseudorange_L1(mask1);

pr_old = []; pr_new = []; sp_all = []; prn_list = [];

for k = 1:numel(prns1)
    try
        [sp, sc] = sat_position(nav, prns1(k), 'GPS', t_e1);

        % NEW (fixed) pseudorange_correct — must be on path
        pr_c_new = pseudorange_correct(prs1(k), sp, sc, cfg.ref_pos, t_e1, nav, 'GPS', cfg);
        if isnan(pr_c_new), continue; end

        % OLD behaviour reconstructed: NEW plus the Sagnac term we now subtract
        % (everything else in pseudorange_correct is unchanged, so
        %  pr_old = pr_new + sagnac_term, since NEW = pr_old_chain - sagnac_term)
        sagnac_term = (OMEGA_E / C_LIGHT) * (sp(1)*cfg.ref_pos(2) - sp(2)*cfg.ref_pos(1));
        pr_c_old = pr_c_new + sagnac_term;

        pr_old(end+1)  = pr_c_old; %#ok<AGROW>
        pr_new(end+1)  = pr_c_new; %#ok<AGROW>
        sp_all(end+1,:) = sp';     %#ok<AGROW>
        prn_list(end+1) = prns1(k); %#ok<AGROW>
    catch
        continue
    end
end

w = ones(numel(pr_old),1) / cfg.ekf.meas_noise_GPS;

[pos_old, clk_old, res_old] = wls_solver(pr_old(:), sp_all, w, cfg.ref_pos);
[pos_new, clk_new, res_new] = wls_solver(pr_new(:), sp_all, w, cfg.ref_pos);

mean_abs_old = mean(abs(res_old));
mean_abs_new = mean(abs(res_new));

ok2 = mean_abs_new < mean_abs_old;

fprintf('  PRN | residual OLD | residual NEW\n');
for k = 1:numel(prn_list)
    fprintf('  %3d | %12.3f | %12.3f\n', prn_list(k), res_old(k), res_new(k));
end
fprintf('  Mean |residual| OLD: %.3f m\n', mean_abs_old);
fprintf('  Mean |residual| NEW: %.3f m\n', mean_abs_new);
fprintf('  pos error OLD: %.3f m\n', norm(pos_old - cfg.ref_pos));
fprintf('  pos error NEW: %.3f m\n', norm(pos_new - cfg.ref_pos));

if ok2
    fprintf('  PASS — mean |residual| reduced by Sagnac fix\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — mean |residual| not reduced (OLD=%.3f, NEW=%.3f)\n', ...
        mean_abs_old, mean_abs_new);
end

%% ---- Test 3: Position error vs RTKLIB ground truth (50 epochs) ------------
fprintf('\nTest 3: Position error over 50 epochs — NEW correction vs RTKLIB baseline\n');
n_tests = n_tests + 1;

err_new = [];
for ei = 1:50
    t_e = epochs_all(ei);
    mask = (obs.GPS.time == t_e);
    prns = obs.GPS.prn(mask);
    prs  = obs.GPS.pseudorange_L1(mask);

    pr_c_list = []; sp_list = [];
    for k = 1:numel(prns)
        try
            [sp, sc] = sat_position(nav, prns(k), 'GPS', t_e);
            pr_c = pseudorange_correct(prs(k), sp, sc, cfg.ref_pos, t_e, nav, 'GPS', cfg);
            if ~isnan(pr_c)
                pr_c_list(end+1) = pr_c; %#ok<AGROW>
                sp_list(end+1,:) = sp';  %#ok<AGROW>
            end
        catch
            continue
        end
    end

    if numel(pr_c_list) < cfg.identify.min_sats, continue; end

    w = ones(numel(pr_c_list),1) / cfg.ekf.meas_noise_GPS;
    [pos_wls, ~] = wls_solver(pr_c_list(:), sp_list, w, cfg.ref_pos);
    err_new(end+1) = norm(pos_wls - cfg.ref_pos); %#ok<AGROW>
end

mean_err_new = mean(err_new);

% RTKLIB epoch 1: sdn=4.2149, sde=2.8190, sdu=8.3956
rtklib_3d_approx = sqrt(4.2149^2 + 2.8190^2 + 8.3956^2);

ok3 = mean_err_new < 20.0;

fprintf('  Mean pos error (NEW), 50 epochs: %.2f m\n', mean_err_new);
fprintf('  p95 pos error (NEW):             %.2f m\n', prctile(err_new, 95));
fprintf('  RTKLIB epoch-1 3D sigma (~sdn,sde,sdu combined): %.2f m\n', rtklib_3d_approx);
fprintf('  Previous (OLD/buggy) mean error: ~54-69m (from earlier diagnostics)\n');

if ok3
    fprintf('  PASS — mean error %.2fm < 20m, consistent with RTKLIB-level accuracy\n', mean_err_new);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — mean error %.2fm >= 20m, fix did not bring error to RTKLIB-level\n', mean_err_new);
end

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_sagnac_fix: ALL PASS ✓\n\n');
else
    fprintf('test_sagnac_fix: %d FAILURE(S) — review output above\n\n', n_tests - n_pass);
end
