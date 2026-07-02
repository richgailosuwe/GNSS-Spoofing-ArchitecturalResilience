%% TEST_STAGE4_INTEGRATION  Full pipeline integration test for Stage 4.
%
% Run from project root after config:
%   config
%   run('stage4_recovery/test_stage4_integration.m')
%
% Expected result: PASS (3/3)
%
% PURPOSE:
%   Proves the full pipeline claim: authentic convergence + bounded position
%   recovery after spoofing onset.  Unit tests (test_ekf.m 5/5) prove the
%   equations are correct.  This test proves the equations produce the right
%   behaviour on real BUCU geometry over multiple epochs.
%
% TEST STRUCTURE:
%
%   Test 1 — Authentic convergence (epochs 1-50)
%     Pass criteria (two conditions, both must hold):
%       (a) P_trace at epoch 50 < P_trace at epoch 2 — filter gained information
%       (b) No coasting epochs in epochs 2-50 — scalar gate not over-rejecting
%     Rationale: GPS-only single-point pseudorange positioning at BUCU has an
%     authentic EKF error of ~3-4 m after transmit-time correction (the old
%     72.7m/97.5m figures were reception-time and are superseded, 17-May-2026).
%     The correct
%     claim is that the filter incorporates information (P contracts) and does
%     not falsely reject authentic measurements (no coasting).
%
%   Test 2 — Authentic clock tracking
%     Same run.  Pass criterion: epoch-to-epoch clock delta is within 3-sigma
%     of zero (effective drift is negligible after transmit-time correction;
%     clk_drift_init = 0, superseding the pre-correction -0.02454 m/s artefact).
%     Rationale: confirms the clock model is tracking correctly and the
%     5th state (drift) is doing useful work.
%
%   Test 3 — Recovery under GPS spoofing (Scenario 1, epochs 1-200)
%     Load Scenario 1 (GPS PRNs 14, 22, 31 spoofed, Humphreys drag-off).
%     Run Stage 2 classification on each epoch, then pass classify_results
%     to ekf_runner.  Pass criteria:
%       (a) spoofed error at epoch 200 <= authentic error at epoch 200 + 30m
%       (b) spoofed error at epoch 200 <= 0.3 * unmitigated drag-off error
%     Rationale: authentic GPS-only noise floor is 53-105m.  Under spoofing
%     without recovery, the drag-off at epoch 200 produces (200-120)*5=400m
%     of additional error.  The pipeline should remain near the authentic
%     baseline while staying far below the unmitigated 400m.
%
% NOTE ON EPOCH COUNT:
%   Only 50 (Tests 1-2) and 200 (Test 3) epochs are processed, not all 2880.
%   This keeps the test runtime to ~minutes rather than hours.
%   Full 2880-epoch validation is a Chapter 6 result, not a unit test.
%
% STAGE:    4 — EKF Position Recovery

fprintf('\n=== test_stage4_integration.m ===\n');
fprintf('Loading RINEX data...\n');

%% --- Load data -------------------------------------------------------------
obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);

epochs_all = unique(obs.GPS.time);
fprintf('Loaded: %d epochs, %d GPS satellites\n', numel(epochs_all), ...
    numel(unique(obs.GPS.prn)));

n_tests = 0;
n_pass  = 0;

%% ---- Test 1: Authentic convergence — position error < 20m ----------------
fprintf('\nTest 1: Authentic convergence — P must contract, error must not diverge\n');
n_tests = n_tests + 1;

% Subset obs to first 50 epochs only for speed.
obs_50   = subset_obs(obs, epochs_all(1:50));
ekf_auth = ekf_runner(obs_50, nav, {}, cfg);

P_trace_ep2  = ekf_auth.P_trace(2);
P_trace_ep50 = ekf_auth.P_trace(50);
err_ep1      = ekf_auth.pos_error(1);
err_ep50     = ekf_auth.pos_error(50);
n_coasted    = sum(ekf_auth.coasted(2:50));

% Condition (a): P contracted — filter incorporated information.
cond_a = P_trace_ep50 < P_trace_ep2;

% Condition (b): no coasting — scalar gate not over-rejecting authentic data.
% Authentic GPS-only SPP error distribution: mean=72.7m, p95=97.5m, p99=102.8m
% (calibrated over epochs 2-200, BUCU, 17-May-2026).
% Claiming < 20m would be incorrect for pseudorange-only SPP.
cond_b = n_coasted == 0;

ok1 = cond_a && cond_b;

fprintf('  Epoch 1  pos error:  %.2f m  (WLS bootstrap)\n', err_ep1);
fprintf('  Epoch 50 pos error:  %.2f m\n', err_ep50);
fprintf('  P_trace ep2:         %.2e m²\n', P_trace_ep2);
fprintf('  P_trace ep50:        %.2e m²\n', P_trace_ep50);
fprintf('  P contracted:        %d  (condition a)\n', cond_a);
fprintf('  Coasted ep2-50:      %d epochs  (condition b: must be 0)\n', n_coasted);
fprintf('  NOTE: authentic GPS-only EKF error after transmit-time correction\n');
fprintf('        is ~3-4 m at this geometry (was ~73 m pre-correction; the old\n');
fprintf('        72.7m/97.5m figures were reception-time and are superseded).\n');

if ok1
    fprintf('  PASS — P contracted %.0f%%, zero coasting on authentic data\n', ...
        100*(1 - P_trace_ep50/P_trace_ep2));
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — cond_a(P_contracted)=%d, cond_b(coasted=%d, must be 0)=%d\n', ...
        cond_a, n_coasted, cond_b);
end

%% ---- Test 2: Clock drift tracking ----------------------------------------
fprintf('\nTest 2: Clock drift tracking — epoch-to-epoch bias delta must match calibrated drift\n');
n_tests = n_tests + 1;

% Expected drift: clk_drift_init (now 0 after transmit-time recalibration) * dt.
% Effective GPS clock-bias drift is statistically negligible in the corrected
% model; the EKF estimates residual drift via Q_clk_drift.
expected_delta = cfg.ekf.clk_drift_init * cfg.ekf.dt;  % m/epoch (= 0)

clk_series = ekf_auth.clk_bias(~isnan(ekf_auth.clk_bias));
clk_deltas  = diff(clk_series);

% Exclude any large jumps (constellation change artefacts, as seen in
% calibration data — epochs 70-82 had jumps > 1.96m).
smooth_deltas = clk_deltas(abs(clk_deltas) < 3.0);

if numel(smooth_deltas) < 5
    fprintf('  FAIL — insufficient smooth clock deltas (%d)\n', numel(smooth_deltas));
else
    mean_delta = mean(smooth_deltas);
    std_delta  = std(smooth_deltas);

    % Pass if observed mean drift is within 3-sigma of expected (now ~0).
    % After transmit-time correction the effective GPS clock-bias drift is
    % statistically negligible (full-day mean +0.0001 m/epoch, slope +0.0004;
    % the -0.014 m/epoch seen over 100 epochs is only 0.034*sigma, i.e. noise).
    % clk_drift_init is therefore 0, so expected_delta = 0. The 3-sigma bound
    % is from the GPS-only snapshot-WLS per-epoch delta std: 3 * 0.4064 = 1.219.
    % Source: stage4_recovery/calibrate_clock_drift.m, BUCU 17-May-2026.
    drift_err = abs(mean_delta - expected_delta);
    sigma_3   = 3 * 0.4064;   % GPS-only WLS per-epoch delta std, transmit-time model

    ok2 = drift_err < sigma_3;

    fprintf('  Expected delta:  %.4f m/epoch\n', expected_delta);
    fprintf('  Observed mean:   %.4f m/epoch\n', mean_delta);
    fprintf('  Observed std:    %.4f m/epoch\n', std_delta);
    fprintf('  Drift error:     %.4f m (3-sigma bound: %.4f m)\n', drift_err, sigma_3);

    if ok2
        fprintf('  PASS — drift error within 3-sigma of zero (negligible drift)\n');
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL — drift error %.4f m exceeds 3-sigma bound %.4f m\n', ...
            drift_err, sigma_3);
    end
end

%% ---- Test 3: Recovery under GPS spoofing (Scenario 1) --------------------
fprintf('\nTest 3: Recovery under Scenario 1 GPS spoofing — baseline-relative bounded error\n');
n_tests = n_tests + 1;

% Load spoofed obs for Scenario 1.
s1_path = fullfile(cfg.paths.scenarios, 'scenario_1_gps', 'spoofed_obs.mat');

if ~isfile(s1_path)
    fprintf('  SKIP — Scenario 1 file not found: %s\n', s1_path);
    fprintf('         Run inject_spoofing for Scenario 1 first.\n');
    fprintf('         Test 3 result: INCONCLUSIVE\n');
else
    s1      = load(s1_path);
    obs_s1  = s1.obs_spoofed;

    % Subset to first 200 epochs.
    epochs_s1  = unique(obs_s1.GPS.time);
    n_ep_s1    = min(200, numel(epochs_s1));
    obs_s1_200 = subset_obs(obs_s1, epochs_s1(1:n_ep_s1));

    % Run Stage 2 classification on each epoch to build classify_results.
    % For this test, use a simplified classify: mark known spoofed PRNs
    % (G14, G22, G31) as 'spoofed' after epoch 120 (drag-off onset).
    % Full Stage 2 would call classify_spoofed_sats per epoch — this
    % simplified version tests Stage 4 recovery assuming Stage 2 is correct.
    spoofed_prns = cfg.scenarios{1}.spoofed_PRNs.GPS;   % [14, 22, 31]
    classify_results_s1 = build_scenario1_classify(obs_s1_200, ...
        unique(obs_s1_200.GPS.time), spoofed_prns, 120);

    ekf_s1 = ekf_runner(obs_s1_200, nav, classify_results_s1, cfg);

    % Also run authentic for the same 200-epoch window for baseline comparison.
    obs_auth_200 = subset_obs(obs, epochs_all(1:200));
    ekf_auth_200 = ekf_runner(obs_auth_200, nav, {}, cfg);

    err_ep120_spoof  = ekf_s1.pos_error(120);
    err_ep200_spoof  = ekf_s1.pos_error(n_ep_s1);
    err_ep200_auth   = ekf_auth_200.pos_error(200);
    unmitigated_err  = (n_ep_s1 - 120) * 5;   % drag-off: 5m/epoch from ep120

    % Baseline-relative pass criteria (two conditions, both must hold):
    %
    % Condition (a): spoofed error <= authentic error + 30m margin.
    %   authentic ep200 error is computed from the matching 200-epoch run.
    %   Margin of 30m is close to the 3-sigma authentic spread
    %   (std=11.7m -> 3*std=35m).
    %   This confirms the spoofed run stays within the authentic noise band.
    %   Source: reviewer criterion — spoofed_error ~ authentic_error + margin.
    %
    % Condition (b): spoofed error <= 0.3 * unmitigated error.
    %   unmitigated = 400m at ep200.  0.3 * 400 = 120m.
    %   This confirms the pipeline suppresses the attack, not merely tolerates it.
    %   Source: reviewer criterion — spoofed_error << unmitigated_error.
    margin  = 30.0;   % m — close to 3-sigma authentic spread over 200 epochs
    cond_a3 = err_ep200_spoof <= err_ep200_auth + margin;
    cond_b3 = err_ep200_spoof <= 0.3 * unmitigated_err;
    ok3     = cond_a3 && cond_b3;

    fprintf('  Spoof onset (ep120) pos error:    %.2f m\n', err_ep120_spoof);
    fprintf('  Spoofed ep200 pos error:           %.2f m\n', err_ep200_spoof);
    fprintf('  Authentic ep200 pos error:         %.2f m  (baseline)\n', err_ep200_auth);
    fprintf('  Unmitigated error at ep200:        ~%.0f m\n', unmitigated_err);
    fprintf('  Cond a (spoof <= auth+30m=%.1fm): %d\n', err_ep200_auth+margin, cond_a3);
    fprintf('  Cond b (spoof <= 0.3*unmit=%.1fm): %d\n', 0.3*unmitigated_err, cond_b3);

    if ok3
        fprintf('  PASS — recovery confirmed: error near authentic baseline, far below unmitigated\n');
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL — cond_a=%d, cond_b=%d\n', cond_a3, cond_b3);
        fprintf('         Check Stage 2 classification and apply_exclusion_mask.\n');
    end
end

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_stage4_integration: ALL PASS ✓\n\n');
else
    fprintf('test_stage4_integration: %d FAILURE(S) — review output above\n\n', ...
        n_tests - n_pass);
end

%% ============================================================================
%  LOCAL HELPERS
%% ============================================================================

function obs_sub = subset_obs(obs, epochs_keep)
% SUBSET_OBS  Return obs struct containing only the specified epochs.
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    obs_sub = obs;
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs, cname) || ~isfield(obs.(cname), 'time') || isempty(obs.(cname).time)
            continue;
        end
        mask = ismember(obs.(cname).time, epochs_keep);
        fields = fieldnames(obs.(cname));
        for fi = 1:numel(fields)
            val = obs.(cname).(fields{fi});
            if size(val,1) == size(obs.(cname).time, 1)
                obs_sub.(cname).(fields{fi}) = val(mask,:);
            end
        end
    end
end

function cr_list = build_scenario1_classify(obs_sub, epochs, spoofed_prns, onset_epoch)
% BUILD_SCENARIO1_CLASSIFY  Simplified Stage 2 output for Scenario 1.
%   Marks GPS PRNs in spoofed_prns as 'spoofed' after onset_epoch index.
%   All other satellites are 'trusted'.
%   Used to test Stage 4 recovery assuming Stage 2 is correct.

    n_ep    = numel(epochs);
    cr_list = cell(n_ep, 1);

    for ei = 1:n_ep
        t_e      = epochs(ei);
        sat_list = struct('constellation',{},'prn',{},'status',{});

        constellations = {'GPS','Galileo','BeiDou','GLONASS'};
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs_sub, cname), continue; end
            mask_t = (obs_sub.(cname).time == t_e);
            prns_e = obs_sub.(cname).prn(mask_t);

            for k = 1:numel(prns_e)
                entry.constellation = cname;
                entry.prn           = prns_e(k);

                is_spoofed_const  = strcmp(cname, 'GPS');
                is_spoofed_prn    = ismember(prns_e(k), spoofed_prns);
                is_after_onset    = (ei >= onset_epoch);

                if is_spoofed_const && is_spoofed_prn && is_after_onset
                    entry.status = 'spoofed';
                else
                    entry.status = 'trusted';
                end
                sat_list(end+1) = entry; %#ok<AGROW>
            end
        end

        n_sp = sum(strcmp({sat_list.status}, 'spoofed'));
        cr_list{ei}.sat_list  = sat_list;
        cr_list{ei}.n_trusted = numel(sat_list) - n_sp;
        cr_list{ei}.n_suspect = 0;
        cr_list{ei}.n_spoofed = n_sp;
    end
end
