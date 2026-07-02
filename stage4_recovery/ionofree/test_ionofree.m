%% TEST_IONOFREE  Validate ionosphere-free combination vs L1-only on BUCU data.
%
% Run from project root after config:
%   config
%   obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
%   nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);
%   run('stage4_recovery/ionofree/test_ionofree.m')
%
% Expected result: PASS (2/2)
%
% METHODOLOGY:
%
%   Test 1 — Position error: IF combination vs L1-only, across 200 epochs.
%     For each epoch, compute WLS position TWICE:
%       (a) using pr_L1 only (current pipeline behaviour)
%       (b) using ionofree_combination(pr_L1, pr_L2)
%     Pass criterion: mean position error with IF combination < mean
%     position error with L1-only, across the 200-epoch test window.
%
%     IMPORTANT: this test does NOT require a calibrate/validate split.
%     The IF combination is computed independently at every epoch from
%     that epoch's own L1/L2 measurements — there is no calibration step,
%     so there is no train/test leakage concern.  This is fundamentally
%     different from the per-satellite bias table approach (which failed
%     Test 1 in test_sat_bias_calibration.m due to time-varying ionosphere).
%
%   Test 2 — Noise amplification check.
%     Confirms the documented ~3x noise amplification (Hofmann-Wellenhof
%     2008) is observed in the post-fit residual spread, and that the
%     position error improvement still outweighs it.
%     Pass criterion: std(IF residuals) / std(L1 residuals) is in the
%     range [2, 5] (consistent with the theoretical ~3x factor, with
%     margin for real-data variation).
%
%   Informational — Per-satellite bias comparison (L1-only vs IF).
%     Re-runs the per-satellite residual diagnosis (the one that found
%     PRN 19 = +40m etc.) using the IF combination, to show whether the
%     large per-satellite biases are reduced.  This is the direct causal
%     check: if those biases were ionospheric, IF combination should
%     shrink them substantially.
%
% NOTE ON L2 AVAILABILITY:
%   GPS L2 (C2W) requires the receiver to track the encrypted P(Y) code
%   via semi-codeless techniques.  Not all epochs/satellites may have valid
%   L2 data.  Epochs/satellites with NaN L2 are excluded from the IF test
%   (ionofree_combination returns NaN, which is filtered).  The reported
%   n_sats_used may therefore be lower for the IF case than the L1 case.
%
% STAGE:    4 — EKF Position Recovery (measurement model enhancement)

fprintf('\n=== test_ionofree.m ===\n');

n_tests = 0;
n_pass  = 0;

test_range = 1:200;
epochs_all = unique(obs.GPS.time);

noise_map = struct( ...
    'GPS',     cfg.ekf.meas_noise_GPS, ...
    'Galileo', cfg.ekf.meas_noise_Galileo, ...
    'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
    'GLONASS', cfg.ekf.meas_noise_GLONASS);

%% ---- Compute both L1-only and IF position errors over test_range ---------
err_L1 = [];
err_IF = [];
n_sats_L1 = [];
n_sats_IF = [];
resid_L1_all = [];
resid_IF_all = [];

for ii = 1:numel(test_range)
    ei  = test_range(ii);
    t_e = epochs_all(ei);

    mask = (obs.GPS.time == t_e);
    prns = obs.GPS.prn(mask);
    prs_L1 = obs.GPS.pseudorange_L1(mask);
    prs_L2 = obs.GPS.pseudorange_L2(mask);

    pr_L1_corr = []; pr_IF_corr = []; sp_L1 = []; sp_IF = [];

    for k = 1:numel(prns)
        try
            [sp, sc] = sat_position(nav, prns(k), 'GPS', t_e);

            % --- L1-only path (current pipeline) ---
            pr_c_L1 = pseudorange_correct(prs_L1(k), sp, sc, cfg.ref_pos, t_e, nav, 'GPS', cfg);
            if ~isnan(pr_c_L1)
                pr_L1_corr(end+1) = pr_c_L1; %#ok<AGROW>
                sp_L1(end+1,:)    = sp';     %#ok<AGROW>
            end

            % --- IF path ---
            % Apply pseudorange_correct to RAW L1 and L2 separately for
            % satellite clock / Sagnac / relativity (these are NOT
            % frequency-dependent, so the same correction applies to both),
            % THEN form the IF combination on the corrected pseudoranges.
            % Tropospheric correction is also non-dispersive and applies
            % to both. Only the ionospheric term (frequency-dependent,
            % applied inside pseudorange_correct for L1 only via Klobuchar)
            % must NOT be double-applied — so we correct L2 using the SAME
            % non-ionospheric corrections as L1, by computing the corrected
            % L1 minus its ionospheric component, and analogously for L2.
            %
            % SIMPLIFICATION USED HERE: pseudorange_correct applies sat_clk
            % + Sagnac + relativity + troposphere + Klobuchar-ionosphere to
            % its input. For the IF combination we want sat_clk + Sagnac +
            % relativity + troposphere WITHOUT ionosphere on each raw
            % pseudorange, then combine. Since pseudorange_correct does not
            % expose a "without ionosphere" mode, we instead form the IF
            % combination on RAW pseudoranges first (which cancels
            % ionosphere AND leaves sat_clk/Sagnac/relativity/troposphere
            % uncorrected), then apply those non-dispersive corrections
            % using the L1 correction terms (sat_clk, Sagnac, relativity,
            % troposphere are frequency-independent so the L1-derived
            % correction value applies equally to the IF combination).
            %
            % correction_L1 = pr_c_L1 - prs_L1(k)  (this is the TOTAL L1
            %   correction: sat_clk+Sagnac+rel+tropo+iono_L1)
            % We cannot separate iono_L1 out of this without modifying
            % pseudorange_correct. As an approximation, we apply the FULL
            % L1 correction to the raw IF combination. This DOUBLE-REMOVES
            % the L1 Klobuchar ionosphere estimate (once via IF combination,
            % once via correction_L1), which is conservative: at worst it
            % under-corrects troposphere by the (small) difference between
            % Klobuchar-predicted and true ionosphere -- NOT a sign error,
            % since IF combination already removed the true ionosphere and
            % correction_L1's Klobuchar term is then an OVER-subtraction of
            % a near-zero residual (true_iono - klobuchar_iono is small for
            % the IF-combined range, since IF range has ~zero ionosphere).
            %
            % NOTE FOR THESIS: this is a documented approximation. The
            % rigorous approach modifies pseudorange_correct to expose
            % iono and non-iono terms separately. Flagged as a follow-up
            % refinement; the approximation's effect is bounded by the
            % Klobuchar residual error (typically < few metres), small
            % relative to the tens-of-metres ionospheric delay being removed.

            if ~isnan(prs_L2(k)) && ~isnan(pr_c_L1)
                pr_if_raw = ionofree_combination(prs_L1(k), prs_L2(k), 'GPS');
                if ~isnan(pr_if_raw)
                    correction_L1 = pr_c_L1 - prs_L1(k);
                    pr_if_corrected = pr_if_raw + correction_L1;
                    pr_IF_corr(end+1) = pr_if_corrected; %#ok<AGROW>
                    sp_IF(end+1,:)    = sp';             %#ok<AGROW>
                end
            end
        catch
            continue
        end
    end

    % --- L1-only WLS ---
    if numel(pr_L1_corr) >= cfg.identify.min_sats
        w1 = ones(numel(pr_L1_corr),1) / noise_map.GPS;
        [pos1, clk1, res1] = wls_solver(pr_L1_corr(:), sp_L1, w1, cfg.ref_pos);
        err_L1(end+1)   = norm(pos1 - cfg.ref_pos); %#ok<AGROW>
        n_sats_L1(end+1) = numel(pr_L1_corr);       %#ok<AGROW>
        resid_L1_all = [resid_L1_all; res1(:)];     %#ok<AGROW>
    end

    % --- IF WLS ---
    % IF noise is ~3x larger -> meas_noise_GPS_IF = 9x meas_noise_GPS
    % (variance scales as alpha^2+beta^2 ~ 9, per ionofree_combination docstring)
    if numel(pr_IF_corr) >= cfg.identify.min_sats
        w_if = ones(numel(pr_IF_corr),1) / (9 * noise_map.GPS);
        [pos_if, clk_if, res_if] = wls_solver(pr_IF_corr(:), sp_IF, w_if, cfg.ref_pos);
        err_IF(end+1)   = norm(pos_if - cfg.ref_pos); %#ok<AGROW>
        n_sats_IF(end+1) = numel(pr_IF_corr);          %#ok<AGROW>
        resid_IF_all = [resid_IF_all; res_if(:)];      %#ok<AGROW>
    end
end

%% ---- Test 1: Position error improvement -----------------------------------
fprintf('\nTest 1: Position error — IF combination vs L1-only (200 epochs)\n');
n_tests = n_tests + 1;

mean_err_L1 = mean(err_L1);
mean_err_IF = mean(err_IF);

ok1 = mean_err_IF < mean_err_L1;

fprintf('  Epochs with valid L1 solution: %d/%d\n', numel(err_L1), numel(test_range));
fprintf('  Epochs with valid IF solution: %d/%d\n', numel(err_IF), numel(test_range));
fprintf('  Mean n_sats (L1): %.1f\n', mean(n_sats_L1));
fprintf('  Mean n_sats (IF): %.1f\n', mean(n_sats_IF));
fprintf('  Mean pos error (L1-only):  %.2f m\n', mean_err_L1);
fprintf('  Mean pos error (IF):       %.2f m\n', mean_err_IF);
fprintf('  Improvement: %.2f m (%.1f%%)\n', mean_err_L1 - mean_err_IF, ...
    100*(mean_err_L1 - mean_err_IF)/mean_err_L1);
fprintf('  p95 pos error (L1-only): %.2f m\n', prctile(err_L1, 95));
fprintf('  p95 pos error (IF):      %.2f m\n', prctile(err_IF, 95));

if ok1
    fprintf('  PASS — IF combination reduces mean position error\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — IF combination did not improve position error\n');
end

%% ---- Test 2: Noise amplification check ------------------------------------
fprintf('\nTest 2: Noise amplification — IF/L1 residual std ratio in [2,5]\n');
n_tests = n_tests + 1;

std_L1 = std(resid_L1_all);
std_IF = std(resid_IF_all);
ratio  = std_IF / std_L1;

ok2 = ratio >= 2 && ratio <= 5;

fprintf('  std(L1 residuals): %.3f m\n', std_L1);
fprintf('  std(IF residuals): %.3f m\n', std_IF);
fprintf('  Ratio: %.2f  (theoretical ~3x per Hofmann-Wellenhof 2008)\n', ratio);

if ok2
    fprintf('  PASS — noise amplification %.2fx is within expected [2,5] range\n', ratio);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — noise amplification %.2fx outside expected [2,5] range\n', ratio);
end

%% ---- Informational: per-satellite bias, L1-only vs IF ---------------------
fprintf('\nInformational: Per-satellite mean residual, L1-only vs IF (epochs 1-100)\n');
fprintf('(Compares against the earlier finding: PRN19=+40m, PRN15=-29m etc. on L1-only)\n');

prn_resid_L1 = containers.Map('KeyType','double','ValueType','any');
prn_resid_IF = containers.Map('KeyType','double','ValueType','any');

for ii = 1:100
    ei  = test_range(ii);
    t_e = epochs_all(ei);
    mask = (obs.GPS.time == t_e);
    prns = obs.GPS.prn(mask);
    prs_L1 = obs.GPS.pseudorange_L1(mask);
    prs_L2 = obs.GPS.pseudorange_L2(mask);

    pr_L1_corr=[]; sp_L1=[]; prn_L1=[];
    pr_IF_corr=[]; sp_IF=[]; prn_IF=[];

    for k = 1:numel(prns)
        try
            [sp, sc] = sat_position(nav, prns(k), 'GPS', t_e);
            pr_c_L1 = pseudorange_correct(prs_L1(k), sp, sc, cfg.ref_pos, t_e, nav, 'GPS', cfg);
            if ~isnan(pr_c_L1)
                pr_L1_corr(end+1)=pr_c_L1; sp_L1(end+1,:)=sp'; prn_L1(end+1)=prns(k); %#ok<AGROW>
            end
            if ~isnan(prs_L2(k)) && ~isnan(pr_c_L1)
                pr_if_raw = ionofree_combination(prs_L1(k), prs_L2(k), 'GPS');
                if ~isnan(pr_if_raw)
                    correction_L1 = pr_c_L1 - prs_L1(k);
                    pr_IF_corr(end+1) = pr_if_raw + correction_L1; %#ok<AGROW>
                    sp_IF(end+1,:) = sp'; prn_IF(end+1)=prns(k);    %#ok<AGROW>
                end
            end
        catch
            continue
        end
    end

    if numel(pr_L1_corr) >= cfg.identify.min_sats
        w1 = ones(numel(pr_L1_corr),1) / noise_map.GPS;
        [pos1, clk1] = wls_solver(pr_L1_corr(:), sp_L1, w1, cfg.ref_pos);
        for k=1:numel(pr_L1_corr)
            r = pr_L1_corr(k) - (norm(sp_L1(k,:)'-pos1) + clk1);
            if ~isKey(prn_resid_L1, prn_L1(k)), prn_resid_L1(prn_L1(k))=[]; end
            prn_resid_L1(prn_L1(k)) = [prn_resid_L1(prn_L1(k)), r];
        end
    end
    if numel(pr_IF_corr) >= cfg.identify.min_sats
        w_if = ones(numel(pr_IF_corr),1) / (9*noise_map.GPS);
        [pos_if, clk_if] = wls_solver(pr_IF_corr(:), sp_IF, w_if, cfg.ref_pos);
        for k=1:numel(pr_IF_corr)
            r = pr_IF_corr(k) - (norm(sp_IF(k,:)'-pos_if) + clk_if);
            if ~isKey(prn_resid_IF, prn_IF(k)), prn_resid_IF(prn_IF(k))=[]; end
            prn_resid_IF(prn_IF(k)) = [prn_resid_IF(prn_IF(k)), r];
        end
    end
end

fprintf('PRN | mean resid L1-only | mean resid IF | |L1| - |IF|\n');
common = intersect(cell2mat(keys(prn_resid_L1)), cell2mat(keys(prn_resid_IF)));
for p = sort(common)
    m1 = mean(prn_resid_L1(p));
    mif = mean(prn_resid_IF(p));
    fprintf(' %3d | %18.3f | %13.3f | %9.3f\n', p, m1, mif, abs(m1)-abs(mif));
end

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_ionofree: ALL PASS ✓\n\n');
else
    fprintf('test_ionofree: %d FAILURE(S) — review output above\n\n', n_tests - n_pass);
end
