function result = detect_clock(obs, nav, cfg)
% detect_clock  Detect spoofing via receiver clock bias/drift consistency.
%
%   result = detect_clock(obs, nav, cfg)
%
%   THEORY:
%     A GNSS receiver's clock bias and drift are governed by its internal
%     oscillator — a physical device with stable, predictable behaviour.
%     Under normal conditions, the clock drift (rate of change of bias)
%     is consistent with the measured bias over time:
%
%       bias(t+1) ≈ bias(t) + drift(t) * dt
%
%     A spoofing attack corrupts pseudoranges and Doppler independently,
%     breaking this physical correlation. The clock bias jumps in a way
%     that is inconsistent with the integrated drift — a detectable anomaly.
%
%     Reference: Xu et al. (2023) "GNSS Spoofing Detection via
%     Self-Consistent Verification of Receiver Clock State"
%     PMC12845604
%
%   INPUT:
%     obs - observation struct from rinex_read_obs()
%     nav - navigation struct from rinex_read_nav()
%     cfg - configuration struct from config.m
%
%   OUTPUT:
%     result - struct with fields:
%       .flag         [n_epochs x 1] logical — true = anomaly detected
%       .clock_bias   [n_epochs x 1] estimated clock bias (metres)
%       .clock_drift  [n_epochs x 1] estimated clock drift (metres/epoch)
%       .consistency  [n_epochs x 1] bias vs integrated drift difference (m)
%       .confidence   [n_epochs x 1] detection confidence 0-1
%       .n_epochs     scalar
%       .threshold    threshold used

%% ── PARAMETERS ───────────────────────────────────────────────────────────
CONST           = 'GPS';
CLK_THRESH      = cfg.detect.clock_consistency_threshold;  % metres
MIN_SATS        = 4;
WIN_SIZE        = 5;   % epochs for drift estimation

%% ── INITIALISE OUTPUT ────────────────────────────────────────────────────
n_epochs = obs.n_epochs;
epochs   = obs.epochs;
gps_prns = unique(obs.GPS.prn);
n_gps    = length(gps_prns);

result.flag        = false(n_epochs, 1);
result.clock_bias  = nan(n_epochs, 1);
result.clock_drift = nan(n_epochs, 1);
result.consistency = nan(n_epochs, 1);
result.confidence  = zeros(n_epochs, 1);
result.n_epochs    = n_epochs;
result.threshold   = CLK_THRESH;

if cfg.verbose
    fprintf('      Running clock bias/drift consistency detector...\n');
    fprintf('      Threshold: %.1f m\n', CLK_THRESH);
end

%% ── COMPUTE REFERENCE POSITION VIA WLS ──────────────────────────────────
rec_pos    = cfg.ref_pos;
best_epoch = min(1441, n_epochs);
t_ref      = epochs(best_epoch);

sat_pos_mat = zeros(n_gps, 3);
pr_vec      = zeros(n_gps, 1);
w_vec       = zeros(n_gps, 1);
n_valid_wls = 0;

for p = 1:n_gps
    prn_k = gps_prns(p);
    mask  = (obs.GPS.time == t_ref) & (obs.GPS.prn == prn_k);
    idx   = find(mask, 1, 'first');
    if isempty(idx), continue; end
    pr_k = obs.GPS.pseudorange_L1(idx);
    if isnan(pr_k), continue; end
    [pos_k, clk_k] = sat_position(nav, prn_k, 'GPS', t_ref);
    if any(isnan(pos_k)), continue; end
    pr_c = pseudorange_correct(pr_k, pos_k, clk_k, rec_pos, t_ref, nav, 'GPS', cfg);
    if isnan(pr_c), continue; end
    n_valid_wls                 = n_valid_wls + 1;
    sat_pos_mat(n_valid_wls,:)  = pos_k';
    pr_vec(n_valid_wls)         = pr_c;
    w_vec(n_valid_wls)          = 1/cfg.ekf.meas_noise_GPS;
end

if n_valid_wls >= 4
    pos_iter = rec_pos;
    for wls_iter = 1:10
        [pos_new, ~, ~, ~, ~] = wls_solver(pr_vec(1:n_valid_wls), ...
                                            sat_pos_mat(1:n_valid_wls,:), ...
                                            w_vec(1:n_valid_wls), pos_iter);
        if any(isnan(pos_new)), break; end
        if norm(pos_new - pos_iter) < 0.01, break; end
        pos_iter = pos_new;
    end
    if ~any(isnan(pos_iter)) && norm(pos_iter) > 6000000
        rec_pos = pos_iter;
    end
end

%% ── ESTIMATE CLOCK BIAS PER EPOCH ───────────────────────────────────────
% Clock bias = median pseudorange residual across all satellites
for e = 1:n_epochs
    t_e = epochs(e);
    residuals_e = nan(n_gps, 1);
    n_valid = 0;

    for p = 1:n_gps
        prn_k = gps_prns(p);
        mask  = (obs.GPS.time == t_e) & (obs.GPS.prn == prn_k);
        idx   = find(mask, 1, 'first');
        if isempty(idx), continue; end

        pr_raw = obs.GPS.pseudorange_L1(idx);
        if isnan(pr_raw), continue; end

        [sat_pos, sat_clk] = sat_position(nav, prn_k, CONST, t_e);
        if any(isnan(sat_pos)), continue; end

        pr_corr = pseudorange_correct(pr_raw, sat_pos, sat_clk, ...
                                      rec_pos, t_e, nav, CONST, cfg);
        if isnan(pr_corr), continue; end

        geo_range       = norm(sat_pos - rec_pos);
        residuals_e(p)  = pr_corr - geo_range;
        n_valid         = n_valid + 1;
    end

    if n_valid >= MIN_SATS
        % Clock bias = median residual (robust to spoofed outliers)
        result.clock_bias(e) = median(residuals_e, 'omitnan');
    end
end

%% ── ESTIMATE CLOCK DRIFT AND TEST CONSISTENCY ────────────────────────────
% Epoch interval in seconds
dt = 30;  % BUCU station logs at 30-second intervals

for e = (WIN_SIZE + 1):n_epochs
    % Estimate drift from slope of bias over window
    window_bias = result.clock_bias(e-WIN_SIZE:e-1);
    valid_win   = ~isnan(window_bias);

    if sum(valid_win) < 3, continue; end

    % Linear regression on window to get drift estimate
    t_win  = (1:WIN_SIZE)' * dt;
    t_win  = t_win(valid_win);
    b_win  = window_bias(valid_win);

    % Drift = slope of linear fit (m/s)
    p_fit  = polyfit(t_win, b_win, 1);
    drift  = p_fit(1);  % metres per second

    result.clock_drift(e) = drift;

    % Predicted bias at current epoch
    bias_predicted = result.clock_bias(e-1) + drift * dt;

    % Actual bias at current epoch
    bias_actual = result.clock_bias(e);

    if isnan(bias_actual) || isnan(bias_predicted), continue; end

    % Consistency = difference between predicted and actual
    consistency = abs(bias_actual - bias_predicted);
    result.consistency(e) = consistency;

    % Flag if inconsistency exceeds threshold
    result.flag(e) = consistency > CLK_THRESH;

    % Confidence
    if result.flag(e)
        result.confidence(e) = min(consistency / (3 * CLK_THRESH), 1.0);
    end
end

%% ── SUMMARY ──────────────────────────────────────────────────────────────
n_flagged = sum(result.flag);
if cfg.verbose
    fprintf('      Flagged epochs: %d / %d (%.1f%%)\n', ...
        n_flagged, n_epochs, 100*n_flagged/n_epochs);
    fprintf('      Max consistency error: %.2f m\n', ...
        max(result.consistency, [], 'omitnan'));
end

end

%% tested on cmd using;
% clear functions
% config
% result_clk_auth  = detect_clock(obs, nav, cfg);
% result_clk_spoof = detect_clock(obs_spoofed_s1, nav, cfg);
% 
% fprintf('\nAUTHENTIC — flagged: %d/%d (%.1f%%)\n', ...
%     sum(result_clk_auth.flag), result_clk_auth.n_epochs, ...
%     100*sum(result_clk_auth.flag)/result_clk_auth.n_epochs);
% fprintf('SPOOFED   — flagged: %d/%d (%.1f%%)\n', ...
%     sum(result_clk_spoof.flag), result_clk_spoof.n_epochs, ...
%     100*sum(result_clk_spoof.flag)/result_clk_spoof.n_epochs);
% 
% first_flag = find(result_clk_spoof.flag, 1, 'first');
% fprintf('\nFirst flagged epoch: %d (attack at epoch 120)\n', first_flag);
% fprintf('Max consistency — authentic: %.3f m\n', ...
%     max(result_clk_auth.consistency, [], 'omitnan'));
% fprintf('Max consistency — spoofed:   %.3f m\n', ...
%     max(result_clk_spoof.consistency, [], 'omitnan'));