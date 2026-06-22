function result = detect_pseudorange(obs, nav, cfg)
% detect_pseudorange  Detect spoofing via pseudorange residual monitoring.
%
%   result = detect_pseudorange(obs, nav, cfg)
%
%   THEORY:
%     Under normal conditions, the difference between measured pseudorange
%     and geometrically expected pseudorange (residual) should be small —
%     dominated by atmospheric noise, multipath, and receiver noise.
%     Typical residuals are 1-10 metres. A spoofing attack injects an
%     additional offset that grows over time (drag-off model), causing
%     residuals to exceed the detection threshold and grow monotonically.
%
%     For each satellite, the expected pseudorange is:
%       rho_expected = geometric_range + receiver_clock_bias + satellite_clock
%
%     The receiver clock bias is estimated epoch-by-epoch from the median
%     residual of all non-spoofed satellites, making the detector robust
%     to receiver clock drift.
%
%   INPUT:
%     obs - observation struct from rinex_read_obs()
%     nav - navigation struct from rinex_read_nav()
%     cfg - configuration struct from config.m
%
%   OUTPUT:
%     result - struct with fields:
%       .flag          [n_epochs x 1] logical — true = anomaly detected
%       .residuals     [n_epochs x n_sats] per-satellite residuals (m)
%       .max_residual  [n_epochs x 1] maximum absolute residual per epoch
%       .confidence    [n_epochs x 1] detection confidence 0-1
%       .n_epochs      scalar
%       .threshold     threshold used

%% ── PARAMETERS ───────────────────────────────────────────────────────────
CONST      = 'GPS';
RES_THRESH = cfg.detect.residual_threshold;
MIN_SATS   = 4;

%% ── INITIALISE OUTPUT ────────────────────────────────────────────────────
n_epochs = obs.n_epochs;
epochs   = obs.epochs;
gps_prns = unique(obs.GPS.prn);
n_gps    = length(gps_prns);

result.flag         = false(n_epochs, 1);
result.residuals    = nan(n_epochs, n_gps);
result.max_residual = nan(n_epochs, 1);
result.confidence   = zeros(n_epochs, 1);
result.n_epochs     = n_epochs;
result.threshold    = RES_THRESH;
result.prns         = gps_prns;

if cfg.verbose
    fprintf('      Running pseudorange residual detector...\n');
    fprintf('      Threshold: %.1f m | Min sats: %d\n', RES_THRESH, MIN_SATS);
end

%% ── COMPUTE INITIAL POSITION VIA WLS ────────────────────────────────────
% Use mid-day epoch where satellites are well distributed
% Gives better reference position than approximate city centre
if cfg.verbose
    fprintf('      Computing reference position via WLS...\n');
end

rec_pos    = cfg.ref_pos;
best_epoch = min(1441, n_epochs);
t_ref      = epochs(best_epoch);

% Preallocate — fixes variable-size-on-every-iteration warning
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
    n_valid_wls                  = n_valid_wls + 1;
    sat_pos_mat(n_valid_wls, :)  = pos_k';
    pr_vec(n_valid_wls)          = pr_c;
    w_vec(n_valid_wls)           = 1 / cfg.ekf.meas_noise_GPS;
end

if n_valid_wls >= 4
    % Run WLS solver iteratively using fixed corrected pseudoranges
    % Do not recompute atmospheric corrections per iteration —
    % corrections are already applied using cfg.ref_pos approximation
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
        [lat_d, lon_d, alt_d] = coord_convert('ecef2lla_deg', rec_pos);
        if cfg.verbose
            fprintf('      Reference pos: Lat=%.4f Lon=%.4f Alt=%.1fm\n', ...
                lat_d, lon_d, alt_d);
        end
    else
        if cfg.verbose
            fprintf('      WLS diverged — using cfg.ref_pos\n');
        end
    end
end
%% ── PROCESS EACH EPOCH ───────────────────────────────────────────────────
for e = 1:n_epochs
    t_e = epochs(e);

    raw_residuals = nan(n_gps, 1);
    n_valid       = 0;

    for p = 1:n_gps
        prn_k = gps_prns(p);

        mask = (obs.GPS.time == t_e) & (obs.GPS.prn == prn_k);
        idx  = find(mask, 1, 'first');
        if isempty(idx), continue; end

        pr_raw = obs.GPS.pseudorange_L1(idx);
        if isnan(pr_raw), continue; end

        [sat_pos, sat_clk] = sat_position(nav, prn_k, CONST, t_e);
        if any(isnan(sat_pos)), continue; end

        pr_corr = pseudorange_correct(pr_raw, sat_pos, sat_clk, ...
                                      rec_pos, t_e, nav, CONST, cfg);
        if isnan(pr_corr), continue; end

        geo_range        = norm(sat_pos - rec_pos);
        raw_residuals(p) = pr_corr - geo_range;
        n_valid          = n_valid + 1;
    end

    if n_valid < MIN_SATS, continue; end

    % Median clock bias removal — robust to spoofed outliers
    clock_bias_est   = median(raw_residuals, 'omitnan');
    residuals_no_clk = raw_residuals - clock_bias_est;

    result.residuals(e, :)  = residuals_no_clk';
    abs_res                 = abs(residuals_no_clk);
    result.max_residual(e)  = max(abs_res, [], 'omitnan');

    n_exceeded     = sum(abs_res > RES_THRESH & ~isnan(abs_res));
    result.flag(e) = n_exceeded >= 1;

    if result.flag(e)
        result.confidence(e) = min(result.max_residual(e) / (5 * RES_THRESH), 1.0);
    end

end

%% ── SUMMARY ──────────────────────────────────────────────────────────────
n_flagged = sum(result.flag);
if cfg.verbose
    fprintf('      Flagged epochs: %d / %d (%.1f%%)\n', ...
        n_flagged, n_epochs, 100*n_flagged/n_epochs);
    fprintf('      Max residual observed: %.1f m\n', ...
        max(result.max_residual, [], 'omitnan'));
end

end

%% tested on cmd using
% clear functions
% config
% result_pr_auth  = detect_pseudorange(obs, nav, cfg);
% result_pr_spoof = detect_pseudorange(obs_spoofed_s1, nav, cfg);
% 
% fprintf('\nAUTHENTIC — flagged: %d/%d (%.1f%%)\n', ...
%     sum(result_pr_auth.flag), result_pr_auth.n_epochs, ...
%     100*sum(result_pr_auth.flag)/result_pr_auth.n_epochs);
% fprintf('SPOOFED   — flagged: %d/%d (%.1f%%)\n', ...
%     sum(result_pr_spoof.flag), result_pr_spoof.n_epochs, ...
%     100*sum(result_pr_spoof.flag)/result_pr_spoof.n_epochs);
% 
% first_flag = find(result_pr_spoof.flag, 1, 'first');
% fprintf('\nFirst flagged epoch: %d (attack at epoch 120)\n', first_flag);
% fprintf('Max authentic residual: %.1f m\n', ...
%     max(result_pr_auth.max_residual, [], 'omitnan'));
% fprintf('Max spoofed residual:   %.1f m\n', ...
%     max(result_pr_spoof.max_residual, [], 'omitnan'));