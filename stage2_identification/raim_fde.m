function result = raim_fde(obs_epoch, nav, cfg, t)
% RAIM_FDE  Receiver Autonomous Integrity Monitoring — Fault Detection & Exclusion
%
% Iterative satellite exclusion loop for identifying spoofed/faulty satellites.
% Uses wls_solver and chi_squared_test as building blocks.
%
% DESIGN NOTE (from chi_squared_test validation, epoch 500):
%   The chi-squared test alone CANNOT detect sophisticated multi-satellite
%   spoofing because the WLS solver absorbs the spoofing offset into its
%   position/clock estimate, making post-fit residuals internally consistent.
%   raim_fde therefore provides a CANDIDATE LIST, not a definitive verdict.
%   inter_constellation.m provides the definitive identification step.
%
% ALGORITHM:
%   1. Compute full WLS solution with all N satellites → chi-squared test
%   2. If test passes → return all satellites as trusted (no fault detected)
%   3. If test fails → exclusion loop:
%      a. For each satellite i, exclude it and recompute WLS
%      b. Record chi-squared statistic of the reduced solution
%      c. Satellite whose removal MOST REDUCES the statistic is flagged
%   4. Remove flagged satellite, repeat until test passes or min_sats reached
%
% INPUTS:
%   obs_epoch  struct  — single-epoch observables (all constellations)
%                        fields: .prn, .constellation, .pseudorange, .cn0
%   nav        struct  — navigation/ephemeris data (output of rinex_read_nav)
%   cfg        struct  — configuration (output of config.m)
%   t          datetime — current epoch timestamp (UTC)
%
% OUTPUT:
%   result     struct with fields:
%     .trusted_sats     [Nx1 cell]  — {constellation, prn} pairs declared healthy
%     .spoofed_sats     [Mx1 cell]  — {constellation, prn} pairs excluded by FDE
%     .fault_detected   logical     — true if chi-squared failed on full set
%     .final_chi2_stat  double      — chi-squared statistic of final solution
%     .final_chi2_thresh double     — threshold used
%     .final_pos_ecef   [3x1]       — ECEF position from trusted sats
%     .final_clk_bias   double      — receiver clock bias (m) from trusted sats
%     .n_trusted        int         — number of trusted satellites
%     .n_excluded       int         — number of satellites excluded by FDE
%     .iterations       int         — number of FDE iterations performed
%     .exclusion_log    struct array — per-iteration exclusion records

% -------------------------------------------------------------------------
% 0. Input validation
% -------------------------------------------------------------------------
if nargin < 4
    error('raim_fde: requires obs_epoch, nav, cfg, t');
end

n_obs = length(obs_epoch.prn);
if n_obs < cfg.identify.min_sats
    warning('raim_fde: only %d satellites — below minimum %d. Returning empty.', ...
        n_obs, cfg.identify.min_sats);
    result = empty_result(obs_epoch);
    return;
end

% -------------------------------------------------------------------------
% 1. Build measurement matrix from obs_epoch
%    Each row: [prn, constellation_id, pseudorange, cn0]
% -------------------------------------------------------------------------
sats = build_sat_list(obs_epoch, nav, cfg, t);

% Remove satellites for which we could not compute a valid position
valid_mask = [sats.valid];
sats = sats(valid_mask);
n_valid = length(sats);

if n_valid < cfg.identify.min_sats
    warning('raim_fde: only %d valid satellite positions — below minimum %d.', ...
        n_valid, cfg.identify.min_sats);
    result = empty_result(obs_epoch);
    return;
end

% -------------------------------------------------------------------------
% 2. Initial full-set WLS solution + chi-squared test
% -------------------------------------------------------------------------
[pos0, clk0, residuals0, weights0] = run_wls(sats, cfg);
chi2_result0 = chi_squared_test(residuals0, weights0, 4, cfg);

result.fault_detected   = ~chi2_result0.passed;
result.final_chi2_stat  = chi2_result0.test_stat;
result.final_chi2_thresh = chi2_result0.threshold;
result.iterations       = 0;
result.exclusion_log    = struct([]);

if chi2_result0.passed
    % No fault detected — all valid satellites are trusted
    result.trusted_sats   = extract_sat_ids(sats);
    result.spoofed_sats   = {};
    result.final_pos_ecef = pos0;
    result.final_clk_bias = clk0;
    result.n_trusted      = n_valid;
    result.n_excluded     = 0;
    return;
end

% -------------------------------------------------------------------------
% 3. FDE exclusion loop
% -------------------------------------------------------------------------
% Working satellite set — starts full, satellites removed iteratively
active_sats   = sats;
excluded_sats = {};   % accumulates excluded {constellation, prn} pairs

max_iterations = n_valid - cfg.identify.min_sats;   % can't go below min_sats

for iter = 1:max_iterations

    n_active = length(active_sats);

    if n_active < cfg.identify.min_sats + 1
        % Cannot exclude any more without going below minimum
        break;
    end

    % --- Try excluding each satellite and record chi-squared statistic ---
    chi2_stats = nan(n_active, 1);

    for i = 1:n_active
        subset = active_sats([1:i-1, i+1:end]);   % all except satellite i
        if length(subset) < cfg.identify.min_sats
            continue;   % skip: would drop below minimum
        end
        [~, ~, res_i, weights_i] = run_wls(subset, cfg);
        chi2_i = chi_squared_test(res_i, weights_i, 4, cfg);
        chi2_stats(i) = chi2_i.test_stat;
    end

    % --- Find satellite whose removal most reduces the test statistic ---
    [min_stat, best_idx] = min(chi2_stats);

    if isnan(min_stat)
        % All subsets were too small
        break;
    end

    % Log this exclusion
    log_entry.iteration       = iter;
    log_entry.excluded_prn    = active_sats(best_idx).prn;
    log_entry.excluded_const  = active_sats(best_idx).constellation;
    log_entry.chi2_before     = chi2_result0.test_stat;   % full set statistic
    log_entry.chi2_after      = min_stat;
    log_entry.threshold       = chi2_result0.threshold;

    if isempty(result.exclusion_log)
        result.exclusion_log = log_entry;
    else
        result.exclusion_log(end+1) = log_entry;
    end

    % Record excluded satellite
    excluded_sats{end+1} = struct( ...
        'prn',           active_sats(best_idx).prn, ...
        'constellation', active_sats(best_idx).constellation ...
    ); %#ok<AGROW>

    % Remove it from active set
    active_sats(best_idx) = [];
    result.iterations = iter;

    % --- Re-test with reduced set ---
    [pos_iter, clk_iter, res_iter, weights_iter] = run_wls(active_sats, cfg);
    chi2_iter = chi_squared_test(res_iter, weights_iter, 4, cfg);

    result.final_chi2_stat   = chi2_iter.test_stat;
    result.final_chi2_thresh = chi2_iter.threshold;
    result.final_pos_ecef    = pos_iter;
    result.final_clk_bias    = clk_iter;

    if chi2_iter.passed
        % Test passes — stop excluding
        break;
    end

    % Update reference statistic for next iteration log
    chi2_result0 = chi2_iter;

end

% -------------------------------------------------------------------------
% 4. Assemble final result
% -------------------------------------------------------------------------
result.trusted_sats   = extract_sat_ids(active_sats);
result.spoofed_sats   = excluded_sats;
result.n_trusted      = length(active_sats);
result.n_excluded     = length(excluded_sats);

% Guard: if we never set final_pos_ecef (e.g. loop never ran WLS), use full set
if ~isfield(result, 'final_pos_ecef') || isempty(result.final_pos_ecef)
    result.final_pos_ecef = pos0;
    result.final_clk_bias = clk0;
end

% -------------------------------------------------------------------------
% 5. Verbose logging
% -------------------------------------------------------------------------
if cfg.verbose
    fprintf('[RAIM-FDE] t=%s | fault_detected=%d | excluded=%d | trusted=%d\n', ...
        string(datetime(t, 'Format', 'HH:mm:ss')), result.fault_detected, result.n_excluded, result.n_trusted);
    if result.n_excluded > 0
        fprintf('[RAIM-FDE]   Chi2: %.2f → %.2f (thresh=%.2f)\n', ...
            result.exclusion_log(1).chi2_before, ...
            result.final_chi2_stat, ...
            result.final_chi2_thresh);
        for k = 1:length(excluded_sats)
            fprintf('[RAIM-FDE]   Excluded: %s PRN %d\n', ...
                excluded_sats{k}.constellation, excluded_sats{k}.prn);
        end
    end
end

end % main function


%% =========================================================================
%  LOCAL HELPER: build_sat_list
%  Computes satellite ECEF positions and corrected pseudoranges for all
%  satellites in obs_epoch. Returns struct array with one entry per satellite.
% =========================================================================
function sats = build_sat_list(obs_epoch, nav, cfg, t)
% Calls sat_position(nav, prn, constellation, t) and
% pseudorange_correct(pr_raw, sat_pos, sat_clk, rec_pos, t, nav, constellation, cfg)

n = length(obs_epoch.prn);
sats = struct( ...
    'prn',           cell(1,n), ...
    'constellation', cell(1,n), ...
    'pseudorange',   cell(1,n), ...
    'pos_ecef',      cell(1,n), ...
    'clk_corr',      cell(1,n), ...
    'weight',        cell(1,n), ...
    'valid',         cell(1,n)  ...
);

for i = 1:n
    prn   = obs_epoch.prn(i);
    const = obs_epoch.constellation{i};
    pr    = obs_epoch.pseudorange(i);

    sats(i).prn           = prn;
    sats(i).constellation = const;
    sats(i).pseudorange   = pr;
    sats(i).valid         = false;

    % NOTE: BeiDou PRN 33 (BDS-3 IGSO) was previously flagged as corrupted, but
    % residual analysis over 843 epochs (std 1.9 m, comparable to healthy PRN 24)
    % confirms it is healthy in this 17-May-2026 dataset. It is included.
    % IGSO ephemeris is genuinely less accurate than MEO and can degrade near
    % maneuvers; verified clean here. See thesis Ch4/Ch6.

    if isnan(pr) || pr <= 0
        continue;
    end

    try
        % Transmit-time corrected measurement. Returns sat_pos at TRANSMIT
        % time and sat_clk for traceability; geometry below is consistent
        % with the correction. rec_approx = cfg.ref_pos.
        [pr_corr, sat_pos, sat_clk] = corrected_pseudorange(pr, prn, const, ...
                                          t, cfg.ref_pos(:), nav, cfg);
        if isnan(pr_corr)
            continue;   % elevation-masked or invalid — skip
        end

        switch const
            case 'GPS';     sigma2 = cfg.ekf.meas_noise_GPS;
            case 'Galileo'; sigma2 = cfg.ekf.meas_noise_Galileo;
            case 'BeiDou';  sigma2 = cfg.ekf.meas_noise_BeiDou;
            case 'GLONASS'; sigma2 = cfg.ekf.meas_noise_GLONASS;
            otherwise;      sigma2 = 500.0;
        end

        sats(i).pos_ecef    = sat_pos;
        sats(i).clk_corr    = sat_clk;
        sats(i).pseudorange = pr_corr;
        sats(i).weight      = 1.0 / sigma2;
        sats(i).valid       = true;

    catch ME
        if cfg.verbose
            fprintf('[RAIM-FDE] Skipping %s PRN %d: %s\n', const, prn, ME.message);
        end
    end
end

end % build_sat_list


%% =========================================================================
%  LOCAL HELPER: get_ephemeris
%  Retrieves the closest valid ephemeris for a given satellite and epoch.
% =========================================================================
function eph = get_ephemeris(nav, constellation, prn, t)
% nav.(constellation) is a single struct with fields:
%   .prn  [N×1 double]   — PRN for each record
%   .toe  [N×1 datetime] — time of ephemeris for each record
%   .data [N×32 timetable] — all ephemeris parameters, one row per record

eph = [];

switch constellation
    case 'GPS';     field = 'GPS';
    case 'Galileo'; field = 'Galileo';
    case 'BeiDou';  field = 'BeiDou';
    case 'GLONASS'; field = 'GLONASS';
    otherwise;      return;
end

if ~isfield(nav, field) || isempty(nav.(field))
    return;
end

nav_const = nav.(field);   % single struct

% Find rows matching this PRN
prn_mask = (nav_const.prn == prn);
if ~any(prn_mask)
    return;
end

% Among matching rows, find the one whose toe is closest to t
toe_candidates = nav_const.toe(prn_mask);       % datetime vector
t_dt = datetime(t, 'TimeZone', 'UTC');
try
    toe_utc = datetime(toe_candidates, 'TimeZone', 'UTC');
catch
    toe_utc = toe_candidates;
end
dt_sec = abs(seconds(toe_utc - t_dt));
[~, best_local] = min(dt_sec);

% Map back to global row index
global_indices = find(prn_mask);
best_row = global_indices(best_local);

% Extract that row from the timetable as a struct
row = nav_const.data(best_row, :);
vars = row.Properties.VariableNames;

eph = struct();
eph.prn = prn;
eph.toe = nav_const.toe(best_row);

for v = 1:length(vars)
    val = row.(vars{v});
    if iscell(val), val = val{1}; end
    eph.(vars{v}) = val;
end

% Expose Toe as GPS seconds-of-week (expected by sat_position)
gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
try
    toe_utc2 = datetime(eph.toe, 'TimeZone', 'UTC');
catch
    toe_utc2 = eph.toe;
end
eph.Toe = mod(seconds(toe_utc2 - gps_epoch), 604800);

end % get_ephemeris


%% =========================================================================
%  LOCAL HELPER: run_wls
%  Runs WLS solver on a struct array of satellites.
%  Returns position, clock bias, post-fit residuals, H matrix, and W matrix.
% =========================================================================
function [pos_ecef, clk_bias, residuals, weights] = run_wls(sats, cfg)
% Returns weights vector (not H/W matrices) for use with chi_squared_test

n = length(sats);

pr_vec  = zeros(n, 1);
sat_pos = zeros(n, 3);
weights = zeros(n, 1);

for i = 1:n
    pr_vec(i)    = sats(i).pseudorange;
    sat_pos(i,:) = sats(i).pos_ecef(:)';
    weights(i)   = sats(i).weight;
end

% Correct signature: wls_solver(pseudoranges, sat_positions, weights, pos_init)
[pos_ecef, clk_bias, residuals] = wls_solver(pr_vec, sat_pos, weights, cfg.ref_pos);

end % run_wls


%% =========================================================================
%  LOCAL HELPER: extract_sat_ids
%  Returns a cell array of structs {prn, constellation} from a sats array.
% =========================================================================
function ids = extract_sat_ids(sats)
ids = cell(1, length(sats));
for i = 1:length(sats)
    ids{i} = struct('prn', sats(i).prn, 'constellation', sats(i).constellation);
end
end % extract_sat_ids


%% =========================================================================
%  LOCAL HELPER: empty_result
%  Returns a well-formed empty result when insufficient satellites available.
% =========================================================================
function result = empty_result(obs_epoch)
result.trusted_sats    = {};
result.spoofed_sats    = {};
result.fault_detected  = false;
result.final_chi2_stat = NaN;
result.final_chi2_thresh = NaN;
result.final_pos_ecef  = [NaN; NaN; NaN];
result.final_clk_bias  = NaN;
result.n_trusted       = 0;
result.n_excluded      = 0;
result.iterations      = 0;
result.exclusion_log   = struct([]);
% Copy satellite IDs from input for traceability
for i = 1:length(obs_epoch.prn)
    result.trusted_sats{i} = struct( ...
        'prn', obs_epoch.prn(i), ...
        'constellation', obs_epoch.constellation{i});
end
end % empty_result
