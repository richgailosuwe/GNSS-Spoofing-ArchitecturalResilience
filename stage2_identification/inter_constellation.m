function result = inter_constellation(obs_epoch, nav, cfg, t)
% INTER_CONSTELLATION  Cross-constellation position consistency check
%
% The PRIMARY identification tool for multi-satellite spoofing.
%
% MOTIVATION (from chi_squared_test validation, epoch 500):
%   Sophisticated spoofing of an entire constellation shifts all pseudoranges
%   coherently, so the WLS solver absorbs the error into its position estimate.
%   Post-fit residuals look internally consistent → chi-squared test PASSES.
%   However, an independent constellation (e.g. Galileo, BeiDou) that was NOT
%   spoofed will yield a position solution displaced by 100m+ from the spoofed
%   GPS solution. This displacement is detectable and unambiguous.
%
% ALGORITHM:
%   1. For each constellation with >= min_sats satellites, compute an
%      independent WLS position solution.
%   2. Compute pairwise ECEF distances between constellation solutions.
%   3. Flag constellations whose solution is an outlier relative to the
%      majority — using a median-based robust consensus.
%   4. Return per-constellation flags and the consensus reference position.
%
% INPUTS:
%   obs_epoch  struct  — single-epoch observables (all constellations)
%                        fields: .prn, .constellation, .pseudorange
%   nav        struct  — navigation/ephemeris data
%   cfg        struct  — configuration (output of config.m)
%   t          datetime — current epoch timestamp (UTC)
%
% OUTPUT:
%   result     struct with fields:
%     .constellation_solutions  struct array — per-constellation position results
%     .pairwise_distances       [NxN double] — ECEF distances between solutions (m)
%     .outlier_flags            [Nx1 logical] — true = outlier constellation
%     .outlier_constellations   cell          — names of flagged constellations
%     .consensus_pos_ecef       [3x1]         — robust consensus position
%     .max_pairwise_dist        double        — largest pairwise distance (m)
%     .n_constellations_solved  int           — number of independent solutions
%     .spoofing_suspected       logical       — true if any outlier found

% -------------------------------------------------------------------------
% 0. Parameters
% -------------------------------------------------------------------------
% Position consistency threshold: disagreement beyond this → spoofing suspected
% Set relative to detection sensitivity (should be >> measurement noise but
% << expected spoofing offset of ~100m).
% Value: 3*sigma where sigma ~ 18.3m → ~55m; use 50m as conservative round number.
CONSISTENCY_THRESHOLD_M = cfg.identify.inter_const_threshold;   % default: 50.0 m

MIN_SATS_PER_CONST = cfg.identify.min_sats_per_constellation;   % default: 4

% -------------------------------------------------------------------------
% 1. Group satellites by constellation
% -------------------------------------------------------------------------
constellations = unique(obs_epoch.constellation);
n_const = length(constellations);

const_solutions = struct( ...
    'name',     {}, ...
    'pos_ecef', {}, ...
    'clk_bias', {}, ...
    'n_sats',   {}, ...
    'solved',   {} ...
);

n_solved = 0;

for c = 1:n_const
    cname = constellations{c};
    mask  = strcmp(obs_epoch.constellation, cname);

    % Extract this constellation's observables
    sub_obs.prn           = obs_epoch.prn(mask);
    sub_obs.constellation = obs_epoch.constellation(mask);
    sub_obs.pseudorange   = obs_epoch.pseudorange(mask);
    if isfield(obs_epoch, 'cn0')
        sub_obs.cn0 = obs_epoch.cn0(mask);
    end

    n_sats = sum(mask);
    if n_sats < MIN_SATS_PER_CONST
        if cfg.verbose
            fprintf('[INTER_CONST] %s: only %d sats — skipping (need %d)\n', ...
                cname, n_sats, MIN_SATS_PER_CONST);
        end
        continue;
    end

    % Build satellite list and compute positions
    sats = build_sat_list_const(sub_obs, nav, cfg, t, cname);
    valid_sats = sats([sats.valid]);

    if length(valid_sats) < MIN_SATS_PER_CONST
        if cfg.verbose
            fprintf('[INTER_CONST] %s: only %d valid positions — skipping\n', ...
                cname, length(valid_sats));
        end
        continue;
    end

    % Independent WLS solution for this constellation
    try
        [pos_c, clk_c] = run_const_wls(valid_sats, cfg);

        if any(isnan(pos_c))
            continue;
        end

        n_solved = n_solved + 1;
        const_solutions(n_solved).name     = cname;
        const_solutions(n_solved).pos_ecef = pos_c;
        const_solutions(n_solved).clk_bias = clk_c;
        const_solutions(n_solved).n_sats   = length(valid_sats);
        const_solutions(n_solved).solved   = true;

        if cfg.verbose
            % Keep verbose logging simple; test scripts print geodetic details.
            fprintf('[INTER_CONST] %s: solved clk=%.1fm n=%d\n', ...
                cname, clk_c, length(valid_sats));
        end

    catch ME
        if cfg.verbose
            fprintf('[INTER_CONST] %s: WLS failed — %s\n', cname, ME.message);
        end
    end
end

% -------------------------------------------------------------------------
% 2. Handle degenerate case: fewer than 2 constellations solved
% -------------------------------------------------------------------------
if n_solved < 2
    if cfg.verbose
        fprintf('[INTER_CONST] Only %d constellation(s) solved — cannot compare.\n', n_solved);
    end
    result = build_degenerate_result(const_solutions, n_solved);
    return;
end

% -------------------------------------------------------------------------
% 3. Pairwise ECEF distance matrix
% -------------------------------------------------------------------------
dist_matrix = zeros(n_solved, n_solved);
for i = 1:n_solved
    for j = i+1:n_solved
        d = norm(const_solutions(i).pos_ecef - const_solutions(j).pos_ecef);
        dist_matrix(i,j) = d;
        dist_matrix(j,i) = d;
    end
end

% -------------------------------------------------------------------------
% 4. Robust outlier detection via median pairwise distance
%
% For each constellation i, compute the MEDIAN distance to all other
% constellations. Outliers have systematically large median distances,
% meaning they disagree with the majority.
%
% This is robust against a single honest constellation disagring with
% multiple spoofed ones — the median naturally sides with the majority.
% -------------------------------------------------------------------------
median_dists = zeros(n_solved, 1);
for i = 1:n_solved
    other_dists = dist_matrix(i, [1:i-1, i+1:n_solved]);
    median_dists(i) = median(other_dists);
end

outlier_flags = median_dists > CONSISTENCY_THRESHOLD_M;
outlier_names = {};
for i = 1:n_solved
    if outlier_flags(i)
        outlier_names{end+1} = const_solutions(i).name; %#ok<AGROW>
    end
end

% -------------------------------------------------------------------------
% 5. Consensus position — average of non-outlier constellations (robust mean)
% -------------------------------------------------------------------------
consensus_indices = find(~outlier_flags);
if isempty(consensus_indices)
    % All constellations flagged as outliers — impossible to determine truth.
    % Fall back to the pair with the minimum pairwise distance (most agreement).
    [~, best_pair_lin] = min(dist_matrix(dist_matrix > 0));
    [r, ~] = ind2sub([n_solved, n_solved], best_pair_lin);
    consensus_indices = r;
    warning('inter_constellation: all constellations flagged — falling back to best pair.');
end

consensus_positions = zeros(3, length(consensus_indices));
for k = 1:length(consensus_indices)
    consensus_positions(:,k) = const_solutions(consensus_indices(k)).pos_ecef;
end
consensus_pos = mean(consensus_positions, 2);

% -------------------------------------------------------------------------
% 6. Assemble result
% -------------------------------------------------------------------------
result.constellation_solutions = const_solutions;
result.pairwise_distances      = dist_matrix;
result.outlier_flags           = outlier_flags;
result.outlier_constellations  = outlier_names;
result.consensus_pos_ecef      = consensus_pos;
result.max_pairwise_dist       = max(dist_matrix(:));
result.n_constellations_solved = n_solved;
result.spoofing_suspected      = any(outlier_flags);
result.consistency_threshold   = CONSISTENCY_THRESHOLD_M;

% -------------------------------------------------------------------------
% 7. Verbose summary
% -------------------------------------------------------------------------
if cfg.verbose
    fprintf('[INTER_CONST] t=%s | solved=%d | max_dist=%.1fm | threshold=%.1fm\n', ...
        string(datetime(t, 'Format', 'HH:mm:ss')), n_solved, result.max_pairwise_dist, CONSISTENCY_THRESHOLD_M);
    if result.spoofing_suspected
        fprintf('[INTER_CONST]   *** SPOOFING SUSPECTED — outlier constellation(s): %s\n', ...
            strjoin(outlier_names, ', '));
    else
        fprintf('[INTER_CONST]   All constellations consistent.\n');
    end
end

end % main function


%% =========================================================================
%  LOCAL HELPER: build_sat_list_const
%  Computes satellite positions for a single constellation's observables.
% =========================================================================
function sats = build_sat_list_const(sub_obs, nav, cfg, t, constellation)

n = length(sub_obs.prn);
sats = struct('prn', cell(1,n), 'pos_ecef', cell(1,n), ...
              'pseudorange', cell(1,n), 'weight', cell(1,n), 'valid', cell(1,n));

switch constellation
    case 'GPS';     sigma2 = cfg.ekf.meas_noise_GPS;
    case 'Galileo'; sigma2 = cfg.ekf.meas_noise_Galileo;
    case 'BeiDou';  sigma2 = cfg.ekf.meas_noise_BeiDou;
    case 'GLONASS'; sigma2 = cfg.ekf.meas_noise_GLONASS;
    otherwise;      sigma2 = 500.0;
end

for i = 1:n
    prn = sub_obs.prn(i);
    pr  = sub_obs.pseudorange(i);

    sats(i).prn         = prn;
    sats(i).pseudorange = pr;
    sats(i).weight      = 1.0 / sigma2;
    sats(i).valid       = false;

    if isnan(pr) || pr <= 0
        continue;
    end

    try
        % Transmit-time corrected measurement. Returns sat_pos at TRANSMIT
        % time, stored below so the per-constellation WLS geometry is
        % consistent with the correction. rec_approx = cfg.ref_pos.
        [pr_corr, sat_pos] = corrected_pseudorange(pr, prn, constellation, ...
                                 t, cfg.ref_pos(:), nav, cfg);
        if isnan(pr_corr), continue; end   % elevation-masked or invalid — skip

        sats(i).pos_ecef    = sat_pos;
        sats(i).pseudorange = pr_corr;
        sats(i).valid       = true;

    catch
        % silently skip — already filtered above
    end
end

end % build_sat_list_const


%% =========================================================================
%  LOCAL HELPER: get_ephemeris_const
% =========================================================================
function eph = get_ephemeris_const(nav, constellation, prn, t)
% nav.(constellation) is a single struct with fields:
%   .prn  [N×1 double]
%   .toe  [N×1 datetime]
%   .data [N×32 timetable]

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

nav_const = nav.(field);

prn_mask = (nav_const.prn == prn);
if ~any(prn_mask), return; end

toe_candidates = nav_const.toe(prn_mask);
t_dt = datetime(t, 'TimeZone', 'UTC');
try
    toe_utc = datetime(toe_candidates, 'TimeZone', 'UTC');
catch
    toe_utc = toe_candidates;
end
dt_sec = abs(seconds(toe_utc - t_dt));
[~, best_local] = min(dt_sec);

global_indices = find(prn_mask);
best_row = global_indices(best_local);

row  = nav_const.data(best_row, :);
vars = row.Properties.VariableNames;

eph = struct();
eph.prn = prn;
eph.toe = nav_const.toe(best_row);

for v = 1:length(vars)
    val = row.(vars{v});
    if iscell(val), val = val{1}; end
    eph.(vars{v}) = val;
end

gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
try
    toe_utc2 = datetime(eph.toe, 'TimeZone', 'UTC');
catch
    toe_utc2 = eph.toe;
end
eph.Toe = mod(seconds(toe_utc2 - gps_epoch), 604800);

end % get_ephemeris_const


%% =========================================================================
%  LOCAL HELPER: run_const_wls
%  Runs WLS for a single constellation's valid satellite struct array.
% =========================================================================
function [pos_ecef, clk_bias] = run_const_wls(valid_sats, cfg)

n = length(valid_sats);
pr_vec  = zeros(n, 1);
sat_pos = zeros(n, 3);
weights = zeros(n, 1);

for i = 1:n
    pr_vec(i)    = valid_sats(i).pseudorange;
    sat_pos(i,:) = valid_sats(i).pos_ecef(:)';
    weights(i)   = valid_sats(i).weight;
end

% Correct signature: wls_solver(pseudoranges, sat_positions, weights, pos_init)
[pos_ecef, clk_bias] = wls_solver(pr_vec, sat_pos, weights, cfg.ref_pos);

end % run_const_wls


%% =========================================================================
%  LOCAL HELPER: build_degenerate_result
% =========================================================================
function result = build_degenerate_result(const_solutions, n_solved)

result.constellation_solutions = const_solutions;
result.pairwise_distances      = zeros(n_solved, n_solved);
result.outlier_flags           = false(n_solved, 1);
result.outlier_constellations  = {};
result.consensus_pos_ecef      = [NaN; NaN; NaN];
result.max_pairwise_dist       = NaN;
result.n_constellations_solved = n_solved;
result.spoofing_suspected      = false;
result.consistency_threshold   = NaN;

if n_solved == 1
    result.consensus_pos_ecef = const_solutions(1).pos_ecef;
end

end % build_degenerate_result

%% ARCHITECTURAL BOUNDARY CONDITIONS — inter_constellation reliability limits
%
% This detector uses a median-based majority vote across constellation solutions.
% Reliability depends on the ratio of spoofed to honest constellations:
%
%   1 spoofed,  3 honest  → RELIABLE
%     The spoofed constellation disagrees with all 3 honest ones.
%     Its median pairwise distance exceeds threshold → correctly flagged.
%     Covers Scenarios 1 (GPS), 2 (Galileo), 3 (BeiDou).
%
%   2 spoofed,  2 honest  → UNRELIABLE
%     Both groups agree internally, disagree with each other.
%     All 4 median distances exceed threshold → all constellations flagged.
%     Fallback to best pair fires — result is indeterminate.
%     Covers Scenario 4 (GPS+GLONASS), Scenario 5 (GPS+Galileo).
%
%   3 spoofed,  1 honest  → INVERTED — dangerous failure mode
%     The 3 spoofed agree with each other; the honest one disagrees with all 3.
%     The honest constellation has the largest median distance → flagged as outlier.
%     Spoofed constellations declared trusted. Detector inverts.
%
% CONSEQUENCE FOR ARCHITECTURE (Chapter 4):
%   inter_constellation is the PRIMARY identifier only for single-constellation
%   attacks. For multi-constellation attacks (Scenarios 4 and 5), RAIM-FDE
%   and Stage 1 detection carry the identification burden.
%   classify_spoofed_sats fuses all evidence sources precisely because no
%   single detector covers all attack scenarios.
%
% REFERENCE:
%   Majority-vote geometric median outlier detection — see Rousseeuw & Leroy
%   (1987), "Robust Regression and Outlier Detection", Wiley, Chapter 7.
%   RAIM reliability under multiple faults — ESA Navipedia, RAIM Fundamentals,
%   https://gssc.esa.int/navipedia/index.php/RAIM_Fundamentals
