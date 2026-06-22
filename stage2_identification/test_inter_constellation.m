% test_inter_constellation.m
% Stage 2 validation — Inter-constellation position consistency check
%
% Runs three tests in sequence:
%   Test 1: Authentic data - all constellations consistent, no outlier flagged
%   Test 2: GPS drag-off spoofing - GPS flagged as outlier
%   Test 3: Single constellation only - graceful degenerate return (no comparison possible)
%
% Also calibrates cfg.identify.inter_const_threshold from authentic pairwise distances.
%
% Run from project root:
%   run('stage2_identification/test_inter_constellation.m')

%clear; 
%clc;

% -------------------------------------------------------------------------
% Self-contained path setup
% -------------------------------------------------------------------------
PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
addpath(PROJECT_ROOT);
addpath(fullfile(PROJECT_ROOT, 'utils'));
addpath(fullfile(PROJECT_ROOT, 'stage1_detection'));
addpath(fullfile(PROJECT_ROOT, 'stage2_identification'));
cd(PROJECT_ROOT);
config;

fprintf('=======================================================\n');
fprintf('  INTER-CONSTELLATION TEST SUITE\n');
fprintf('  %s\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
fprintf('=======================================================\n\n');

% -------------------------------------------------------------------------
% 0. Load data
% -------------------------------------------------------------------------
obs_path = fullfile(cfg.paths.obs, 'authentic.obs');
nav_path = fullfile(cfg.paths.nav, 'authentic.nav');

fprintf('[SETUP] Loading RINEX files...\n');
obs = rinex_read_obs(obs_path, cfg);
nav = rinex_read_nav(nav_path, cfg);

all_times = unique(obs.GPS.time);
t = all_times(1);
fprintf('[SETUP] Using epoch: %s\n\n', string(datetime(t, 'Format', 'yyyy-MM-dd HH:mm:ss')));

% Build obs_epoch for this epoch
obs_epoch = build_obs_epoch(obs, t);
fprintf('[SETUP] Total satellites this epoch: %d\n\n', length(obs_epoch.prn));

% -------------------------------------------------------------------------
% Test 1: Authentic data — expect all constellations consistent
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 1: Authentic data — expect no outlier constellation\n');
fprintf('-------------------------------------------------------\n');

result1 = inter_constellation(obs_epoch, nav, cfg, t);

fprintf('  n_constellations_solved : %d\n', result1.n_constellations_solved);
fprintf('  spoofing_suspected      : %d  (expected: 0)\n', result1.spoofing_suspected);
fprintf('  max_pairwise_dist       : %.2f m\n', result1.max_pairwise_dist);
fprintf('  consistency_threshold   : %.1f m\n', result1.consistency_threshold);

fprintf('\n  Per-constellation solutions:\n');
for c = 1:result1.n_constellations_solved
    cs = result1.constellation_solutions(c);
    [lla_lat, lla_lon, ~] = ecef2lla_simple(cs.pos_ecef); lla = [rad2deg(lla_lat), rad2deg(lla_lon)];
    dist_to_ref = norm(cs.pos_ecef - cfg.ref_pos);
    fprintf('    %-8s n=%2d  pos_err=%.1f m  lat=%.4f  lon=%.4f  %s\n', ...
        cs.name, cs.n_sats, dist_to_ref, lla(1), lla(2), ...
        ternary(result1.outlier_flags(c), '*** OUTLIER ***', 'ok'));
end

fprintf('\n  Pairwise distances (m):\n');
n_solved = result1.n_constellations_solved;
names = {result1.constellation_solutions.name};
for i = 1:n_solved
    for j = i+1:n_solved
        fprintf('    %s <-> %s : %.2f m\n', ...
            names{i}, names{j}, result1.pairwise_distances(i,j));
    end
end

% Calibration: record max authentic pairwise distance
max_authentic_dist = result1.max_pairwise_dist;
fprintf('\n  Max authentic pairwise distance: %.2f m\n', max_authentic_dist);
fprintf('  --> Current calibrated threshold = %.1f m\n', cfg.identify.inter_const_threshold);

pass1 = ~result1.spoofing_suspected && result1.n_constellations_solved >= 2;
fprintf('\n  TEST 1 RESULT: %s\n\n', pass_fail(pass1));

% -------------------------------------------------------------------------
% Test 2: GPS drag-off spoofing - expect GPS flagged as outlier
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 2: GPS drag-off spoofing - expect GPS flagged as outlier\n');
fprintf('-------------------------------------------------------\n');

obs_spoofed = obs_epoch;
target_offset = cfg.spoof.target_offset(:);
[obs_spoofed, n_injected, applied_offsets] = inject_position_drag( ...
    obs_spoofed, nav, cfg, t, 'GPS', target_offset);

fprintf('  Target GPS position offset: %.1f m\n', norm(target_offset));
fprintf('  Injected geometry-dependent offsets on %d GPS satellites\n', n_injected);
fprintf('  Offset range: %.1f m to %.1f m\n\n', min(applied_offsets), max(applied_offsets));

result2 = inter_constellation(obs_spoofed, nav, cfg, t);

fprintf('  n_constellations_solved : %d\n', result2.n_constellations_solved);
fprintf('  spoofing_suspected      : %d  (expected: 1)\n', result2.spoofing_suspected);
fprintf('  max_pairwise_dist       : %.2f m\n', result2.max_pairwise_dist);
fprintf('  outlier_constellations  : %s\n', strjoin(result2.outlier_constellations, ', '));

fprintf('\n  Per-constellation solutions:\n');
for c = 1:result2.n_constellations_solved
    cs = result2.constellation_solutions(c);
    dist_to_ref = norm(cs.pos_ecef - cfg.ref_pos);
    fprintf('    %-8s n=%2d  pos_err=%.1f m  %s\n', ...
        cs.name, cs.n_sats, dist_to_ref, ...
        ternary(result2.outlier_flags(c), '*** OUTLIER ***', 'ok'));
end

fprintf('\n  Consensus position error: %.2f m\n', ...
    norm(result2.consensus_pos_ecef - cfg.ref_pos));

gps_flagged = any(strcmp(result2.outlier_constellations, 'GPS'));
pass2 = result2.spoofing_suspected && gps_flagged;
fprintf('\n  TEST 2 RESULT: %s\n\n', pass_fail(pass2));

% -------------------------------------------------------------------------
% Test 3: Single constellation only — expect graceful degenerate return
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 3: GPS-only obs_epoch — expect graceful no-comparison return\n');
fprintf('-------------------------------------------------------\n');

obs_gps_only = obs_epoch;
gps_mask = strcmp(obs_epoch.constellation, 'GPS');
obs_gps_only.prn           = obs_epoch.prn(gps_mask);
obs_gps_only.constellation = obs_epoch.constellation(gps_mask);
obs_gps_only.pseudorange   = obs_epoch.pseudorange(gps_mask);
obs_gps_only.cn0           = obs_epoch.cn0(gps_mask);

fprintf('  Kept only %d GPS satellites\n', sum(gps_mask));

try
    result3 = inter_constellation(obs_gps_only, nav, cfg, t);
    no_crash = true;
    fprintf('  Returned without crash: YES\n');
    fprintf('  n_constellations_solved : %d  (expected: 1)\n', result3.n_constellations_solved);
    fprintf('  spoofing_suspected      : %d  (expected: 0)\n', result3.spoofing_suspected);
catch ME
    no_crash = false;
    fprintf('  CRASHED: %s\n', ME.message);
end

pass3 = no_crash && ~result3.spoofing_suspected;
fprintf('\n  TEST 3 RESULT: %s\n\n', pass_fail(pass3));

% -------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------
fprintf('=======================================================\n');
fprintf('  SUMMARY\n');
fprintf('  Test 1 (authentic, no outlier):     %s\n', pass_fail(pass1));
fprintf('  Test 2 (GPS spoofed, GPS outlier):  %s\n', pass_fail(pass2));
fprintf('  Test 3 (single const, graceful):    %s\n', pass_fail(pass3));
fprintf('  Overall: %s\n', pass_fail(pass1 && pass2 && pass3));
fprintf('=======================================================\n');

% -------------------------------------------------------------------------
% Threshold calibration summary
% -------------------------------------------------------------------------
fprintf('\n[THRESHOLD CALIBRATION]\n');
fprintf('  Max authentic pairwise distance : %.2f m\n', max_authentic_dist);
fprintf('  Current cfg.identify.inter_const_threshold = %.1f m\n', ...
    cfg.identify.inter_const_threshold);
fprintf('  --> Threshold source: 99.9th percentile from all authentic epochs.\n');
fprintf('\n');


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
%  LOCAL HELPER: inject_position_drag
%  Applies constellation-specific pseudorange changes for a target position
%  offset. A common offset is clock-like; this geometry-dependent offset is
%  what creates a displaced WLS position solution.
% =========================================================================
function [obs_out, n_injected, applied_offsets] = inject_position_drag(obs_in, nav, cfg, t, spoof_const, target_offset)

obs_out = obs_in;
n_injected = 0;
applied_offsets = [];

true_pos = cfg.ref_pos;
fake_pos = cfg.ref_pos + target_offset(:);

for i = 1:length(obs_out.prn)
    if ~strcmp(obs_out.constellation{i}, spoof_const)
        continue;
    end

    prn = obs_out.prn(i);
    [sat_pos, sat_clk] = sat_position(nav, prn, spoof_const, t);
    if any(isnan(sat_pos)) || isnan(sat_clk)
        continue;
    end

    true_range = norm(sat_pos - true_pos);
    fake_range = norm(sat_pos - fake_pos);
    delta_pr = fake_range - true_range;

    obs_out.pseudorange(i) = obs_out.pseudorange(i) + delta_pr;
    applied_offsets(end+1) = delta_pr; %#ok<AGROW>
    n_injected = n_injected + 1;
end

end % inject_position_drag


%% =========================================================================
%  LOCAL HELPERS
% =========================================================================
function s = pass_fail(condition)
    if condition, s = 'PASS'; else, s = 'FAIL'; end
end

function s = ternary(condition, a, b)
    if condition, s = a; else, s = b; end
end
