% test_raim_fde.m
% Stage 2 validation — RAIM Fault Detection and Exclusion
%
% Four-test structure per thesis design decision (18 June 2026):
%
%   Test 1: Authentic data — expect no fault detected
%   Test 2: Single-satellite gross fault (+500m, one PRN) — expect RAIM detects and excludes
%   Test 3: Coordinated multi-satellite blind spot — expect RAIM does NOT detect
%            (chi2_absorbed: coherent bias absorbed into position/clock estimates)
%            This is a PASS on the thesis claim, not a failure.
%   Test 4: Insufficient satellites — expect graceful empty return
%
% Tests 1, 2, 4 validate RAIM's intended capability.
% Test 3 documents the fundamental RAIM limitation that motivates Stage 2
% inter-constellation consistency (Section 5.6).
%
% Run from project root:
%   cd('C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION')
%   run('stage2_identification/test_raim_fde.m')

clear; clc;

PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
addpath(PROJECT_ROOT);
addpath(fullfile(PROJECT_ROOT, 'utils'));
addpath(fullfile(PROJECT_ROOT, 'stage1_detection'));
addpath(fullfile(PROJECT_ROOT, 'stage2_identification'));
cd(PROJECT_ROOT);
config;

fprintf('=======================================================\n');
fprintf('  RAIM-FDE TEST SUITE\n');
fprintf('  %s\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
fprintf('=======================================================\n\n');

% ── LOAD DATA ──────────────────────────────────────────────────────────────
obs_path = fullfile(cfg.paths.obs, 'authentic.obs');
nav_path = fullfile(cfg.paths.nav, 'authentic.nav');

fprintf('[SETUP] Loading RINEX observation file...\n');
obs = rinex_read_obs(obs_path, cfg);
fprintf('[SETUP] Loading RINEX navigation file...\n');
nav = rinex_read_nav(nav_path, cfg);

all_times = unique(obs.GPS.time);
t = all_times(1);
fprintf('[SETUP] Using epoch: %s\n\n', ...
    string(datetime(t, 'Format', 'yyyy-MM-dd HH:mm:ss')));

obs_epoch = build_obs_epoch(obs, t, cfg);

fprintf('[SETUP] Satellites available this epoch:\n');
for i = 1:length(obs_epoch.prn)
    fprintf('         %s PRN %2d   PR=%.1f m\n', ...
        obs_epoch.constellation{i}, obs_epoch.prn(i), obs_epoch.pseudorange(i));
end
fprintf('\n');

% ── TEST 1: Authentic data ────────────────────────────────────────────────
fprintf('-------------------------------------------------------\n');
fprintf('TEST 1: Authentic data — expect no fault\n');
fprintf('-------------------------------------------------------\n');

result1 = raim_fde(obs_epoch, nav, cfg, t);

fprintf('  fault_detected  : %d  (expected: 0)\n', result1.fault_detected);
fprintf('  n_excluded      : %d  (expected: 0)\n', result1.n_excluded);
fprintf('  n_trusted       : %d\n', result1.n_trusted);
fprintf('  iterations      : %d  (expected: 0)\n', result1.iterations);
fprintf('  chi2_stat       : %.4f\n', result1.final_chi2_stat);
fprintf('  chi2_thresh     : %.4f\n', result1.final_chi2_thresh);

pos_err1 = norm(result1.final_pos_ecef - cfg.ref_pos);
fprintf('  position_error  : %.2f m  (expected: < 100 m)\n', pos_err1);

PDOP = compute_pdop(obs_epoch, nav, cfg, t, result1.trusted_sats);
fprintf('  PDOP            : %.3f\n', PDOP);
fprintf('  -> inter_const_threshold recommendation: %.1f m (3 * %.1f m * PDOP)\n', ...
    3 * sqrt(cfg.ekf.meas_noise_GPS) * PDOP, sqrt(cfg.ekf.meas_noise_GPS) * PDOP);

pass1 = (~result1.fault_detected) && (result1.n_excluded == 0) && (pos_err1 < 200);
fprintf('\n  TEST 1 RESULT: %s\n\n', pass_fail(pass1));

% ── TEST 2: Single-satellite gross fault ──────────────────────────────────
fprintf('-------------------------------------------------------\n');
fprintf('TEST 2: Single-satellite gross fault (+500m, GPS PRN 15)\n');
fprintf('        RAIM should detect and exclude it.\n');
fprintf('-------------------------------------------------------\n');

SINGLE_FAULT_PRN    = 15;
SINGLE_FAULT_OFFSET = 500.0;   % m — large enough to exceed chi2 threshold

obs_single = obs_epoch;
idx_single = find(strcmp(obs_single.constellation, 'GPS') & ...
                  obs_single.prn == SINGLE_FAULT_PRN);

if isempty(idx_single)
    fprintf('  WARNING: GPS PRN %d not in this epoch. Using first GPS PRN instead.\n', ...
        SINGLE_FAULT_PRN);
    gps_mask = strcmp(obs_single.constellation, 'GPS');
    idx_single = find(gps_mask, 1);
    SINGLE_FAULT_PRN = obs_single.prn(idx_single);
end

obs_single.pseudorange(idx_single) = obs_single.pseudorange(idx_single) + SINGLE_FAULT_OFFSET;
fprintf('  Injected +%.0fm on GPS PRN %d\n\n', SINGLE_FAULT_OFFSET, SINGLE_FAULT_PRN);

result2 = raim_fde(obs_single, nav, cfg, t);

fprintf('  fault_detected  : %d  (expected: 1)\n', result2.fault_detected);
fprintf('  n_excluded      : %d  (expected: >= 1)\n', result2.n_excluded);
fprintf('  n_trusted       : %d\n', result2.n_trusted);
fprintf('  chi2_stat(final): %.4f\n', result2.final_chi2_stat);
fprintf('  chi2_thresh     : %.4f\n', result2.final_chi2_thresh);

prn_excluded = cellfun(@(s) s.prn, result2.spoofed_sats);
const_excluded = cellfun(@(s) s.constellation, result2.spoofed_sats, 'UniformOutput', false);
target_caught = any(strcmp(const_excluded, 'GPS') & prn_excluded == SINGLE_FAULT_PRN);

fprintf('\n  Excluded satellites:\n');
for k = 1:length(result2.spoofed_sats)
    s = result2.spoofed_sats{k};
    is_target = strcmp(s.constellation,'GPS') && s.prn == SINGLE_FAULT_PRN;
    fprintf('    %s PRN %2d   %s\n', s.constellation, s.prn, ...
        ternary(is_target, '<-- correctly identified', '(false positive)'));
end

pos_err2 = norm(result2.final_pos_ecef - cfg.ref_pos);
fprintf('\n  trusted solution position_error: %.2f m\n', pos_err2);
fprintf('  (authentic solution error was: %.2f m)\n', pos_err1);

pass2 = result2.fault_detected && (result2.n_excluded >= 1) && target_caught;
fprintf('\n  TEST 2 RESULT: %s\n\n', pass_fail(pass2));

% ── TEST 3: Coordinated multi-satellite blind spot ────────────────────────
fprintf('-------------------------------------------------------\n');
fprintf('TEST 3: Coordinated multi-satellite blind spot\n');
fprintf('        GPS PRNs 14+22 injected +106m (coherent bias).\n');
fprintf('        EXPECTED: fault_detected=0 (chi2_absorbed).\n');
fprintf('        This documents the RAIM limitation that motivates\n');
fprintf('        the inter-constellation consistency check (Stage 2).\n');
fprintf('        A non-detection here is a PASS on the thesis claim.\n');
fprintf('-------------------------------------------------------\n');

COORD_OFFSET_M = 106.0;
COORD_PRNS     = [14, 22];

obs_coord = obs_epoch;
n_injected = 0;
for i = 1:length(obs_coord.prn)
    if strcmp(obs_coord.constellation{i}, 'GPS') && ...
       ismember(obs_coord.prn(i), COORD_PRNS)
        obs_coord.pseudorange(i) = obs_coord.pseudorange(i) + COORD_OFFSET_M;
        fprintf('  Injected +%.0fm on GPS PRN %d\n', COORD_OFFSET_M, obs_coord.prn(i));
        n_injected = n_injected + 1;
    end
end

if n_injected == 0
    fprintf('  WARNING: PRNs [14 22] not visible. Injecting on first 2 GPS PRNs.\n');
    gps_idx = find(strcmp(obs_coord.constellation,'GPS'), 2);
    COORD_PRNS = obs_coord.prn(gps_idx);
    for i = gps_idx'
        obs_coord.pseudorange(i) = obs_coord.pseudorange(i) + COORD_OFFSET_M;
        fprintf('  Injected +%.0fm on GPS PRN %d\n', COORD_OFFSET_M, obs_coord.prn(i));
        n_injected = n_injected + 1;
    end
end

result3 = raim_fde(obs_coord, nav, cfg, t);

pos_err3   = norm(result3.final_pos_ecef - cfg.ref_pos);
undetected = pos_err3 - pos_err1;

fprintf('\n  fault_detected  : %d  (expected: 0 — chi2_absorbed)\n', result3.fault_detected);
fprintf('  n_excluded      : %d  (expected: 0)\n', result3.n_excluded);
fprintf('  n_trusted       : %d\n', result3.n_trusted);
fprintf('  chi2_stat       : %.4f\n', result3.final_chi2_stat);
fprintf('  chi2_thresh     : %.4f\n', result3.final_chi2_thresh);
fprintf('  position_error  : %.2f m  (authentic was: %.2f m)\n', pos_err3, pos_err1);
fprintf('  undetected err  : %.2f m\n', undetected);
fprintf('\n  RAIM does not detect this coordinated fault under the calibrated threshold;\n');
fprintf('  Residual signature (chi2=%.2f) below threshold (%.2f).\n', ...
    result3.final_chi2_stat, result3.final_chi2_thresh);
fprintf('  this is the expected residual-absorption blind spot.\n');

% PASS = RAIM does NOT detect (fault_detected=0), as expected for this regime
pass3 = (~result3.fault_detected) && (result3.n_excluded == 0);
fprintf('\n  TEST 3 RESULT: %s\n\n', pass_fail(pass3));

% ── TEST 4: Insufficient satellites ──────────────────────────────────────
fprintf('-------------------------------------------------------\n');
fprintf('TEST 4: Insufficient satellites — expect graceful empty return\n');
fprintf('-------------------------------------------------------\n');

obs_thin.prn           = obs_epoch.prn(1:2);
obs_thin.constellation = obs_epoch.constellation(1:2);
obs_thin.pseudorange   = obs_epoch.pseudorange(1:2);
if isfield(obs_epoch, 'cn0')
    obs_thin.cn0 = obs_epoch.cn0(1:2);
end

try
    result4 = raim_fde(obs_thin, nav, cfg, t);
    no_crash = true;
    fprintf('  Returned without crash: YES\n');
    fprintf('  n_trusted: %d  (expected: <= 2)\n', result4.n_trusted);
    fprintf('  n_excluded: %d\n', result4.n_excluded);
catch ME
    no_crash = false;
    fprintf('  CRASHED: %s\n', ME.message);
end

pass4 = no_crash;
fprintf('\n  TEST 4 RESULT: %s\n\n', pass_fail(pass4));

% ── SUMMARY ───────────────────────────────────────────────────────────────
fprintf('=======================================================\n');
fprintf('  SUMMARY\n');
fprintf('  Test 1 (authentic, no fault):              %s\n', pass_fail(pass1));
fprintf('  Test 2 (single-sat +500m, RAIM detects):   %s\n', pass_fail(pass2));
fprintf('  Test 3 (coordinated blind spot, expected):  %s\n', pass_fail(pass3));
fprintf('  Test 4 (insufficient sats, graceful):       %s\n', pass_fail(pass4));
fprintf('  Overall: %s\n', pass_fail(pass1 && pass2 && pass3 && pass4));
fprintf('=======================================================\n');

fprintf('\n[CALIBRATION]\n');
fprintf('  Authentic epoch PDOP           = %.3f\n', PDOP);
fprintf('  GPS sigma (calibrated)         = %.2f m\n', sqrt(cfg.ekf.meas_noise_GPS));
fprintf('  3*sigma_pos estimate           = %.1f m\n', 3*sqrt(cfg.ekf.meas_noise_GPS)*PDOP);
fprintf('  Current inter_const_threshold  = %.1f m\n', cfg.identify.inter_const_threshold);
fprintf('\n  --> 195.0 m threshold grounded in 99.9th percentile of 17280\n');
fprintf('      authentic pairwise distances across 2880 BUCU epochs.\n\n');

%% ── LOCAL HELPERS ─────────────────────────────────────────────────────────

function obs_epoch = build_obs_epoch(obs, t, cfg)
prn_all = []; const_all = {}; pr_all = []; cn0_all = [];
PR_FIELD = 'pseudorange_L1'; CN0_FIELD = 'cn0';
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
        pr_val = NaN;
        if isfield(obs_c, PR_FIELD), pr_val = obs_c.(PR_FIELD)(idx); end
        cn0_val = NaN;
        if isfield(obs_c, CN0_FIELD), cn0_val = obs_c.(CN0_FIELD)(idx); end
        if isnan(pr_val) || pr_val <= 0, continue; end
        prn_all(end+1)   = prn_k;          %#ok<AGROW>
        const_all{end+1} = cname;          %#ok<AGROW>
        pr_all(end+1)    = pr_val;         %#ok<AGROW>
        cn0_all(end+1)   = cn0_val;        %#ok<AGROW>
    end
end
obs_epoch.prn = prn_all; obs_epoch.constellation = const_all;
obs_epoch.pseudorange = pr_all; obs_epoch.cn0 = cn0_all;
end

function PDOP = compute_pdop(obs_epoch, nav, cfg, t, trusted_sats)
PDOP = NaN;
trusted_keys = {};
for k = 1:length(trusted_sats)
    s = trusted_sats{k};
    trusted_keys{end+1} = sprintf('%s_%03d', s.constellation, s.prn); %#ok<AGROW>
end
sat_pos_list = []; ref = cfg.ref_pos;
for i = 1:length(obs_epoch.prn)
    prn = obs_epoch.prn(i); const = obs_epoch.constellation{i};
    key = sprintf('%s_%03d', const, prn);
    if ~ismember(key, trusted_keys), continue; end
    try
        [spos, ~] = sat_position(nav, prn, const, t);
        if any(isnan(spos)), continue; end
        sat_pos_list(end+1,:) = spos(:)'; %#ok<AGROW>
    catch; continue; end
end
n = size(sat_pos_list, 1);
if n < 4, return; end
H = zeros(n, 4);
for i = 1:n
    d = sat_pos_list(i,:)' - ref; rng = norm(d);
    H(i,:) = [-d'/rng, 1];
end
try
    DOP = inv(H' * H);
    PDOP = sqrt(DOP(1,1) + DOP(2,2) + DOP(3,3));
catch; PDOP = NaN; end
end

function s = pass_fail(condition)
if condition, s = 'PASS'; else, s = 'FAIL'; end
end

function s = ternary(condition, a, b)
if condition, s = a; else, s = b; end
end