% test_classify_spoofed_sats.m
% Stage 2 validation — classify_spoofed_sats fusion layer
%
% Runs four tests:
%   Test 1: Authentic data — expect all satellites trusted, no spoofed
%   Test 2: Single-satellite fault (RAIM catches it, inter-const does not)
%   Test 3: Full GPS constellation spoofed (inter-const catches it, RAIM may not)
%   Test 4: Both detectors agree — GPS flagged by RAIM and inter-const
%
% Run from project root:
%   run('stage2_identification/test_classify_spoofed_sats.m')

clear; clc;

PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
addpath(PROJECT_ROOT);
addpath(fullfile(PROJECT_ROOT, 'utils'));
addpath(fullfile(PROJECT_ROOT, 'stage1_detection'));
addpath(fullfile(PROJECT_ROOT, 'stage2_identification'));
cd(PROJECT_ROOT);
config;

fprintf('=======================================================\n');
fprintf('  CLASSIFY_SPOOFED_SATS TEST SUITE\n');
fprintf('  %s\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
fprintf('=======================================================\n\n');

% -------------------------------------------------------------------------
% 0. Load data and build authentic obs_epoch
% -------------------------------------------------------------------------
fprintf('[SETUP] Loading RINEX files...\n');
obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);

all_times = unique(obs.GPS.time);
t = all_times(1);
fprintf('[SETUP] Using epoch: %s\n\n', string(datetime(t,'Format','yyyy-MM-dd HH:mm:ss')));

obs_epoch = build_obs_epoch(obs, t);
fprintf('[SETUP] Total satellites: %d\n\n', length(obs_epoch.prn));

% -------------------------------------------------------------------------
% Test 1: Authentic data — all satellites should be trusted
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 1: Authentic data — expect all trusted\n');
fprintf('-------------------------------------------------------\n');

cfg.verbose = 0;
raim_r1  = raim_fde(obs_epoch, nav, cfg, t);
inter_r1 = inter_constellation(obs_epoch, nav, cfg, t);
cfg.verbose = 1;

result1 = classify_spoofed_sats(raim_r1, inter_r1, obs_epoch, cfg);

fprintf('  n_trusted  : %d  (expected: %d)\n', result1.n_trusted, length(obs_epoch.prn));
fprintf('  n_spoofed  : %d  (expected: 0)\n', result1.n_spoofed);
fprintf('  n_suspect  : %d  (expected: 0)\n', result1.n_suspect);
fprintf('  attack_type: %s  (expected: none)\n', result1.attack_type);

pass1 = (result1.n_spoofed == 0) && (result1.n_suspect == 0) && ...
        strcmp(result1.attack_type, 'none');
fprintf('\n  TEST 1 RESULT: %s\n\n', pass_fail(pass1));

% -------------------------------------------------------------------------
% Test 2: Single GPS satellite fault — RAIM catches, inter-const unaffected
%   Inject large offset on ONE satellite only.
%   RAIM-FDE should exclude it. inter_constellation should not flag GPS
%   (one bad satellite does not displace the whole GPS solution enough).
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 2: Single satellite fault (GPS PRN 15, +500m)\n');
fprintf('        RAIM should catch it; inter-const should not flag GPS\n');
fprintf('-------------------------------------------------------\n');

obs_t2 = obs_epoch;
for i = 1:length(obs_t2.prn)
    if strcmp(obs_t2.constellation{i},'GPS') && obs_t2.prn(i)==15
        obs_t2.pseudorange(i) = obs_t2.pseudorange(i) + 500.0;
        fprintf('  Injected +500m on GPS PRN 15\n\n');
    end
end

cfg.verbose = 0;
raim_r2  = raim_fde(obs_t2, nav, cfg, t);
inter_r2 = inter_constellation(obs_t2, nav, cfg, t);
cfg.verbose = 1;

fprintf('  raim_fde:           fault_detected=%d  n_excluded=%d\n', ...
    raim_r2.fault_detected, raim_r2.n_excluded);
fprintf('  inter_constellation: spoofing_suspected=%d  outliers={%s}\n', ...
    inter_r2.spoofing_suspected, strjoin(inter_r2.outlier_constellations,','));

result2 = classify_spoofed_sats(raim_r2, inter_r2, obs_t2, cfg);

fprintf('\n  n_trusted  : %d\n', result2.n_trusted);
fprintf('  n_spoofed  : %d  (expected: >= 1)\n', result2.n_spoofed);
fprintf('  attack_type: %s  (expected: single_sat)\n', result2.attack_type);

% Check PRN 15 was classified spoofed
prn15_spoofed = false;
for i = 1:length(obs_t2.prn)
    if strcmp(obs_t2.constellation{i},'GPS') && obs_t2.prn(i)==15
        if strcmp(result2.classifications(i).status,'spoofed')
            prn15_spoofed = true;
        end
        fprintf('  GPS PRN 15 status: %s  (expected: spoofed)\n', ...
            result2.classifications(i).status);
    end
end

% Count GPS satellites spoofed — in single_sat mode, should be exactly 1
n_gps_spoofed_t2 = 0;
for i = 1:length(obs_t2.prn)
    if strcmp(obs_t2.constellation{i},'GPS') && strcmp(result2.classifications(i).status,'spoofed')
        n_gps_spoofed_t2 = n_gps_spoofed_t2 + 1;
    end
end
fprintf('  GPS satellites spoofed: %d  (expected: 1)\n', n_gps_spoofed_t2);

% Pass conditions:
%   1. attack_type = single_sat
%   2. PRN 15 specifically is spoofed
%   3. Exactly 1 GPS satellite spoofed (not all of GPS)
pass2 = strcmp(result2.attack_type,'single_sat') && prn15_spoofed && (n_gps_spoofed_t2 == 1);
fprintf('\n  TEST 2 RESULT: %s\n\n', pass_fail(pass2));

% -------------------------------------------------------------------------
% Test 3: Full GPS constellation spoofed via position drag-off
%   inter_constellation should flag GPS; RAIM may or may not
%   (chi-sq absorption possible for coherent attack)
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 3: Full GPS constellation drag-off (591.6m offset)\n');
fprintf('        inter-const should flag GPS\n');
fprintf('-------------------------------------------------------\n');

% cfg.spoof.target_offset is a [1x3] ENU vector [East, North, Up] in metres.
% Convert to ECEF displacement using rotation matrix at Bucharest.
% Reference: Misra & Enge, "Global Positioning System", 2nd ed., Appendix A.
if isfield(cfg,'spoof') && isfield(cfg.spoof,'target_offset')
    enu_offset = cfg.spoof.target_offset(:);   % ensure [3x1]
else
    enu_offset = [0; 591.6; 0];   % fallback: 591.6 m North
end

% Derive lat/lon from cfg.ref_pos — do not hardcode
[lat_deg, lon_deg, ~] = ecef2lla_simple(cfg.ref_pos);
lat = lat_deg;   % ecef2lla_simple returns radians
lon = lon_deg;

% ENU to ECEF rotation matrix
R_enu2ecef = [-sin(lon),          -sin(lat)*cos(lon),  cos(lat)*cos(lon);
               cos(lon),          -sin(lat)*sin(lon),  cos(lat)*sin(lon);
               0,                  cos(lat),            sin(lat)         ];

ecef_offset = R_enu2ecef * enu_offset;
true_pos    = cfg.ref_pos;
fake_pos    = true_pos + ecef_offset;

fprintf('  ENU offset : [%.1f E, %.1f N, %.1f U] m\n', enu_offset(1), enu_offset(2), enu_offset(3));
fprintf('  Magnitude  : %.1f m\n', norm(enu_offset));

obs_t3 = obs_epoch;
n_injected = 0;
for i = 1:length(obs_t3.prn)
    if strcmp(obs_t3.constellation{i},'GPS')
        [sp, ~] = sat_position(nav, obs_t3.prn(i), 'GPS', t);
        if any(isnan(sp)), continue; end
        delta_pr = norm(sp - fake_pos) - norm(sp - true_pos);
        obs_t3.pseudorange(i) = obs_t3.pseudorange(i) + delta_pr;
        n_injected = n_injected + 1;
    end
end
fprintf('  Injected geometry-dependent drag-off on %d GPS satellites\n', n_injected);
fprintf('  Effective position offset (magnitude): %.1f m\n\n', norm(enu_offset));

cfg.verbose = 0;
raim_r3  = raim_fde(obs_t3, nav, cfg, t);
inter_r3 = inter_constellation(obs_t3, nav, cfg, t);
cfg.verbose = 1;

fprintf('  raim_fde:            fault_detected=%d  n_excluded=%d\n', ...
    raim_r3.fault_detected, raim_r3.n_excluded);
fprintf('  inter_constellation: spoofing_suspected=%d  outliers={%s}\n', ...
    inter_r3.spoofing_suspected, strjoin(inter_r3.outlier_constellations,','));

result3 = classify_spoofed_sats(raim_r3, inter_r3, obs_t3, cfg);

fprintf('\n  n_trusted  : %d\n', result3.n_trusted);
fprintf('  n_spoofed  : %d  (expected: >= 1)\n', result3.n_spoofed);
fprintf('  attack_type: %s  (expected: constellation)\n', result3.attack_type);
fprintf('  recommended_action: %s\n', result3.recommended_action);

% Count GPS satellites classified as spoofed
n_gps_spoofed = 0;
for i = 1:length(obs_t3.prn)
    if strcmp(obs_t3.constellation{i},'GPS') && ...
       strcmp(result3.classifications(i).status,'spoofed')
        n_gps_spoofed = n_gps_spoofed + 1;
    end
end
fprintf('  GPS satellites classified spoofed: %d / %d\n', n_gps_spoofed, n_injected);

% Count non-GPS satellites incorrectly marked spoofed
n_nongps_spoofed = 0;
for i = 1:length(obs_t3.prn)
    if ~strcmp(obs_t3.constellation{i},'GPS') && ...
       strcmp(result3.classifications(i).status,'spoofed')
        n_nongps_spoofed = n_nongps_spoofed + 1;
        fprintf('  WARNING: %s PRN %d incorrectly marked spoofed\n', ...
            obs_t3.constellation{i}, obs_t3.prn(i));
    end
end
fprintf('  Non-GPS satellites incorrectly marked spoofed: %d  (expected: 0)\n', n_nongps_spoofed);

% Pass conditions:
%   1. attack_type = constellation (not single_sat)
%   2. At least some GPS satellites flagged (spoofed or suspect)
%   3. No non-GPS satellites falsely marked spoofed
n_gps_flagged = 0;
for i = 1:length(obs_t3.prn)
    if strcmp(obs_t3.constellation{i},'GPS') && ...
       ~strcmp(result3.classifications(i).status,'trusted')
        n_gps_flagged = n_gps_flagged + 1;
    end
end
fprintf('  GPS satellites flagged (spoofed+suspect): %d / %d\n', n_gps_flagged, n_injected);

pass3 = strcmp(result3.attack_type,'constellation') && ...
        (n_gps_flagged >= 1) && ...
        (n_nongps_spoofed == 0);
fprintf('\n  TEST 3 RESULT: %s\n\n', pass_fail(pass3));

% -------------------------------------------------------------------------
% Test 4: Both detectors agree — single GPS satellite, large offset
%   RAIM excludes it AND it belongs to an inter-const flagged constellation
%   fusion:both_detectors evidence path
% -------------------------------------------------------------------------
fprintf('-------------------------------------------------------\n');
fprintf('TEST 4: Both detectors agree (GPS PRN 15, large offset)\n');
fprintf('        Expect fusion:both_detectors evidence on PRN 15\n');
fprintf('-------------------------------------------------------\n');

% Inject large offset on GPS PRN 15 AND displace all GPS geometrically
% so inter-const also flags GPS
obs_t4 = obs_t3;   % start from the drag-off scenario
for i = 1:length(obs_t4.prn)
    if strcmp(obs_t4.constellation{i},'GPS') && obs_t4.prn(i)==15
        obs_t4.pseudorange(i) = obs_t4.pseudorange(i) + 500.0;
        fprintf('  Additional +500m on GPS PRN 15 (on top of drag-off)\n\n');
    end
end

cfg.verbose = 0;
raim_r4  = raim_fde(obs_t4, nav, cfg, t);
inter_r4 = inter_constellation(obs_t4, nav, cfg, t);
cfg.verbose = 1;

result4 = classify_spoofed_sats(raim_r4, inter_r4, obs_t4, cfg);

% Find PRN 15 classification and check for both_detectors evidence
prn15_both = false;
for i = 1:length(obs_t4.prn)
    if strcmp(obs_t4.constellation{i},'GPS') && obs_t4.prn(i)==15
        status   = result4.classifications(i).status;
        evidence = result4.classifications(i).evidence;
        fprintf('  GPS PRN 15 status  : %s\n', status);
        fprintf('  GPS PRN 15 evidence: %s\n', strjoin(evidence,', '));
        has_both = any(contains(evidence,'both_detectors'));
        has_raim = any(contains(evidence,'raim_fde'));
        has_inter = any(contains(evidence,'inter_constellation'));
        prn15_both = strcmp(status,'spoofed') && (has_both || (has_raim && has_inter));
    end
end

fprintf('\n  n_spoofed: %d\n', result4.n_spoofed);
pass4 = prn15_both && (result4.n_spoofed >= 1);
fprintf('\n  TEST 4 RESULT: %s\n\n', pass_fail(pass4));

% -------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------
fprintf('=======================================================\n');
fprintf('  SUMMARY\n');
fprintf('  Test 1 (authentic, all trusted):          %s\n', pass_fail(pass1));
fprintf('  Test 2 (single sat fault, RAIM):          %s\n', pass_fail(pass2));
fprintf('  Test 3 (full GPS drag-off, inter-const):  %s\n', pass_fail(pass3));
fprintf('  Test 4 (both detectors agree):            %s\n', pass_fail(pass4));
fprintf('  Overall: %s\n', pass_fail(pass1&&pass2&&pass3&&pass4));
fprintf('=======================================================\n\n');


%% =========================================================================
%  LOCAL HELPER: build_obs_epoch
% =========================================================================
function obs_epoch = build_obs_epoch(obs, t)

prn_all=[]; const_all={}; pr_all=[]; cn0_all=[];
PR_FIELD='pseudorange_L1'; CN0_FIELD='cn0';

for cn = {'GPS','Galileo','BeiDou','GLONASS'}
    c = cn{1};
    if ~isfield(obs,c), continue; end
    obs_c = obs.(c);
    tm = (obs_c.time == t);
    if ~any(tm), continue; end

    for prn = obs_c.prn(tm)'
        mk  = (obs_c.time==t) & (obs_c.prn==prn);
        idx = find(mk,1);
        if isempty(idx), continue; end

        pr  = NaN; cn0 = NaN;
        if isfield(obs_c,PR_FIELD),  pr  = obs_c.(PR_FIELD)(idx);  end
        if isfield(obs_c,CN0_FIELD), cn0 = obs_c.(CN0_FIELD)(idx); end
        if isnan(pr)||pr<=0, continue; end

        prn_all(end+1)   = prn;   %#ok<AGROW>
        const_all{end+1} = c;     %#ok<AGROW>
        pr_all(end+1)    = pr;    %#ok<AGROW>
        cn0_all(end+1)   = cn0;   %#ok<AGROW>
    end
end

obs_epoch.prn           = prn_all;
obs_epoch.constellation = const_all;
obs_epoch.pseudorange   = pr_all;
obs_epoch.cn0           = cn0_all;

end % build_obs_epoch


%% =========================================================================
%  LOCAL HELPERS
% =========================================================================
function s = pass_fail(c)
    if c, s = 'PASS'; else, s = 'FAIL'; end
end