%% TEST_APPLY_EXCLUSION_MASK  Unit tests for apply_exclusion_mask.m
%
% Run from project root:
%   run('stage3_exclusion/test_apply_exclusion_mask.m')
%
% Expected result: PASS (5/5)
%
% Test design follows the mandatory debugging lifecycle documented in the
% project protocols: each test is deterministic, describes its expected
% outcome, and will FAIL explicitly if the function is absent or broken.
%
% Test 0 is a regression test for the original bug: obs struct with NO .weight
% field (as returned by rinex_read_obs) must not crash the function.
%

fprintf('\n=== test_apply_exclusion_mask.m ===\n');

% Clear local test variables to prevent workspace pollution from prior runs.
clear obs cr sat_list result

% ---- Minimal cfg -----------------------------------------------------------
cfg.identify.min_sats             = 5;
cfg.stage3.spoof_weight_inflation  = 1e6;
cfg.stage3.suspect_weight_inflation = 5;

% ---- Build a synthetic obs_epoch WITH .weight (tests 1-4) ------------------
% GPS: 4 satellites (PRNs 1-4)
% Galileo: 3 satellites (PRNs 1-3)
% BeiDou: 3 satellites (PRNs 1-3)

w_gps = 1/333;  w_gal = 1/301;  w_bds = 1/4972;

obs.GPS.prn         = [1;2;3;4];
obs.GPS.weight      = repmat(w_gps, 4, 1);
obs.GPS.pseudorange = zeros(4,1);
obs.GPS.cn0         = 40*ones(4,1);
% Realistic elevation spread (rad). Identical elevations make the approximate
% geometry matrix rank-deficient (sin(el) column constant) -> condition number
% ~1e16 -> insufficient_geometry spuriously true. A varied 15-80 deg spread
% across the trusted set is well-conditioned (combined cond ~37 << 1e6), so the
% geometry flag reflects real geometry, not a degenerate test fixture.
% NOTE: these specific angles are an ILLUSTRATIVE realistic spread, not taken
% from any standard or dataset. They are unit-test fixture inputs only and do
% not affect any reported result, scenario, or the live hardware processing.
obs.GPS.elevation   = deg2rad([15;35;55;75]);

obs.Galileo.prn         = [1;2;3];
obs.Galileo.weight      = repmat(w_gal, 3, 1);
obs.Galileo.pseudorange = zeros(3,1);
obs.Galileo.cn0         = 40*ones(3,1);
obs.Galileo.elevation   = deg2rad([25;45;65]);

obs.BeiDou.prn         = [1;2;3];
obs.BeiDou.weight      = repmat(w_bds, 3, 1);
obs.BeiDou.pseudorange = zeros(3,1);
obs.BeiDou.cn0         = 40*ones(3,1);
obs.BeiDou.elevation   = deg2rad([20;50;80]);

n_tests = 0;
n_pass  = 0;

%% ---- Test 0: REGRESSION — obs struct without .weight field ----------------
% Root cause of original crash: rinex_read_obs does not add a .weight field.
% apply_exclusion_mask must initialise weights from cfg.ekf.meas_noise_* when
% .weight is absent, rather than crashing with "Unrecognized field name".
fprintf('\nTest 0: obs struct WITHOUT .weight field — must not crash\n');
n_tests = n_tests + 1;

obs_noweight.GPS.prn         = [1;2;3;4];
obs_noweight.GPS.pseudorange = zeros(4,1);
obs_noweight.GPS.cn0         = 40*ones(4,1);
obs_noweight.GPS.elevation   = deg2rad([15;35;55;75]);
% NOTE: no .weight field — deliberate, matches rinex_read_obs output

obs_noweight.Galileo.prn         = [1;2;3];
obs_noweight.Galileo.pseudorange = zeros(3,1);
obs_noweight.Galileo.cn0         = 40*ones(3,1);
obs_noweight.Galileo.elevation   = deg2rad([25;45;65]);

obs_noweight.BeiDou.prn         = [1;2;3];
obs_noweight.BeiDou.pseudorange = zeros(3,1);
obs_noweight.BeiDou.cn0         = 40*ones(3,1);
obs_noweight.BeiDou.elevation   = deg2rad([20;50;80]);

% All trusted sat_list
sat_list0 = struct('constellation',{},'prn',{},'status',{});
for k = 1:4
    sat_list0(end+1).constellation = 'GPS';     sat_list0(end).prn = k; sat_list0(end).status = 'trusted';
end
for k = 1:3
    sat_list0(end+1).constellation = 'Galileo'; sat_list0(end).prn = k; sat_list0(end).status = 'trusted';
end
for k = 1:3
    sat_list0(end+1).constellation = 'BeiDou';  sat_list0(end).prn = k; sat_list0(end).status = 'trusted';
end
cr0.sat_list = sat_list0; cr0.n_trusted = 10; cr0.n_suspect = 0; cr0.n_spoofed = 0;

% Add cfg.ekf.meas_noise_* so the fallback initialisation can read them.
cfg.ekf.meas_noise_GPS     = 333.0;
cfg.ekf.meas_noise_Galileo = 301.0;
cfg.ekf.meas_noise_BeiDou  = 4972.0;
cfg.ekf.meas_noise_GLONASS = 3476.0;

try
    result0 = apply_exclusion_mask(obs_noweight, cr0, cfg);
    % Weight field must now exist and equal 1/sigma^2
    expected_gps_w = 1 / 333.0;
    ok0 = isfield(result0.GPS, 'weight') && ...
          abs(result0.GPS.weight(1) - expected_gps_w) < 1e-12 && ...
          result0.n_trusted_post_mask == 10;
    if ok0
        fprintf('  PASS — no crash; GPS weight initialised to %.6e (expected %.6e)\n', ...
            result0.GPS.weight(1), expected_gps_w);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL — ran without crash but wrong weight or count\n');
        fprintf('    GPS weight(1) = %.6e, expected %.6e\n', result0.GPS.weight(1), expected_gps_w);
        fprintf('    n_trusted     = %d, expected 10\n', result0.n_trusted_post_mask);
    end
catch ME
    fprintf('  FAIL — crashed with: %s\n', ME.message);
end

%% ---- Test 1: All trusted — weights unchanged ------------------------------
fprintf('\nTest 1: All satellites trusted — weights must be unchanged\n');
n_tests = n_tests + 1;

sat_list = struct('constellation',{},'prn',{},'status',{});
for k = 1:4
    sat_list(end+1).constellation = 'GPS';
    sat_list(end).prn    = k;
    sat_list(end).status = 'trusted';
end
for k = 1:3
    sat_list(end+1).constellation = 'Galileo';
    sat_list(end).prn    = k;
    sat_list(end).status = 'trusted';
end
for k = 1:3
    sat_list(end+1).constellation = 'BeiDou';
    sat_list(end).prn    = k;
    sat_list(end).status = 'trusted';
end

cr.sat_list = sat_list;
cr.n_trusted = 10; cr.n_suspect = 0; cr.n_spoofed = 0;

result = apply_exclusion_mask(obs, cr, cfg);

% All weights must equal originals
gps_weights_unchanged = all(abs(result.GPS.weight - obs.GPS.weight) < 1e-15);
ok = gps_weights_unchanged && ~result.insufficient_geometry && result.n_trusted_post_mask == 10;

if ok
    fprintf('  PASS — all 10 weights unchanged, geometry sufficient\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — weights changed or geometry flag wrong\n');
    fprintf('    GPS weights delta: %g\n', max(abs(result.GPS.weight - obs.GPS.weight)));
    fprintf('    insufficient_geometry: %d\n', result.insufficient_geometry);
    fprintf('    n_trusted_post_mask:   %d\n', result.n_trusted_post_mask);
end

%% ---- Test 2: Spoofed GPS PRN 2 — weight inflated by 1e6 ------------------
fprintf('\nTest 2: GPS PRN 2 spoofed - weight must be deflated by 1e6\n');
n_tests = n_tests + 1;

sat_list2 = sat_list;  % copy — modify GPS PRN 2
for k = 1:numel(sat_list2)
    if strcmp(sat_list2(k).constellation,'GPS') && sat_list2(k).prn == 2
        sat_list2(k).status = 'spoofed';
    end
end
cr2.sat_list = sat_list2;
cr2.n_trusted = 9; cr2.n_suspect = 0; cr2.n_spoofed = 1;

result2 = apply_exclusion_mask(obs, cr2, cfg);

expected_spoofed_weight = w_gps / cfg.stage3.spoof_weight_inflation;
prn2_idx = find(result2.GPS.prn == 2);
got_weight = result2.GPS.weight(prn2_idx);

ok2 = abs(got_weight - expected_spoofed_weight) < 1e-20 && result2.n_spoofed_post_mask == 1;

if ok2
    fprintf('  PASS - GPS PRN 2 weight = %.3e (expected %.3e)\n', got_weight, expected_spoofed_weight);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — GPS PRN 2 weight = %.3e (expected %.3e)\n', got_weight, expected_spoofed_weight);
end

%% ---- Test 3: Suspect Galileo PRN 1 — weight deflated by 5 ----------------
fprintf('\nTest 3: Galileo PRN 1 suspect - weight must be deflated by 5\n');
n_tests = n_tests + 1;

sat_list3 = sat_list;
for k = 1:numel(sat_list3)
    if strcmp(sat_list3(k).constellation,'Galileo') && sat_list3(k).prn == 1
        sat_list3(k).status = 'suspect';
    end
end
cr3.sat_list = sat_list3;
cr3.n_trusted = 9; cr3.n_suspect = 1; cr3.n_spoofed = 0;

result3 = apply_exclusion_mask(obs, cr3, cfg);

expected_suspect_weight = w_gal / cfg.stage3.suspect_weight_inflation;
gal_prn1_idx = find(result3.Galileo.prn == 1);
got_w3 = result3.Galileo.weight(gal_prn1_idx);

ok3 = abs(got_w3 - expected_suspect_weight) < 1e-18;

if ok3
    fprintf('  PASS — Galileo PRN 1 weight = %.3e (expected %.3e)\n', got_w3, expected_suspect_weight);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — Galileo PRN 1 weight = %.3e (expected %.3e)\n', got_w3, expected_suspect_weight);
end

%% ---- Test 4: Insufficient geometry after exclusion -----------------------
fprintf('\nTest 4: All GPS spoofed + all non-GPS spoofed -> insufficient_geometry = true\n');
n_tests = n_tests + 1;

% Spoof everything — n_trusted will be 0 < min_sats=5
sat_list4 = struct('constellation',{},'prn',{},'status',{});
for k = 1:4
    sat_list4(end+1).constellation = 'GPS';     sat_list4(end).prn = k; sat_list4(end).status = 'spoofed';
end
for k = 1:3
    sat_list4(end+1).constellation = 'Galileo'; sat_list4(end).prn = k; sat_list4(end).status = 'spoofed';
end
for k = 1:3
    sat_list4(end+1).constellation = 'BeiDou';  sat_list4(end).prn = k; sat_list4(end).status = 'spoofed';
end

cr4.sat_list = sat_list4;
cr4.n_trusted = 0; cr4.n_suspect = 0; cr4.n_spoofed = 10;

result4 = apply_exclusion_mask(obs, cr4, cfg);

ok4 = result4.insufficient_geometry == true && result4.n_trusted_post_mask == 0;

if ok4
    fprintf('  PASS — insufficient_geometry=true, n_trusted=0, no crash\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — insufficient_geometry=%d, n_trusted=%d\n', ...
        result4.insufficient_geometry, result4.n_trusted_post_mask);
end

%% ---- Summary --------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_apply_exclusion_mask: ALL PASS ✓\n\n');
else
    fprintf('test_apply_exclusion_mask: FAILURES PRESENT — review output above\n\n');
end
