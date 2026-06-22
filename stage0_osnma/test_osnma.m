%% TEST_OSNMA  Unit tests for osnma_status.m
%
% Run from project root:
%   run('stage0_osnma/test_osnma.m')
%
% Expected result: PASS (4/4)
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

fprintf('\n=== test_osnma.m ===\n');
clear

cfg.stage0.mode = 'simulation';

n_tests = 0;
n_pass  = 0;

%% ---- Test 1: No Galileo satellites — no alert, zero counts ---------------
fprintf('\nTest 1: No Galileo satellites — must return no alert, zero counts\n');
n_tests = n_tests + 1;

obs1.GPS.prn = [14; 22; 31];  % GPS only, no Galileo
cfg.stage0.classify = [];

r1 = osnma_status(obs1, cfg);

ok1 = ~r1.osnma_alert && r1.n_authenticated == 0 && ...
       r1.n_unknown == 0 && r1.n_failed == 0;

if ok1
    fprintf('  PASS — no Galileo satellites, no alert, zero counts\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — alert=%d, n_auth=%d, n_unk=%d, n_fail=%d\n', ...
        r1.osnma_alert, r1.n_authenticated, r1.n_unknown, r1.n_failed);
end

%% ---- Test 2: All Galileo trusted — AUTH_OK for all ----------------------
fprintf('\nTest 2: All Galileo trusted — all must return AUTH_OK, no alert\n');
n_tests = n_tests + 1;

obs2.Galileo.prn = [1; 3; 5; 7];

sat_list2 = struct('constellation',{},'prn',{},'status',{});
for k = [1, 3, 5, 7]
    sat_list2(end+1).constellation = 'Galileo';
    sat_list2(end).prn    = k;
    sat_list2(end).status = 'trusted';
end
cfg.stage0.classify.sat_list = sat_list2;

r2 = osnma_status(obs2, cfg);

ok2 = ~r2.osnma_alert && r2.n_authenticated == 4 && r2.n_failed == 0;

if ok2
    fprintf('  PASS — 4/4 AUTH_OK, no alert\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — alert=%d, n_auth=%d, n_fail=%d\n', ...
        r2.osnma_alert, r2.n_authenticated, r2.n_failed);
end

%% ---- Test 3: One Galileo spoofed — AUTH_FAIL, alert raised --------------
fprintf('\nTest 3: One Galileo PRN 3 spoofed — must return AUTH_FAIL and raise alert\n');
n_tests = n_tests + 1;

obs3.Galileo.prn = [1; 3; 5];

sat_list3 = struct('constellation',{},'prn',{},'status',{});
for k = [1, 5]
    sat_list3(end+1).constellation = 'Galileo'; sat_list3(end).prn = k; sat_list3(end).status = 'trusted';
end
sat_list3(end+1).constellation = 'Galileo'; sat_list3(end).prn = 3; sat_list3(end).status = 'spoofed';
cfg.stage0.classify.sat_list = sat_list3;

r3 = osnma_status(obs3, cfg);

prn3_idx = find([r3.sat_status.prn] == 3);
ok3 = r3.osnma_alert && ...
      strcmp(r3.sat_status(prn3_idx).auth_status, 'AUTH_FAIL') && ...
      r3.sat_status(prn3_idx).auth_code == uint8(2) && ...
      r3.n_failed == 1 && r3.n_authenticated == 2;

if ok3
    fprintf('  PASS — PRN 3: AUTH_FAIL (code=%d), alert=true, n_auth=2, n_fail=1\n', ...
        r3.sat_status(prn3_idx).auth_code);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — alert=%d, PRN3 status=%s, n_auth=%d, n_fail=%d\n', ...
        r3.osnma_alert, r3.sat_status(prn3_idx).auth_status, ...
        r3.n_authenticated, r3.n_failed);
end

%% ---- Test 4: No classify provided — all AUTH_UNKNOWN (cold-start model) --
fprintf('\nTest 4: No classify result — all Galileo must return AUTH_UNKNOWN\n');
n_tests = n_tests + 1;

obs4.Galileo.prn = [2; 4; 6];
cfg.stage0.classify = [];   % no Stage 2 output available

r4 = osnma_status(obs4, cfg);

ok4 = ~r4.osnma_alert && r4.n_unknown == 3 && ...
       r4.n_authenticated == 0 && r4.n_failed == 0;

if ok4
    fprintf('  PASS — 3/3 AUTH_UNKNOWN (cold-start), no alert\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — alert=%d, n_unk=%d, n_auth=%d, n_fail=%d\n', ...
        r4.osnma_alert, r4.n_unknown, r4.n_authenticated, r4.n_failed);
end

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_osnma: ALL PASS ✓\n');
    fprintf('\nHARDWARE NOTE: osnma_status runs in SIMULATION MODE only.\n');
    fprintf('ZED-F9P HPG 1.32 does not support OSNMA.\n');
    fprintf('Real OSNMA output requires firmware upgrade to HPG 1.50.\n\n');
else
    fprintf('test_osnma: FAILURES PRESENT — review output above\n\n');
end