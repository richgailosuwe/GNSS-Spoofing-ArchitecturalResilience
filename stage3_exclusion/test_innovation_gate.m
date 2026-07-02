%% TEST_INNOVATION_GATE  Unit tests for innovation_gate.m
%
% Run from project root:
%   run('stage3_exclusion/test_innovation_gate.m')
%
% Expected result: PASS (5/5)
%
% Test design: deterministic inputs with analytically verifiable expected
% outputs.  Scalar mode thresholds are derived from the chi2(dof=1, p=0.999)
% value = 10.8276 (Wilson-Hilferty approximation; exact = 10.8276 from
% chi2inv(0.999,1)).  This is verified externally to 4 s.f.
%
% Reference for threshold:
%   Bar-Shalom, Y., Li, X.R., & Kirubarajan, T. (2001). Estimation with
%   Applications to Tracking and Navigation. Wiley, Section 1.4.3.
%

fprintf('\n=== test_innovation_gate.m ===\n');

cfg.identify.false_alarm_prob = 0.001;  % Parkinson 1988
cfg.stage3.gate_mode          = 'scalar';

% chi2(dof=1, p=0.999) ≈ 10.8276
% A scalar innovation of magnitude sqrt(10.8276 * sigma^2) sits exactly at
% the boundary; anything smaller should be accepted.

n_tests = 0;
n_pass  = 0;

%% ---- Test 1: Small innovation — scalar gate accepts all measurements ------
fprintf('\nTest 1: Small innovations in scalar mode — all must be accepted\n');
n_tests = n_tests + 1;

m   = 4;   % 4 measurements (typical epoch)
v   = [1; 2; -1; 0.5];          % small innovations [m]
H   = [eye(3), ones(3,1); 0 0 1 1];  % rough observation matrix [4x4]
P   = 100 * eye(4);              % liberal prior covariance
R   = diag([333, 333, 333, 333]);% GPS noise variance

gr1 = innovation_gate(v, H, P, R, cfg);

ok1 = all(gr1.accepted) && gr1.n_accepted == m && gr1.n_rejected == 0;

if ok1
    fprintf('  PASS — all %d innovations accepted (mahal=[%s])\n', m, ...
        num2str(gr1.mahal_distances', '%.3f '));
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — n_accepted=%d, n_rejected=%d\n', gr1.n_accepted, gr1.n_rejected);
    fprintf('         mahal = %s\n', mat2str(gr1.mahal_distances', 4));
    fprintf('         threshold = %.4f\n', gr1.threshold);
end

%% ---- Test 2: Large outlier innovation — scalar gate rejects it ------------
fprintf('\nTest 2: One large outlier (spoofed-residual scale) — must be rejected\n');
n_tests = n_tests + 1;

% Build innovation where one element has Mahalanobis >> threshold.
% S_ii for element 1 ≈ H(1,:) * P * H(1,:)' + R(1,1)
% Set v(1) such that v(1)^2 / S_11 >> 10.83.
% Use v(1) = 500m against S_11 ≈ 4*100 + 333 = 733 -> d^2 ≈ 341 >> 10.83

v2    = [500; 2; -1; 0.5];
gr2   = innovation_gate(v2, H, P, R, cfg);

ok2 = ~gr2.accepted(1) && all(gr2.accepted(2:end));

if ok2
    fprintf('  PASS — element 1 rejected (d^2=%.1f >> threshold=%.2f), others accepted\n', ...
        gr2.mahal_distances(1), gr2.threshold);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — accepted = [%s]\n', num2str(gr2.accepted', '%d '));
    fprintf('         mahal    = [%s]\n', num2str(gr2.mahal_distances', '%.1f '));
    fprintf('         threshold = %.4f\n', gr2.threshold);
end

%% ---- Test 3: Joint gate — small innovation accepted -----------------------
fprintf('\nTest 3: Small innovation, joint gate mode — epoch must be accepted\n');
n_tests = n_tests + 1;

cfg3               = cfg;
cfg3.stage3.gate_mode = 'joint';

v3  = [0.5; -0.3; 0.8; 0.1];
gr3 = innovation_gate(v3, H, P, R, cfg3);

% Joint d^2 must be far below chi2(dof=4, p=0.999) ≈ 18.47
ok3 = gr3.epoch_accepted && gr3.mahal_distances <= gr3.threshold;

if ok3
    fprintf('  PASS — epoch accepted (d^2=%.4f <= threshold=%.4f, dof=4)\n', ...
        gr3.mahal_distances, gr3.threshold);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — epoch_accepted=%d, d^2=%.4f, threshold=%.4f\n', ...
        gr3.epoch_accepted, gr3.mahal_distances, gr3.threshold);
end

%% ---- Test 4: Joint gate — full-constellation spoof rejected ---------------
fprintf('\nTest 4: Large joint innovation (full GPS drag-off scale) — epoch must be rejected\n');
n_tests = n_tests + 1;

% Simulate 4 measurements all offset by 591m (Scenario 1 spoofing magnitude
% from test_inter_constellation.m: 591.6m drag-off).
% All innovations large -> joint d^2 >> chi2(4, 0.999)=18.47

v4  = [591.6; 589.3; 592.1; 590.8];
gr4 = innovation_gate(v4, H, P, R, cfg3);  % still joint mode

ok4 = ~gr4.epoch_accepted && gr4.mahal_distances > gr4.threshold;

if ok4
    fprintf('  PASS — epoch rejected (d^2=%.1f >> threshold=%.2f, dof=4)\n', ...
        gr4.mahal_distances, gr4.threshold);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — epoch_accepted=%d, d^2=%.1f, threshold=%.2f\n', ...
        gr4.epoch_accepted, gr4.mahal_distances, gr4.threshold);
end

%% ---- Test 5: REGRESSION — 8-column H (Stage 4 runner path) --------------
% innovation_gate previously asserted H must have 4 columns.
% ekf_runner passes 8-column H (8-state model).
% This test confirms the assertion was generalised correctly.
fprintf('\nTest 5: REGRESSION — 8-column H must not crash innovation_gate\n');
n_tests = n_tests + 1;

m5   = 4;
v5   = [1; 2; -1; 0.5];
H5   = [eye(3), zeros(3,3), ones(3,1), zeros(3,1);
        0 0 1  0 0 0  1  0];   % [4x8]
P5   = 100 * eye(8);
R5   = diag([333, 333, 333, 333]);

cfg5 = cfg;
cfg5.stage3.gate_mode = 'scalar';

try
    gr5  = innovation_gate(v5, H5, P5, R5, cfg5);
    ok5  = all(gr5.accepted) && gr5.n_accepted == m5;
    if ok5
        fprintf('  PASS — 8-column H accepted, all %d innovations gated correctly\n', m5);
        n_pass = n_pass + 1;
    else
        fprintf('  FAIL — ran without crash but wrong gate result\n');
    end
catch ME
    fprintf('  FAIL — crashed with: %s\n', ME.message);
end

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_innovation_gate: ALL PASS ✓\n\n');
else
    fprintf('test_innovation_gate: FAILURES PRESENT — review output above\n\n');
end
