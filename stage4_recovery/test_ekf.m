%% TEST_EKF  Unit tests for ekf_predict.m and ekf_update.m — 8-state model
%
% Run from project root:
%   run('stage4_recovery/test_ekf.m')
%
% Expected result: PASS (5/5)
%
% All expected values are analytically derived from the state transition
% and Kalman update equations — not fed back from numerical results.
%
% STATE LAYOUT: [x, y, z, vx, vy, vz, clk_bias, clk_drift]
%               indices 1-3, 4-6, 7, 8
%


fprintf('\n=== test_ekf.m (8-state) ===\n');
%% --- Minimal cfg -----------------------------------------------------------
cfg.ekf.dt           = 30.0;
cfg.ekf.Q_pos        = 0.01;
cfg.ekf.Q_vel        = 0.01;
cfg.ekf.Q_clk        = 0.10;
cfg.ekf.Q_clk_drift  = 1e-6;
cfg.ekf.meas_noise_GPS = 333.0;
cfg.identify.false_alarm_prob = 0.001;
cfg.stage3.gate_mode = 'scalar';

ref_pos  = [4093761.206; 2007793.576; 4445129.764];  % BUCU ECEF

n_tests = 0;
n_pass  = 0;

%% ---- Test 1: ekf_predict — position propagated by velocity ---------------
% x = [ref_pos; [1;0;0]; 100; -0.02454]  (1 m/s eastward velocity)
% After predict:
%   pos(k+1) = pos(k) + 30 × [1;0;0] = ref_pos + [30;0;0]
%   clk(k+1) = 100 + 30×(-0.02454) = 99.2638 m
%   vel and drift unchanged
fprintf('\nTest 1: ekf_predict — pos += dt*vel, clk += dt*drift\n');
n_tests = n_tests + 1;

vel0 = [1; 0; 0];
x0   = [ref_pos; vel0; 100.0; -0.02454];
P0   = diag([456;456;456; 100;100;100; 1e6; 1.6e-4]);

[x_p, P_p] = ekf_predict(x0, P0, cfg);

expected_pos = ref_pos + 30.0 * vel0;
expected_clk = 100.0 + 30.0 * (-0.02454);  % = 99.2638 m

pos_ok   = norm(x_p(1:3) - expected_pos) < 1e-9;
vel_ok   = norm(x_p(4:6) - vel0)         < 1e-12;
clk_ok   = abs(x_p(7) - expected_clk)    < 1e-9;
drift_ok = abs(x_p(8) - (-0.02454))      < 1e-12;
P_grew   = P_p(7,7) > P0(7,7);

ok1 = pos_ok && vel_ok && clk_ok && drift_ok && P_grew;
if ok1
    fprintf('  PASS — pos correct, clk=%.6f (exp %.6f), P grew\n', x_p(7), expected_clk);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — pos_ok=%d vel_ok=%d clk_ok=%d drift_ok=%d P_grew=%d\n', ...
        pos_ok, vel_ok, clk_ok, drift_ok, P_grew);
end

%% ---- Test 2: ekf_predict — Q added to P correctly -----------------------
% With P0=0, P_pred = F*0*F^T + Q = Q.
% Diagonal of P_pred must equal diagonal of Q.
fprintf('\nTest 2: ekf_predict — process noise Q correctly populates P from zero prior\n');
n_tests = n_tests + 1;

[~, P_from_zero] = ekf_predict(zeros(8,1), zeros(8), cfg);

q_pos_ok   = abs(P_from_zero(1,1) - cfg.ekf.Q_pos)       < 1e-15;
q_vel_ok   = abs(P_from_zero(4,4) - cfg.ekf.Q_vel)       < 1e-15;
q_clk_ok   = abs(P_from_zero(7,7) - cfg.ekf.Q_clk)       < 1e-15;
q_drift_ok = abs(P_from_zero(8,8) - cfg.ekf.Q_clk_drift) < 1e-20;

ok2 = q_pos_ok && q_vel_ok && q_clk_ok && q_drift_ok;
if ok2
    fprintf('  PASS — P diag: pos=%.4e vel=%.4e clk=%.4e drift=%.2e\n', ...
        P_from_zero(1,1), P_from_zero(4,4), P_from_zero(7,7), P_from_zero(8,8));
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — P diag = %s\n', mat2str(diag(P_from_zero)', 4));
    fprintf('         Q diag = [%.4e %.4e %.4e %.2e]\n', ...
        cfg.ekf.Q_pos, cfg.ekf.Q_vel, cfg.ekf.Q_clk, cfg.ekf.Q_clk_drift);
end

%% ---- Test 3: ekf_update — zero innovation leaves state unchanged --------
% Predicted pseudorange exactly matches measured — v=0 — state must not change.
% P must decrease (information was gained).
fprintf('\nTest 3: ekf_update — zero innovation: state unchanged, P decreases\n');
n_tests = n_tests + 1;

sat_pos3 = [26559000; 0; 0]';
clk_true = 50.0;
rng_true = norm(sat_pos3' - ref_pos);
pr_true  = rng_true + clk_true;

x_pred3 = [ref_pos; zeros(3,1); clk_true; -0.02454];
P_pred3 = diag([456;456;456; 100;100;100; 1e6; 1.6e-4]);

gr3.accepted   = true;
gr3.n_accepted = 1;
gr3.n_rejected = 0;

[x_u3, P_u3, ur3] = ekf_update(x_pred3, P_pred3, pr_true, sat_pos3, 1/333, gr3, cfg);

state_unchanged = norm(x_u3 - x_pred3) < 1e-6;
P_decreased     = trace(P_u3) < trace(P_pred3);

ok3 = state_unchanged && P_decreased && ~ur3.coasted;
if ok3
    fprintf('  PASS — state unchanged (delta=%.2e), trace(P): %.4e -> %.4e\n', ...
        norm(x_u3 - x_pred3), trace(P_pred3), trace(P_u3));
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — state_unchanged=%d (delta=%.4e), P_decreased=%d\n', ...
        state_unchanged, norm(x_u3 - x_pred3), P_decreased);
end

%% ---- Test 4: ekf_update — position offset corrected toward truth --------
% Predicted position is 10m offset in X. EKF must pull it toward ref_pos.
fprintf('\nTest 4: ekf_update — 10m position offset corrected toward truth\n');
n_tests = n_tests + 1;

pos_offset = ref_pos + [10; 0; 0];
x_pred4    = [pos_offset; zeros(3,1); clk_true; -0.02454];
P_pred4    = diag([456;456;456; 100;100;100; 1e6; 1.6e-4]);

pr4 = norm(sat_pos3' - ref_pos) + clk_true;  % pseudorange from true position

gr4.accepted   = true;
gr4.n_accepted = 1;
gr4.n_rejected = 0;

[x_u4, ~, ur4] = ekf_update(x_pred4, P_pred4, pr4, sat_pos3, 1/333, gr4, cfg);

err_before = norm(x_pred4(1:3) - ref_pos);
err_after  = norm(x_u4(1:3)   - ref_pos);
ok4 = (err_after < err_before) && ~ur4.coasted;

if ok4
    fprintf('  PASS — pos error: %.4f m -> %.4f m (reduced)\n', err_before, err_after);
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — pos error: %.4f m -> %.4f m (should decrease)\n', err_before, err_after);
end

%% ---- Test 5: ekf_update — no accepted measurements, coasting -----------
fprintf('\nTest 5: ekf_update — no accepted measurements returns coasted=true\n');
n_tests = n_tests + 1;

x_pred5 = [ref_pos; zeros(3,1); clk_true; -0.02454];
P_pred5 = diag([456;456;456; 100;100;100; 1e6; 1.6e-4]);

gr5.accepted   = false(3,1);
gr5.n_accepted = 0;
gr5.n_rejected = 3;

[x_u5, P_u5, ur5] = ekf_update(x_pred5, P_pred5, ...
    [pr_true; pr_true+100; pr_true+200], ...
    repmat(sat_pos3, 3, 1), repmat(1/333, 3, 1), gr5, cfg);

ok5 = ur5.coasted && ...
      norm(x_u5 - x_pred5) < 1e-15 && ...
      norm(P_u5 - P_pred5, 'fro') < 1e-10;

if ok5
    fprintf('  PASS — coasted=true, state and P unchanged from prediction\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL — coasted=%d, state_delta=%.4e, P_delta=%.4e\n', ...
        ur5.coasted, norm(x_u5-x_pred5), norm(P_u5-P_pred5,'fro'));
end

%% ---- Summary ---------------------------------------------------------------
fprintf('\n--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_ekf: ALL PASS ✓\n\n');
else
    fprintf('test_ekf: FAILURES PRESENT — review output above\n\n');
end