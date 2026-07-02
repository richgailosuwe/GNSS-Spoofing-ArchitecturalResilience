function [x_upd, P_upd, update_result] = ekf_update(x_pred, P_pred, ...
                                                      pseudoranges, sat_positions, ...
                                                      weights, gate_result, cfg, ...
                                                      constellations)
% EKF_UPDATE  Stage 4 — EKF update step (measurement update).
%
% State dimension is derived from numel(x_pred), so this serves both the base
% 8-state model and the ISB-extended model (8 + n_isb states).  The
% observation matrix and predicted pseudorange are built by the shared helper
% build_meas_model_isb, the single source of truth shared with ekf_runner.
%
% Only measurements accepted by the scalar innovation gate are incorporated.
% Source: Bar-Shalom, Li & Kirubarajan (2001), Sections 5.4 and 1.4.3.
%
% MEASUREMENT MODEL (per build_meas_model_isb):
%   GPS row:      z = ||sat - pos|| + clk_GPS + noise
%   non-GPS row:  z = ||sat - pos|| + clk_GPS + ISB_c + noise
%   H row places -e_i^T in the position columns, 1 in the clock column (7),
%   and 1 in the relevant ISB column for non-GPS rows.
%
% KALMAN UPDATE (Joseph form):
%   S = H P H' + R ;  K = P H' S^-1 ;  x = x + K v
%   P = (I-KH) P (I-KH)' + K R K'        Source: Groves (2013), 3.2.2.
%
% BACKWARD COMPATIBILITY:
%   Called with 7 arguments (no constellations) and an 8-state x_pred, this
%   behaves exactly as the original single-clock 8-state update: all rows are
%   treated as GPS, no ISB columns exist, and pr_pred = rng + clk.  This keeps
%   test_ekf.m (8-state, minimal cfg) byte-identical.
%
% INPUTS
%   x_pred         [n x 1]  predicted state
%   P_pred         [n x n]  predicted covariance
%   pseudoranges   [m x 1]  corrected pseudoranges [m]
%   sat_positions  [m x 3]  satellite ECEF positions [m]
%   weights        [m x 1]  per-satellite weights (1/sigma^2), after masking
%   gate_result    struct from innovation_gate (scalar): .accepted, .n_accepted, .n_rejected
%   cfg            config struct
%   constellations {m x 1} OPTIONAL cell of labels. If omitted/empty, all GPS.
%
% OUTPUTS
%   x_upd, P_upd, update_result   (fields unchanged from the original API)
%
% STAGE:    4 — EKF Position Recovery

    n_states = numel(x_pred);
    n_meas   = numel(pseudoranges);

    if nargin < 8 || isempty(constellations)
        constellations = repmat({'GPS'}, n_meas, 1);
    end

    %% --- Graceful degradation: no accepted measurements --------------------
    if gate_result.n_accepted == 0
        x_upd   = x_pred;
        P_upd   = P_pred;
        update_result.H          = zeros(0, n_states);
        update_result.innovation = zeros(0, 1);
        update_result.S          = zeros(0);
        update_result.K          = zeros(n_states, 0);
        update_result.n_accepted = 0;
        update_result.n_rejected = n_meas;
        update_result.coasted    = true;
        update_result.pos_update = zeros(3,1);
        update_result.vel_update = zeros(3,1);
        update_result.clk_update = 0;
        return
    end

    %% --- Restrict to accepted measurements ---------------------------------
    acc_idx = find(gate_result.accepted);
    m       = numel(acc_idx);

    sp_acc  = sat_positions(acc_idx, :);
    cn_acc  = constellations(acc_idx);
    pr_acc  = pseudoranges(acc_idx);
    w_acc   = weights(acc_idx);

    %% --- Build H, predicted pseudorange, innovation, R ---------------------
    [H, pr_pred] = build_meas_model_isb(sp_acc, cn_acc, x_pred, cfg);
    v = pr_acc(:) - pr_pred;
    R = diag(1 ./ w_acc(:));

    %% --- Kalman gain --------------------------------------------------------
    S = H * P_pred * H' + R;
    S = 0.5 * (S + S');                       % symmetrise before inversion
    K = P_pred * H' * (S \ eye(size(S)));     % [n_states x m]

    %% --- State update -------------------------------------------------------
    x_upd = x_pred + K * v;

    %% --- Covariance update — Joseph form ------------------------------------
    IKH   = eye(n_states) - K * H;
    P_upd = IKH * P_pred * IKH' + K * R * K';
    P_upd = 0.5 * (P_upd + P_upd');

    %% --- Populate result struct ---------------------------------------------
    update_result.H          = H;
    update_result.innovation = v;
    update_result.S          = S;
    update_result.K          = K;
    update_result.n_accepted = m;
    update_result.n_rejected = n_meas - m;
    update_result.coasted    = false;
    update_result.pos_update = x_upd(1:3) - x_pred(1:3);
    update_result.vel_update = x_upd(4:6) - x_pred(4:6);
    update_result.clk_update = x_upd(7)   - x_pred(7);

end
