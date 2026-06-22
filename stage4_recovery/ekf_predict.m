function [x_pred, P_pred] = ekf_predict(x, P, cfg)
% EKF_PREDICT  Stage 4 — EKF predict step (time update).
%
% State dimension is derived from numel(x), so the same function serves the
% base 8-state model and the ISB-extended model (8 + n_isb states).
%
% STATE VECTOR (n_states = 8 + n_isb):
%   x(1:3) — receiver position ECEF [m]
%   x(4:6) — receiver velocity ECEF [m/s]
%   x(7)   — receiver GPS clock bias  [m]   (= c x t_bias)
%   x(8)   — receiver clock drift     [m/s] (= c x f_offset / f_nominal)
%   x(9..) — inter-system bias (ISB) states [m], one per non-GPS constellation,
%            ordered by cfg.ekf.isb_order
%
% STATE TRANSITION:
%   pos(k+1)   = pos(k) + dt x vel(k)
%   vel(k+1)   = vel(k)                  (random walk)
%   clk(k+1)   = clk(k) + dt x drift(k)
%   drift(k+1) = drift(k)                (random walk)
%   ISB(k+1)   = ISB(k)                  (random walk — slow hardware/time drift)
%
% Process noise: Q_pos, Q_vel, Q_clk, Q_clk_drift on the base states, and
% Q_isb on each ISB state (only present when n_states > 8).
%
% Justification for constant-velocity model and discrete Q approximation:
%   Groves (2013), Sections 9.4.2 and 3.4.  Joseph-form symmetrisation: 3.2.1.
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery

    dt       = cfg.ekf.dt;     % 30.0 s
    n_states = numel(x);

    %% --- State transition matrix F [n_states x n_states] --------------------
    F          = eye(n_states);
    F(1:3,4:6) = dt * eye(3);   % pos += dt x vel
    F(7,8)     = dt;            % clk += dt x drift
    % ISB block (9:end) is identity (random walk) — already set by eye().

    %% --- Process noise matrix Q [n_states x n_states] -----------------------
    Q      = zeros(n_states);
    Q(1,1) = cfg.ekf.Q_pos;
    Q(2,2) = cfg.ekf.Q_pos;
    Q(3,3) = cfg.ekf.Q_pos;
    Q(4,4) = cfg.ekf.Q_vel;
    Q(5,5) = cfg.ekf.Q_vel;
    Q(6,6) = cfg.ekf.Q_vel;
    Q(7,7) = cfg.ekf.Q_clk;
    Q(8,8) = cfg.ekf.Q_clk_drift;
    if n_states > 8
        for s = 9:n_states
            Q(s,s) = cfg.ekf.Q_isb;    % ISB random-walk process noise
        end
    end

    %% --- Predict ------------------------------------------------------------
    x_pred = F * x;
    P_pred = F * P * F' + Q;

    % Symmetrise to prevent floating-point asymmetry accumulating over epochs.
    % Source: Groves (2013), Section 3.2.1.
    P_pred = 0.5 * (P_pred + P_pred');

end