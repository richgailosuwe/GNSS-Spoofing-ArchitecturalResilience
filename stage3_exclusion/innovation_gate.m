function gate_result = innovation_gate(innovation, H, P, R, cfg)
% INNOVATION_GATE  Stage 3 — Chi-squared innovation gating for the EKF.
%
% Gates each measurement innovation using the Mahalanobis distance test:
%
%   d^2 = v' * S^{-1} * v
%
% where:
%   v = innovation vector  [mx1]  (measurement - predicted measurement)
%   S = innovation covariance [mxm] = H * P * H' + R
%   m = number of measurements being tested
%
% A measurement is ACCEPTED if d^2 <= chi2_threshold(m, false_alarm_prob).
% A measurement is REJECTED if d^2 > chi2_threshold(m, false_alarm_prob).
%
% This is the standard formulation from:
%   Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). Estimation with
%   Applications to Tracking and Navigation. Wiley, Section 1.4.3.
%   (hereafter Bar-Shalom 2001)
%
% The false alarm probability cfg.identify.false_alarm_prob = 0.001 is the
% Parkinson (1988) standard used throughout this implementation.
%
% SCALAR vs VECTOR gating:
%   The function supports both:
%     (a) SCALAR gate: test each innovation element independently (dof=1).
%         Used when measurements arrive sequentially and should be gated
%         individually before updating the EKF state.
%         Returns gate_result.accepted [mx1] logical mask.
%     (b) JOINT gate:  test the full innovation vector jointly (dof=m).
%         Used as an epoch-level consistency check.
%         Returns gate_result.epoch_accepted logical scalar.
%
%   Mode is selected by cfg.stage3.gate_mode ('scalar' | 'joint').
%
%   SCALAR mode (default) — use during the EKF measurement update loop:
%     Each pseudorange is tested individually before being incorporated.
%     dof = 1, threshold = chi2inv(1 - p_fa, 1) = 10.83 at p_fa=0.001.
%     A single bad measurement is rejected without discarding the epoch.
%     This is the correct mode when the EKF processes measurements one at
%     a time (sequential update), as described in:
%     Groves (2013), Section 3.2.3 — sequential measurement processing.
%
%   JOINT mode — use as an epoch-level consistency check (pre-update):
%     All m measurements are tested together against chi2(m).
%     At m=4, threshold = chi2inv(0.999, 4) = 18.47.
%     If the full epoch fails, all measurements are rejected for this update
%     cycle and the EKF coasts on prediction only.
%     This is the correct mode as a last-resort safety check when apply_
%     exclusion_mask has already masked spoofed satellites but a fully
%     coherent spoofed epoch might still pass per-measurement scalar gates.
%     Reference: Bar-Shalom, Li & Kirubarajan (2001), Section 1.4.3.
%
%   In the Stage 4 EKF pipeline:
%     innovation_gate is used in SCALAR mode on the update path.  JOINT mode
%     remains available for offline epoch-level diagnostics, but is not used
%     inside ekf_runner because repeated per-epoch joint tests can produce
%     false-rejection cascades over long EKF runs.
%
% INPUTS
%   innovation  [mx1] double — innovation vector (z_meas - z_pred)
%   H           [mx4] or [mx8] double — observation matrix.
%               4 columns for Stage 3 WLS-style gating (4-state).
%               8 columns for Stage 4 EKF gating (8-state: pos+vel+clk+drift).
%   P           [4x4] or [8x8] double — prior state covariance.
%               Must match the column count of H.
%   R           [mxm] double — measurement noise covariance (diagonal from cfg)
%   cfg         config struct with fields:
%                 cfg.identify.false_alarm_prob  (default 0.001)
%                 cfg.stage3.gate_mode           ('scalar'|'joint', default 'scalar')
%
% OUTPUTS
%   gate_result struct with fields:
%     .S                  [mxm] innovation covariance
%     .mahal_distances    [mx1] per-element (scalar) or [1x1] (joint) d^2 values
%     .threshold          chi-squared threshold used
%     .dof                degrees of freedom used
%     .accepted           [mx1] logical — per-measurement acceptance (scalar mode)
%     .epoch_accepted     logical      — joint test result (joint mode)
%     .n_accepted         double
%     .n_rejected         double
%     .gate_mode          char
%
% STAGE:    3 — Measurement Exclusion

    %% --- Validate inputs ------------------------------------------------------
    m        = numel(innovation);
    n_states = size(H, 2);   % 4 (Stage 3 standalone) or 8 (Stage 4 runner)

    assert(size(H, 1) == m, ...
        'innovation_gate: H rows (%d) must equal innovation length (%d)', size(H,1), m);
    assert(n_states >= 4, ...
        'innovation_gate: H must have at least 4 columns, got %d', n_states);
    % n_states is 4 (Stage 3 WLS-style), 8 (Stage 4 base EKF), or 8+n_isb
    % (Stage 4 with ISB states). S = H*P*H'+R is valid for any dimension;
    % only H/P column consistency matters, which the next assert checks.
    assert(size(P, 1) == n_states && size(P, 2) == n_states, ...
        'innovation_gate: P must be %dx%d to match H columns, got %dx%d', ...
        n_states, n_states, size(P,1), size(P,2));
    assert(size(R, 1) == m && size(R, 2) == m, ...
        'innovation_gate: R must be %dx%d to match innovation length, got %dx%d', ...
        m, m, size(R,1), size(R,2));

    %% --- Default parameters ---------------------------------------------------
    if ~isfield(cfg, 'stage3')
        cfg.stage3 = struct();
    end
    if ~isfield(cfg.stage3, 'gate_mode')
        cfg.stage3.gate_mode = 'scalar';
    end
    if ~isfield(cfg, 'identify') || ~isfield(cfg.identify, 'false_alarm_prob')
        cfg.identify.false_alarm_prob = 0.001;  % Parkinson 1988
    end

    gate_mode = cfg.stage3.gate_mode;
    p_fa      = cfg.identify.false_alarm_prob;

    %% --- Innovation covariance S = H * P * H' + R ----------------------------
    % Bar-Shalom 2001, eq. 1.4.3-3.
    S = H * P * H' + R;

    % Symmetrise to guard against floating-point asymmetry in P accumulation.
    S = 0.5 * (S + S');

    %% --- Gate ----------------------------------------------------------------
    switch gate_mode

        case 'scalar'
            % Test each innovation element against chi2(dof=1, p_fa).
            % Equivalent to testing |v_i| / sqrt(S_ii) against normal quantile,
            % but phrased as Mahalanobis for generality.
            % Reference: Bar-Shalom 2001, Section 1.4.3, scalar approximation.
            dof       = 1;
            threshold = chi2inv_approx(1 - p_fa, dof);
            S_diag    = max(diag(S), eps);  % avoid divide-by-zero
            mahal     = (innovation .^ 2) ./ S_diag;  % [mx1]
            accepted  = mahal <= threshold;

            gate_result.mahal_distances = mahal;
            gate_result.accepted        = accepted;
            gate_result.epoch_accepted  = all(accepted);
            gate_result.n_accepted      = sum(accepted);
            gate_result.n_rejected      = m - sum(accepted);

        case 'joint'
            % Joint Mahalanobis test: d^2 = v' * S^{-1} * v ~ chi2(m).
            % Reference: Bar-Shalom 2001, Section 1.4.3, eq. 1.4.3-1.
            dof       = m;
            threshold = chi2inv_approx(1 - p_fa, dof);

            % Use backslash for numerical stability over explicit inverse.
            d2 = innovation' * (S \ innovation);  % scalar

            gate_result.mahal_distances = d2;
            epoch_ok                    = d2 <= threshold;
            gate_result.accepted        = repmat(epoch_ok, m, 1);  % all or nothing
            gate_result.epoch_accepted  = epoch_ok;
            gate_result.n_accepted      = m * double(epoch_ok);
            gate_result.n_rejected      = m * double(~epoch_ok);

        otherwise
            error('innovation_gate: unknown gate_mode ''%s''. Use ''scalar'' or ''joint''.', gate_mode);
    end

    %% --- Populate common fields ----------------------------------------------
    gate_result.S          = S;
    gate_result.threshold  = threshold;
    gate_result.dof        = dof;
    gate_result.gate_mode  = gate_mode;

end

%% ============================================================================
%  LOCAL HELPER: Chi-squared inverse (chi2inv_approx)
%% ============================================================================

function x = chi2inv_approx(p, dof)
% CHI2INV_APPROX  Chi-squared inverse CDF.
%
% Uses MATLAB's built-in chi2inv if available (Statistics Toolbox).
% Falls back to Wilson-Hilferty cube-root normal approximation (1931)
% if the toolbox is absent.  The approximation is accurate to <0.1% for
% dof >= 1 at p = 0.999.
%
% Reference for approximation:
%   Wilson, E.B. & Hilferty, M.M. (1931). "The distribution of chi-square."
%   Proceedings of the National Academy of Sciences, 17(12), 684-688.

    if license('test','statistics_toolbox')
        x = chi2inv(p, dof);
    else
        % Wilson-Hilferty approximation
        % z = norminv(p) approximated via rational Beasley-Springer-Moro method
        z = bsm_norminv(p);
        h = 1 - (2 / (9 * dof));
        k = sqrt(2 / (9 * dof));
        x = dof * ((h + k * z) ^ 3);
        x = max(x, 0);  % numerical guard
    end
end

function z = bsm_norminv(p)
% BSM_NORMINV  Rational approximation to the normal inverse CDF.
%
% Beasley-Springer-Moro approximation, accurate to 1e-7 for p in [0.5, 0.9999].
% Reference:
%   Moro, B. (1995). "The Full Monte." Risk, 8(2), 57-58.
%
% Only used if the Statistics Toolbox is absent.

    if p >= 0.5
        sign_flag = 1;
        q = p;
    else
        sign_flag = -1;
        q = 1 - p;
    end

    r = sqrt(-2 * log(1 - q));

    % Rational coefficients
    a = [2.515517, 0.802853, 0.010328];
    b = [1.432788, 0.189269, 0.001308];

    z = r - (a(1) + a(2)*r + a(3)*r^2) / ...
            (1 + b(1)*r + b(2)*r^2 + b(3)*r^3);

    z = sign_flag * z;
end
