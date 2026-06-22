function ekf_out = ekf_runner(obs, nav, classify_results, cfg)
% EKF_RUNNER  Stage 4 — Full epoch loop: predict → mask → gate → update.
%
% Runs the EKF over all epochs in obs, using Stage 2 classification results
% to mask spoofed measurements before each update. State dimension is
% 8 + n_isb (one inter-system bias state per enabled non-GPS constellation).
%
% EPOCH ALIGNMENT:
%   Epoch 1: WLS bootstrap solution stored directly as output.
%             No predict step — there is no prior state to propagate from.
%   Epoch 2+: standard predict → mask → gate → update cycle.
%
%   This avoids the off-by-one error of predicting from the epoch-1 WLS
%   state into epoch 1 measurements (which would apply 30s of process noise
%   before the first measurement update).
%
% INITIALISATION:
%   Position and clock bias initialised from WLS at epoch 1.
%   cfg.ref_pos is used as the WLS linearisation point (numerical starting
%   point for the iterative solver only — not asserted as the true position).
%   For a fielded receiver without a known survey position, replace with
%   [0;0;0] and allow more WLS iterations to converge.
%   Velocity initialised to zero. Clock drift from cfg.ekf.clk_drift_init
%   (calibrated from 100 authentic BUCU epochs). ISB states initialised to
%   zero with cfg.ekf.P_init_isb uncertainty.
%
% PIPELINE PER EPOCH (epochs 2+):
%   1. ekf_predict          — propagate state and covariance by dt
%   2. Assemble obs_epoch   — collect corrected pseudoranges + sat positions
%   3. apply_exclusion_mask — apply Stage 2 trust weights
%   4. innovation_gate (scalar) — isolate individual bad measurements
%   5. ekf_update           — Kalman update on accepted measurements only
%
% CONSTELLATIONS:
%   All four constellations (GPS, Galileo, BeiDou, GLONASS) used when
%   available. Measurement noise per constellation from cfg.ekf.meas_noise_*.
%   Inter-system bias modelled per non-GPS constellation (cfg.ekf.isb_order).
%
% INPUTS
%   obs               struct from rinex_read_obs
%   nav               struct from rinex_read_nav
%   classify_results  cell array {n_epochs x 1} of classify_spoofed_sats
%                     output structs, one per epoch.  Pass {} or [] to run
%                     in authentic mode (all satellites trusted).
%   cfg               config struct
%
% OUTPUTS
%   ekf_out   struct with fields:
%               .epochs      [Ex1] datetime — epoch timestamps
%               .pos         [Ex3] ECEF position estimates [m]
%               .clk_bias    [Ex1] clock bias estimates [m]
%               .clk_drift   [Ex1] clock drift estimates [m/s]
%               .pos_error   [Ex1] norm distance from cfg.ref_pos [m]
%               .P_trace     [Ex1] trace of position covariance [m²]
%               .hpl         [Ex1] horizontal protection level [m]
%               .n_accepted  [Ex1] measurements accepted per epoch
%               .n_rejected  [Ex1] measurements rejected per epoch
%               .coasted     [Ex1] logical — epoch coasted on prediction
%               .epoch_log   [Ex1] struct array — full per-epoch detail
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery

    %% --- Setup --------------------------------------------------------------
    epochs       = unique(obs.GPS.time);
    n_epochs     = numel(epochs);
    use_classify = ~isempty(classify_results);

    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    noise_map = struct( ...
        'GPS',     cfg.ekf.meas_noise_GPS, ...
        'Galileo', cfg.ekf.meas_noise_Galileo, ...
        'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
        'GLONASS', cfg.ekf.meas_noise_GLONASS);

    %% --- Pre-allocate outputs -----------------------------------------------
    out_pos       = nan(n_epochs, 3);
    out_clk_bias  = nan(n_epochs, 1);
    out_clk_drift = nan(n_epochs, 1);
    out_pos_err   = nan(n_epochs, 1);
    out_P_trace   = nan(n_epochs, 1);
    out_hpl       = nan(n_epochs, 1);   % horizontal protection level [m]
    out_n_acc     = zeros(n_epochs, 1);
    out_n_rej     = zeros(n_epochs, 1);
    out_coasted   = false(n_epochs, 1);
    out_exclusion_fallback = false(n_epochs, 1);
    epoch_log     = cell(n_epochs, 1);

    %% --- Epoch 1: WLS bootstrap — no predict step --------------------------
    % The EKF has no prior state to propagate from at epoch 1.
    % Store the WLS solution directly as the epoch-1 output.
    % Predict is NOT called — calling predict before epoch-1 measurements
    % would add 30s of process noise to a state that has never been updated,
    % shifting the state forward in time before the first observation.
    %
    % Linearisation point: cfg.ref_pos is used as the numerical starting
    % point for the iterative WLS solver only.  For a receiver without a
    % known survey position, [0;0;0] or a coarse almanac position would be
    % used instead.  This is a known limitation of the current offline
    % validation implementation and is noted in the thesis.
    % Source: Groves (2013), Section 9.4.1.

    [pos_init, clk_init] = bootstrap_wls(obs, nav, epochs(1), constellations, noise_map, cfg);

    if isnan(pos_init(1))
        warning('ekf_runner: WLS bootstrap failed at epoch 1 — falling back to cfg.ref_pos');
        pos_init = cfg.ref_pos(:);
        clk_init = 0.0;
    end

    % State: [x, y, z, vx, vy, vz, clk_GPS, clk_drift, ISB_1..ISB_nisb]
    n_isb    = cfg.ekf.n_isb;
    x = [pos_init(:); ...
         zeros(3,1); ...                  % velocity: zero cold-start
         clk_init; ...                    % GPS clock bias: from WLS epoch 1
         cfg.ekf.clk_drift_init; ...      % clock drift: calibrated from BUCU
         zeros(n_isb,1)];                 % ISB states: zero cold-start

    P = diag([cfg.ekf.P_init_pos  * ones(3,1); ...
              cfg.ekf.P_init_vel  * ones(3,1); ...
              cfg.ekf.P_init_clk; ...
              cfg.ekf.P_init_drift; ...
              cfg.ekf.P_init_isb  * ones(n_isb,1)]);

    % Store epoch 1 output directly from WLS.
    out_pos(1,:)    = x(1:3)';
    out_clk_bias(1) = x(7);
    out_clk_drift(1)= x(8);
    out_pos_err(1)  = norm(x(1:3) - cfg.ref_pos);
    out_P_trace(1)  = trace(P(1:3,1:3));
    out_hpl(1)      = compute_hpl(x(1:3), P(1:3,1:3), cfg.integrity.K_H);
    out_n_acc(1)    = 0;   % WLS bootstrap: no EKF update counted
    epoch_log{1}    = struct('bootstrap', true, 'pos_init', pos_init, 'clk_init', clk_init);

    %% --- Epoch loop: epochs 2 onward ----------------------------------------
    for ei = 2:n_epochs
        t_e = epochs(ei);

        %% Step 1: Predict ---------------------------------------------------
        [x_pred, P_pred] = ekf_predict(x, P, cfg);

        %% Step 2: Assemble obs_epoch ----------------------------------------
        obs_epoch = struct();
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs, cname), continue; end
            mask_t = (obs.(cname).time == t_e);
            if ~any(mask_t), continue; end

            prns_e = obs.(cname).prn(mask_t);
            prs_e  = obs.(cname).pseudorange_L1(mask_t);
            cn0_e  = obs.(cname).cn0(mask_t);

            sub.prn        = [];
            sub.pseudorange = [];
            sub.cn0        = [];
            sub.elevation  = [];
            sub.weight     = [];
            sat_pos_sub    = [];

            for k = 1:numel(prns_e)
                try
                    % Transmit-time corrected measurement. Returns sat_pos at
                    % TRANSMIT time, used below for the EKF geometry so range
                    % and correction are consistent. rec_approx = x_pred(1:3).
                    [pr_corr, sp] = corrected_pseudorange(prs_e(k), prns_e(k), ...
                                        cname, t_e, x_pred(1:3), nav, cfg);
                    if isnan(pr_corr), continue; end

                    % Elevation mask already applied inside corrected_pseudorange.
                    sub.prn(end+1)         = prns_e(k);
                    sub.pseudorange(end+1) = pr_corr;
                    sub.cn0(end+1)         = cn0_e(k);
                    sub.weight(end+1)      = 1 / noise_map.(cname);
                    sat_pos_sub(end+1,:)   = sp';
                catch
                    continue
                end
            end

            % Store column vectors.
            obs_epoch.(cname).prn         = sub.prn(:);
            obs_epoch.(cname).pseudorange = sub.pseudorange(:);
            obs_epoch.(cname).cn0         = sub.cn0(:);
            obs_epoch.(cname).weight      = sub.weight(:);
            obs_epoch.(cname).elevation   = zeros(numel(sub.prn),1); % placeholder
            obs_epoch.(cname).sat_pos     = sat_pos_sub;
        end

        %% Step 3: Apply exclusion mask --------------------------------------
        if use_classify && ei <= numel(classify_results) && ...
                ~isempty(classify_results{ei})
            cr = classify_results{ei};
        else
            % Authentic mode: build trivial all-trusted classify result.
            cr = build_trusted_classify(obs_epoch, constellations);
        end
        obs_masked = apply_exclusion_mask(obs_epoch, cr, cfg);

        %% Step 4: Assemble flat measurement vectors --------------------------
        % all_w      : post-mask weights (Stage 2/3 exclusion applied)
        % all_w_orig : original nominal weights from obs_epoch (pre-mask).
        %              Used by the insufficient-geometry fallback below so
        %              the scalar gate sees nominal R, not a 1e6-inflated R
        %              that would blind it (all-spoofed collapse case).
        all_pr    = [];
        all_sp    = [];
        all_w     = [];
        all_w_orig = [];
        all_const = {};   % constellation label per measurement row (for ISB)
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs_masked, cname), continue; end
            sub = obs_masked.(cname);
            if isempty(sub.prn), continue; end
            n_sub     = numel(sub.pseudorange(:));
            all_pr    = [all_pr;    sub.pseudorange(:)];          %#ok<AGROW>
            all_w     = [all_w;     sub.weight(:)];               %#ok<AGROW>
            all_sp    = [all_sp;    sub.sat_pos];                 %#ok<AGROW>
            all_const = [all_const; repmat({cname}, n_sub, 1)];   %#ok<AGROW>
            % Original nominal weight for the same row, from obs_epoch.
            all_w_orig = [all_w_orig; obs_epoch.(cname).weight(:)]; %#ok<AGROW>
        end

        %% Step 4b: Insufficient-geometry fallback --------------------------
        % apply_exclusion_mask sets insufficient_geometry=true when fewer
        % than min_sats trusted satellites remain (e.g. the all-spoofed
        % classification collapse under 2-vs-2 inter-constellation
        % ambiguity in dual-constellation attacks). In that state the
        % masked weights are degenerate (every weight deflated by 1e6),
        % which both lets the contaminated set leak into the EKF and
        % blinds the scalar gate (R inflated, Mahalanobis distances shrink).
        %
        % 'all satellites spoofed' here is an INDETERMINATE classifier
        % state (no trusted consensus), not a literal physical claim.
        % Operational response (policy cfg.stage3.insufficient_geometry_policy):
        %   'gate_only' (default): discard the degenerate mask, restore
        %                nominal weights, let the scalar gate protect
        %                position per-measurement against the filter's
        %                own prediction (proven to recover at collapse epochs).
        %   'coast'   : skip the measurement update (stricter integrity mode).
        exclusion_fallback = false;
        fallback_reason    = '';
        ig_policy = 'gate_only';
        if isfield(cfg,'stage3') && isfield(cfg.stage3,'insufficient_geometry_policy')
            ig_policy = cfg.stage3.insufficient_geometry_policy;
        end
        % TRIGGER: use the trusted-satellite COUNT, not obs_masked.
        % insufficient_geometry. The latter also folds in a condition-number
        % check that, inside ekf_runner, is computed from PLACEHOLDER-zero
        % elevations (line ~186) and would therefore fire on every epoch.
        % The count test (n_trusted_post_mask < min_sats) is the clean,
        % elevation-independent signal of the all-spoofed classification
        % collapse we are guarding against.
        n_trusted_pm = numel(all_pr);   % default: all rows present
        if isfield(obs_masked,'n_trusted_post_mask')
            n_trusted_pm = obs_masked.n_trusted_post_mask;
        end
        if n_trusted_pm < cfg.identify.min_sats
            exclusion_fallback = true;
            fallback_reason    = 'insufficient_trusted_geometry_after_classification';
            if strcmp(ig_policy,'gate_only')
                % Restore nominal weights so the gate works on nominal R.
                all_w = all_w_orig;
            end
        end

        n_meas = numel(all_pr);
        if n_meas < cfg.identify.min_sats
            % Insufficient measurements — coast.
            x      = x_pred;
            P      = P_pred;
            out_coasted(ei)  = true;
            out_n_rej(ei)    = n_meas;
            out_pos(ei,:)    = x(1:3)';
            out_clk_bias(ei) = x(7);
            out_clk_drift(ei)= x(8);
            out_pos_err(ei)  = norm(x(1:3) - cfg.ref_pos);
            out_P_trace(ei)  = trace(P(1:3,1:3));
            out_hpl(ei)      = compute_hpl(x(1:3), P(1:3,1:3), cfg.integrity.K_H);
            continue
        end

        %% Step 5: Scalar innovation gate — per-measurement gating -----------
        % DESIGN DECISION: joint gate removed from the EKF update path.
        %
        % A joint chi²(m) gate at p_fa=0.001 over m=36 measurements has
        % probability 1-(1-0.001)^200 = 18% of falsely rejecting at least
        % one authentic epoch over a 200-epoch run.  A single false rejection
        % triggers a coasting cascade: P grows each predict step from Q
        % accumulation, innovations grow, and every subsequent epoch is
        % rejected.  This was confirmed empirically: epoch 106 was rejected
        % on authentic data, causing P to grow from 114 m² to 19,100 m²
        % by epoch 200 with zero measurements accepted.
        %
        % The scalar gate (dof=1 per measurement, chi²(1)=10.83 at p=0.001)
        % is the correct choice for sequential EKF processing.  Each
        % measurement is tested independently; a single bad measurement is
        % rejected without discarding the epoch.
        % Source: Groves (2013), Section 3.2.3.
        %
        % The joint gate remains available in innovation_gate.m for offline
        % diagnostic use (epoch-level consistency checking in post-processing)
        % but must not be on the EKF update critical path.

        % Build measurement model with inter-system bias (ISB) support.
        % Non-GPS rows get their ISB column set in H and their ISB state added
        % to the predicted pseudorange, so the innovation is ISB-consistent.
        % GPS-only runs (n_states==8) reduce to the original single-clock model.
        [H_flat, pr_pred] = build_meas_model_isb(all_sp, all_const, x_pred, cfg);
        v_flat = all_pr(:) - pr_pred;
        R_flat = diag(1 ./ all_w(:));

        % Scalar innovation gate (per-measurement). Can be bypassed for
        % ablation studies via cfg.stage3.use_innovation_gate = false, which
        % accepts ALL measurements (no residual rejection). Default true.
        use_gate = true;
        if isfield(cfg,'stage3') && isfield(cfg.stage3,'use_innovation_gate')
            use_gate = cfg.stage3.use_innovation_gate;
        end
        if exclusion_fallback && strcmp(ig_policy,'coast')
            % COAST policy: indeterminate classification -> skip update.
            % Build an accept-nothing gate; ekf_update will coast.
            m_meas = numel(v_flat);
            gr_scalar = struct( ...
                'S',               H_flat*P_pred*H_flat' + R_flat, ...
                'mahal_distances', zeros(m_meas,1), ...
                'threshold',       0, ...
                'dof',             1, ...
                'accepted',        false(m_meas,1), ...
                'epoch_accepted',  false, ...
                'n_accepted',      0, ...
                'n_rejected',      m_meas, ...
                'gate_mode',       'coast_fallback');
        elseif use_gate
            cfg_scalar = cfg;
            cfg_scalar.stage3.gate_mode = 'scalar';
            gr_scalar = innovation_gate(v_flat, H_flat, P_pred, R_flat, cfg_scalar);
        else
            % ABLATION: gate disabled - accept every measurement.
            m_meas = numel(v_flat);
            gr_scalar = struct( ...
                'S',               H_flat*P_pred*H_flat' + R_flat, ...
                'mahal_distances', zeros(m_meas,1), ...
                'threshold',       Inf, ...
                'dof',             1, ...
                'accepted',        true(m_meas,1), ...
                'epoch_accepted',  true, ...
                'n_accepted',      m_meas, ...
                'n_rejected',      0, ...
                'gate_mode',       'disabled');
        end

        %% Step 6: EKF update ------------------------------------------------
        [x_upd, P_upd, ur] = ekf_update(x_pred, P_pred, ...
                                         all_pr, all_sp, all_w, gr_scalar, cfg, all_const);

        x = x_upd;
        P = P_upd;

        % Record fallback status for auditability (thesis traceability).
        ur.exclusion_fallback = exclusion_fallback;
        ur.fallback_reason    = fallback_reason;
        out_exclusion_fallback(ei) = exclusion_fallback;

        out_pos(ei,:)    = x(1:3)';
        out_clk_bias(ei) = x(7);
        out_clk_drift(ei)= x(8);
        out_pos_err(ei)  = norm(x(1:3) - cfg.ref_pos);
        out_P_trace(ei)  = trace(P(1:3,1:3));
        out_hpl(ei)      = compute_hpl(x(1:3), P(1:3,1:3), cfg.integrity.K_H);
        out_n_acc(ei)    = ur.n_accepted;
        out_n_rej(ei)    = ur.n_rejected;
        out_coasted(ei)  = ur.coasted;
        epoch_log{ei}    = ur;
    end

    %% --- Package output ----------------------------------------------------
    ekf_out.epochs     = epochs;
    ekf_out.pos        = out_pos;
    ekf_out.clk_bias   = out_clk_bias;
    ekf_out.clk_drift  = out_clk_drift;
    ekf_out.pos_error  = out_pos_err;
    ekf_out.P_trace    = out_P_trace;
    ekf_out.hpl        = out_hpl;    % horizontal protection level per epoch [m]
    ekf_out.n_accepted = out_n_acc;
    ekf_out.n_rejected = out_n_rej;
    ekf_out.coasted    = out_coasted;
    ekf_out.exclusion_fallback = out_exclusion_fallback;
    ekf_out.epoch_log  = epoch_log;

end

%% ============================================================================
%  LOCAL HELPERS
%% ============================================================================

function cr = build_trusted_classify(obs_epoch, constellations)
% BUILD_TRUSTED_CLASSIFY  Returns an all-trusted classify result for authentic mode.
    sat_list = struct('constellation',{},'prn',{},'status',{});
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs_epoch, cname), continue; end
        for k = 1:numel(obs_epoch.(cname).prn)
            sat_list(end+1).constellation = cname;  %#ok<AGROW>
            sat_list(end).prn    = obs_epoch.(cname).prn(k);
            sat_list(end).status = 'trusted';
        end
    end
    cr.sat_list  = sat_list;
    cr.n_trusted = numel(sat_list);
    cr.n_suspect = 0;
    cr.n_spoofed = 0;
end

function [pos_wls, clk_wls] = bootstrap_wls(obs, nav, t_e, constellations, noise_map, cfg)
% BOOTSTRAP_WLS  Compute a single-epoch WLS solution for EKF initialisation.
%
% Uses all available measurements at epoch t_e with all-trusted weights.
% Single-clock WLS (no ISB): this only provides the epoch-1 starting
% position. The ISB states start at zero and converge in the main epoch
% loop. Returns [NaN; NaN; NaN] and 0 if fewer than cfg.identify.min_sats
% measurements are available.
%
% This ensures ekf_runner initialises from an observable position estimate
% rather than the known reference position — a requirement for any receiver
% that does not have prior knowledge of its location.
%
% Source: Groves (2013), Section 9.4.1.

    all_pr = [];
    all_sp = [];
    all_w  = [];

    % Linearisation point for WLS solver.
    % cfg.ref_pos is used here as the iterative WLS starting point because
    % this is an offline validation implementation with a known survey position.
    % A fielded receiver would use a coarse approximate position from one of:
    %   - a previous fix stored in non-volatile memory
    %   - an almanac-based position estimate
    %   - a survey-provided approximate coordinate
    % Using Earth-centre [0;0;0] is NOT appropriate here because
    % pseudorange_correct requires a physically meaningful receiver position
    % for elevation mask and tropospheric correction computation.
    % This limitation is noted in the thesis (Section 4.6, implementation scope).
    pos_lin = cfg.ref_pos(:);

    for ci = 1:numel(constellations)
        cname  = constellations{ci};
        if ~isfield(obs, cname), continue; end
        mask_t = (obs.(cname).time == t_e);
        if ~any(mask_t), continue; end

        prns_e = obs.(cname).prn(mask_t);
        prs_e  = obs.(cname).pseudorange_L1(mask_t);

        for k = 1:numel(prns_e)
            try
                % Transmit-time corrected measurement; rec_approx = pos_lin
                % (cfg.ref_pos cold-start linearization point).
                [pr_corr, sp] = corrected_pseudorange(prs_e(k), prns_e(k), ...
                                    cname, t_e, pos_lin, nav, cfg);
                if isnan(pr_corr), continue; end
                all_pr(end+1)   = pr_corr;    %#ok<AGROW>
                all_sp(end+1,:) = sp';         %#ok<AGROW>
                all_w(end+1)    = 1 / noise_map.(cname);  %#ok<AGROW>
            catch
                continue
            end
        end
    end

    if numel(all_pr) < cfg.identify.min_sats
        pos_wls = [NaN; NaN; NaN];
        clk_wls = 0;
        return
    end

    [pos_wls, clk_wls] = wls_solver(all_pr(:), all_sp, all_w(:), pos_lin);
end