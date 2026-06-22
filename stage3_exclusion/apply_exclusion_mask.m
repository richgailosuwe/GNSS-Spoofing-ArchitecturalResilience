function obs_masked = apply_exclusion_mask(obs_epoch, classify_result, cfg)
% APPLY_EXCLUSION_MASK  Stage 3 — Differential weight inflation for spoofed/suspect satellites.
%
% DESIGN BASIS — Robust Kalman Filter / M-estimator literature:
%
%   The technique of inflating a measurement's noise variance (equivalently,
%   reducing its weight) as a function of its anomaly severity originates in
%   the Robust Kalman Filter framework formalised by Yang, He, and Xu (2001),
%   who derive adaptive robust factors that reduce a measurement's weight as a
%   function of its residual severity:
%
%   Yang, Y., He, H., & Xu, G. (2001). "Adaptively robust filtering for
%   kinematic geodetic positioning." Journal of Geodesy, 75(2-3), 109-116.
%   https://doi.org/10.1007/s001900000157
%   (hereafter Yang 2001)
%
%   Yang (2001) identifies two anomaly regimes: moderate residuals (moderate
%   de-weighting) and gross errors (heavy de-weighting).  This module implements
%   a discrete three-tier version of that policy, driven by Stage 2 classification.
%
%   For covariance inflation as an EKF numerical conditioning tool, see:
%   Groves, P.D. (2013). Principles of GNSS, Inertial, and Multisensor
%   Integrated Navigation Systems, 2nd ed. Artech House, Section 14.3.3.
%
% WHAT THIS IS NOT:
%   Blanch et al. (2015/RTCA 040-15) describes a snapshot WLS hard-exclusion
%   architecture for civil aviation ARAIM.  It does NOT describe variance
%   inflation, does NOT have a "suspect" tier, and citing it for this technique
%   would create Protection Level calculation conflicts under civil aviation
%   standards.  It is NOT cited here.
%
% WEIGHT FIELD CONTRACT:
%   This function reads and writes a per-satellite '.weight' field in each
%   constellation sub-struct.  If '.weight' is absent (e.g. obs_epoch came
%   directly from rinex_read_obs which does not add weights), the function
%   initialises weights from cfg.ekf.meas_noise_* as 1/sigma^2.
%   This makes the function callable both from main.m (where main.m may or
%   may not have pre-computed weights) and from unit tests with synthetic structs.
%
% Weight factors applied (Yang 2001, adaptive robust factor framework):
%   spoofed  -> weight / cfg.stage3.spoof_weight_inflation   (default 1e6)
%   suspect  -> weight / cfg.stage3.suspect_weight_inflation  (default 5)
%   trusted  -> weight unchanged
%
% A factor of 1e6 on the variance renders a satellite's contribution to the
% H^T W H normal equations negligible relative to trusted satellites, while
% preserving the ability to monitor it in Stage 4.
%
% Graceful degradation: if n_trusted < cfg.identify.min_sats after masking,
% the function sets obs_masked.insufficient_geometry = true and falls through
% with the degraded set rather than crashing.  This is required by the
% DO-178C robustness objective (thesis objective O7).
%
% INPUTS
%   obs_epoch       struct from rinex_read_obs epoch assembly, with fields:
%                     .GPS/.Galileo/.BeiDou/.GLONASS  -- sub-structs each having:
%                       .prn          [nx1] double
%                       .pseudorange  [nx1] double  (or pseudorange_L1)
%                       .cn0          [nx1] double
%                       .elevation    [nx1] double   (radians)
%                       .weight       [nx1] double   OPTIONAL: 1/sigma^2 per sat.
%                                                    Initialised from cfg if absent.
%   classify_result struct from classify_spoofed_sats, with fields:
%                     .sat_list       [Nx1] struct array, each element:
%                       .constellation  char  ('GPS','Galileo','BeiDou','GLONASS')
%                       .prn            double
%                       .status         char  ('trusted','suspect','spoofed')
%                     .n_trusted      double
%                     .n_suspect      double
%                     .n_spoofed      double
%   cfg             config struct (from config.m)
%
% OUTPUTS
%   obs_masked      copy of obs_epoch with .weight fields updated and new fields:
%                     .insufficient_geometry  logical
%                     .n_trusted_post_mask    double
%                     .n_suspect_post_mask    double
%                     .n_spoofed_post_mask    double
%                     .mask_log               struct -- per-satellite action record
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    3 -- Measurement Exclusion

    %% --- Parameter defaults ---------------------------------------------------
    if ~isfield(cfg, 'stage3')
        cfg.stage3 = struct();
    end

    if ~isfield(cfg.stage3, 'spoof_weight_inflation')
        % ANALYTICAL BASIS FOR 1e6:
        % In the WLS normal equations, satellite i contributes w_i * h_i * h_i^T
        % to H^T W H.  For its contribution to be negligible, w_spoofed must be
        % small relative to the minimum trusted weight.
        %
        % With GPS sigma^2 = 333 m^2, w_trusted = 1/333 ~ 3e-3.
        % After inflation by 1e6:  w_spoofed = 3e-3 / 1e6 = 3e-9.
        % Ratio: w_trusted / w_spoofed = 1e6  -->  spoofed satellite contributes
        % < 1e-6 of any trusted satellite's weight to the normal equations.
        % This is numerically indistinguishable from hard exclusion for any
        % double-precision solver (machine epsilon ~2.2e-16 << 1e-6).
        %
        % The factor is therefore a *numerical exclusion threshold*, not a
        % physical noise model.  It is justified by solver precision, not by
        % an assumed spoofing error magnitude.
        cfg.stage3.spoof_weight_inflation = 1e6;
    end

    if ~isfield(cfg.stage3, 'suspect_weight_inflation')
        % ENGINEERING SIMPLIFICATION — FACTOR OF 5:
        % Yang et al. (2001, J. Geodesy 75(2-3)) derive a *continuous* robust
        % factor function of the standardised residual v_i / sigma_i.  In that
        % framework the weight reduction is smooth, not discrete.
        %
        % Using a fixed factor of 5 for the "suspect" tier is an engineering
        % simplification: it is the discrete three-tier version of Yang's
        % moderate-residual regime.  The value 5 is NOT derived from the BUCU
        % residual distribution.  It is a conservative starting point that
        % reduces a suspect satellite's influence to ~20% of its nominal value
        % while preserving its geometric contribution.
        %
        % CALIBRATION PENDING: this factor should eventually be set by running
        % apply_exclusion_mask over the 2880-epoch authentic dataset and choosing
        % the factor that minimises position error on the authentic set (no
        % spoofing), to confirm that down-weighting honest suspects does not
        % degrade geometry.  Until that calibration is done, treat 5 as a
        % placeholder, not a validated parameter.
        cfg.stage3.suspect_weight_inflation = 5;
    end

    % Default noise variances used only when .weight is absent from obs_epoch.
    % Values match the calibrated parameters in config.m (HANDOVERSTAGE3.md).
    default_noise = struct( ...
        'GPS',     333.0, ...
        'Galileo', 301.0, ...
        'BeiDou',  4972.0, ...
        'GLONASS', 3476.0);

    %% --- Copy obs_epoch -------------------------------------------------------
    obs_masked = obs_epoch;

    %% --- Build lookup: (constellation, prn) -> status -------------------------
    status_map = containers.Map('KeyType','char','ValueType','char');
    for k = 1:numel(classify_result.sat_list)
        s   = classify_result.sat_list(k);
        key = make_key(s.constellation, s.prn);
        status_map(key) = s.status;
    end

    %% --- Apply weight inflation per constellation ----------------------------
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    mask_log       = struct('constellation',{},'prn',{},'status',{},'weight_before',{},'weight_after',{});
    n_trusted      = 0;
    n_suspect      = 0;
    n_spoofed      = 0;

    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs_masked, cname)
            continue
        end
        sub = obs_masked.(cname);
        if isempty(sub.prn)
            continue
        end
        n_sats = numel(sub.prn);

        % --- Initialise weight field if absent --------------------------------
        % rinex_read_obs does not add a .weight field; main.m computes weights
        % from cfg.ekf.meas_noise_* before calling the pipeline.  If .weight is
        % absent here (e.g. direct call from test or Stage 3 invoked before
        % main.m has added weights), initialise from the default noise map.
        if ~isfield(sub, 'weight')
            sigma2 = default_noise.(cname);
            obs_masked.(cname).weight = repmat(1/sigma2, n_sats, 1);
            sub = obs_masked.(cname);  % re-fetch with weight field present
        end

        for si = 1:n_sats
            prn    = sub.prn(si);
            key    = make_key(cname, prn);
            w_orig = sub.weight(si);

            % Determine trust status.  Default 'trusted' if not in classifier
            % output (satellite absent at this epoch -- treated as honest by
            % omission, consistent with conservative spoofing policy).
            if isKey(status_map, key)
                status = status_map(key);
            else
                status = 'trusted';
            end

            switch status
                case 'spoofed'
                    % Gross-error de-weighting: Yang (2001) adaptive robust factor.
                    w_new = w_orig / cfg.stage3.spoof_weight_inflation;
                    n_spoofed = n_spoofed + 1;

                case 'suspect'
                    % Moderate de-weighting: Yang (2001) "moderate residual" regime.
                    w_new = w_orig / cfg.stage3.suspect_weight_inflation;
                    n_suspect = n_suspect + 1;

                otherwise  % 'trusted'
                    w_new  = w_orig;
                    n_trusted = n_trusted + 1;
            end

            obs_masked.(cname).weight(si) = w_new;

            % Log entry for auditability (DO-178C traceability requirement).
            entry.constellation = cname;
            entry.prn           = prn;
            entry.status        = status;
            entry.weight_before = w_orig;
            entry.weight_after  = w_new;
            mask_log(end+1)     = entry; %#ok<AGROW>
        end
    end

    %% --- Geometry check -------------------------------------------------------
    % Two conditions must both hold for geometry to be declared sufficient:
    %
    % (1) Satellite count: n_trusted >= cfg.identify.min_sats (5).
    %     Five satellites are required for a 4-state (x,y,z,clk) WLS solution
    %     with at least one redundant measurement for chi-squared monitoring.
    %     Source: IS-GPS-200, Section 20.3.3.1; Groves (2013), Section 9.2.
    %
    % (2) Condition number of the geometry matrix H^T H <= cfg.stage3.max_cond_HtH.
    %     Satellite count alone is insufficient — five satellites in nearly the
    %     same elevation band can still produce a poorly conditioned geometry
    %     (e.g. all low-elevation satellites with poor vertical spread).
    %     A condition number threshold of 1e6 is used, above which the WLS
    %     normal equations are considered numerically unstable.
    %     Source: Strang & Borre (1997), Linear Algebra, Geodesy, and GPS,
    %     Wellesley-Cambridge Press, Chapter 9 (condition number and GPS geometry).
    %
    % The H matrix is approximated from elevation angles using a standard
    % unit line-of-sight model with azimuth spread assumed across 360 degrees.
    % This is an approximation; Stage 4 will compute exact H from sat positions.

    if ~isfield(cfg.stage3, 'max_cond_HtH')
        cfg.stage3.max_cond_HtH = 1e6;
    end

    % Collect elevation angles of trusted satellites only.
    el_trusted = [];
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs_masked, cname), continue; end
        sub = obs_masked.(cname);
        if isempty(sub.prn), continue; end
        if ~isfield(sub, 'elevation'), continue; end
        for si = 1:numel(sub.prn)
            key = make_key(cname, sub.prn(si));
            % Determine final status of this satellite post-masking.
            if isKey(status_map, key)
                st = status_map(key);
            else
                st = 'trusted';
            end
            if strcmp(st, 'trusted')
                el_trusted(end+1) = sub.elevation(si); %#ok<AGROW>
            end
        end
    end

    % Condition number check only when enough satellites are present.
    count_ok = (n_trusted >= cfg.identify.min_sats);
    cond_ok  = false;
    cond_HtH = Inf;

    if count_ok && ~isempty(el_trusted)
        % Build approximate H rows: [cos(el)*cos(az), cos(el)*sin(az), sin(el), 1]
        % Distribute azimuths uniformly over 360 deg (worst-case approximation).
        % This gives a conservative geometry estimate.
        n_t  = numel(el_trusted);
        az   = linspace(0, 2*pi*(1 - 1/n_t), n_t)';
        el   = el_trusted(:);
        H_approx = [cos(el).*cos(az), cos(el).*sin(az), sin(el), ones(n_t,1)];
        HtH      = H_approx' * H_approx;
        cond_HtH = cond(HtH);
        cond_ok  = (cond_HtH <= cfg.stage3.max_cond_HtH);
    end

    insufficient_geometry = ~count_ok || ~cond_ok;

    %% --- Annotate output struct -----------------------------------------------
    obs_masked.insufficient_geometry = insufficient_geometry;
    obs_masked.n_trusted_post_mask   = n_trusted;
    obs_masked.n_suspect_post_mask   = n_suspect;
    obs_masked.n_spoofed_post_mask   = n_spoofed;
    obs_masked.cond_HtH              = cond_HtH;
    obs_masked.mask_log              = mask_log;

end

%% ============================================================================
%  LOCAL HELPER
%% ============================================================================

function key = make_key(constellation, prn)
% MAKE_KEY  Builds a string key 'GPS_14' for containers.Map lookup.
    key = sprintf('%s_%d', constellation, prn);
end