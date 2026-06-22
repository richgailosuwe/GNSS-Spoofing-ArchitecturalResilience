function bias_table = calibrate_sat_bias(obs, nav, cfg, epoch_range, constellation)
% CALIBRATE_SAT_BIAS  Per-satellite pseudorange bias calibration (DGNSS-style).
%
% Estimates a per-satellite, per-constellation pseudorange bias correction
% from a window of authentic epochs at a KNOWN receiver position
% (cfg.ref_pos).  This is the local-area DGNSS architecture: a reference
% station at a surveyed location observes the same systematic errors
% (broadcast ephemeris error, ionospheric residual at that pierce point,
% receiver-specific biases) that any nearby rover would experience, and
% broadcasts per-satellite corrections.
%
% Source: Kaplan, E.D. & Hegarty, C.J. (2017). Understanding GPS/GNSS:
% Principles and Applications, 3rd ed. Artech House, Chapter 8 —
% local-area DGNSS, reference-station pseudorange correction.
%
% METHOD:
%   For each epoch in epoch_range:
%     1. Compute WLS position/clock solution using ALL available satellites
%        (uniform weights — the bias itself is what we're estimating,
%        so we don't want to pre-weight by an assumption about it)
%     2. For each satellite, compute post-fit residual:
%          residual = pr_corrected - (geometric_range + clk_bias)
%     3. Accumulate residuals per (constellation, PRN)
%
%   bias_table(prn) = mean(residuals for that PRN across all epochs)
%
% IMPORTANT — WHAT THIS DOES AND DOES NOT CAPTURE:
%   The bias table captures the per-satellite-per-pierce-point systematic
%   error AT THE CALIBRATION LOCATION.  It is NOT a universal satellite
%   property — a different receiver location has different ionospheric
%   pierce points for the same satellite, so the bias table is
%   location-specific.  Re-run this function for each new dataset
%   (different RINEX file = different location and/or time).
%
%   This function does NOT distinguish between ionospheric residual,
%   ephemeris error, and other systematic sources — it captures their SUM
%   as observed at this location during this time window.  Separating
%   these would require dual-frequency ionosphere-free combinations or
%   precise ephemeris comparison, both out of scope.
%
% OVERFITTING WARNING:
%   A bias table calibrated on epochs 1-100 and applied to epochs 1-100
%   will trivially reduce residuals to zero (by construction) and proves
%   nothing.  The bias table MUST be validated on a DIFFERENT epoch range
%   than it was calibrated on.  See test_sat_bias_calibration.m for the
%   calibrate/validate split methodology.
%
% INPUTS
%   obs           struct from rinex_read_obs
%   nav           struct from rinex_read_nav
%   cfg           config struct (cfg.ref_pos, cfg.ekf.meas_noise_*, cfg.identify.min_sats)
%   epoch_range   [1xN] or [Nx1] array of epoch INDICES (not timestamps)
%                 into unique(obs.(constellation).time)
%   constellation char — 'GPS' | 'Galileo' | 'BeiDou' | 'GLONASS'
%
% OUTPUTS
%   bias_table    struct with fields:
%     .constellation   char — the constellation this table applies to
%     .prn             [Mx1] double — satellite PRNs with calibrated bias
%     .bias            [Mx1] double — mean residual [m] per PRN
%                       (SUBTRACT this from pr_corrected to remove the bias:
%                        pr_debiased = pr_corrected - bias_table.bias(idx))
%     .n_samples       [Mx1] double — number of epochs contributing to each PRN
%     .std             [Mx1] double — std of residuals per PRN (informational —
%                       large std relative to bias means the "bias" may be
%                       dominated by noise, not a stable offset)
%     .epoch_range     the input epoch_range (for provenance)
%     .calibration_date char — timestamp of when this was computed
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery (calibration utility)

    noise_map = struct( ...
        'GPS',     cfg.ekf.meas_noise_GPS, ...
        'Galileo', cfg.ekf.meas_noise_Galileo, ...
        'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
        'GLONASS', cfg.ekf.meas_noise_GLONASS);

    if ~isfield(obs, constellation)
        error('calibrate_sat_bias: constellation %s not present in obs', constellation);
    end

    epochs_all = unique(obs.(constellation).time);
    n_req      = numel(epoch_range);

    % Accumulators: PRN -> list of residuals
    resid_accum = containers.Map('KeyType','double','ValueType','any');

    n_used = 0;
    for ii = 1:n_req
        ei = epoch_range(ii);
        if ei < 1 || ei > numel(epochs_all)
            continue
        end
        t_e = epochs_all(ei);

        % Collect all-constellation measurements at this epoch for WLS,
        % but only accumulate residuals for the TARGET constellation.
        % Using all constellations for WLS gives a better position solution
        % (more measurements -> better geometry) even though we only
        % calibrate one constellation's biases at a time.
        all_pr  = [];
        all_sp  = [];
        all_w   = [];
        target_idx_in_all = [];   % indices into all_pr/all_sp that belong to `constellation`
        target_prns        = [];

        constellations = {'GPS','Galileo','BeiDou','GLONASS'};
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs, cname), continue; end
            mask_t = (obs.(cname).time == t_e);
            if ~any(mask_t), continue; end

            prns_e = obs.(cname).prn(mask_t);
            prs_e  = obs.(cname).pseudorange_L1(mask_t);

            for k = 1:numel(prns_e)
                try
                    [sp, sc] = sat_position(nav, prns_e(k), cname, t_e);
                    pr_c     = pseudorange_correct(prs_e(k), sp, sc, cfg.ref_pos, t_e, nav, cname, cfg);
                    if isnan(pr_c), continue; end

                    all_pr(end+1)  = pr_c;          %#ok<AGROW>
                    all_sp(end+1,:) = sp';           %#ok<AGROW>
                    all_w(end+1)   = 1 / noise_map.(cname); %#ok<AGROW>

                    if strcmp(cname, constellation)
                        target_idx_in_all(end+1) = numel(all_pr); %#ok<AGROW>
                        target_prns(end+1)       = prns_e(k);     %#ok<AGROW>
                    end
                catch
                    continue
                end
            end
        end

        if numel(all_pr) < cfg.identify.min_sats || isempty(target_idx_in_all)
            continue
        end

        [pos_wls, clk_wls] = wls_solver(all_pr(:), all_sp, all_w(:), cfg.ref_pos);

        % Compute residuals only for target-constellation satellites.
        for jj = 1:numel(target_idx_in_all)
            idx  = target_idx_in_all(jj);
            prn  = target_prns(jj);
            r_vec = all_sp(idx,:)' - pos_wls;
            rng   = norm(r_vec);
            resid = all_pr(idx) - (rng + clk_wls);

            if ~isKey(resid_accum, prn)
                resid_accum(prn) = [];
            end
            resid_accum(prn) = [resid_accum(prn), resid];
        end

        n_used = n_used + 1;
    end

    %% --- Build output table ---------------------------------------------------
    prns_found = sort(cell2mat(keys(resid_accum)));
    n_prns     = numel(prns_found);

    bias_table.constellation    = constellation;
    bias_table.prn              = prns_found(:);
    bias_table.bias             = zeros(n_prns, 1);
    bias_table.n_samples        = zeros(n_prns, 1);
    bias_table.std              = zeros(n_prns, 1);
    bias_table.epoch_range      = epoch_range;
    bias_table.calibration_date = char(datetime('now'));

    for k = 1:n_prns
        r = resid_accum(prns_found(k));
        bias_table.bias(k)      = mean(r);
        bias_table.n_samples(k) = numel(r);
        bias_table.std(k)       = std(r);
    end

    if cfg.verbose
        fprintf('calibrate_sat_bias: %s, %d/%d epochs used, %d satellites calibrated\n', ...
            constellation, n_used, n_req, n_prns);
    end

end