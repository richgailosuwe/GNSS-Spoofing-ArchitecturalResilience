function [pr_corr, sat_pos_tx, sat_clk] = corrected_pseudorange( ...
        pr_raw, prn, constellation, t_rx, rec_approx, nav, cfg)
% CORRECTED_PSEUDORANGE  Single owner of the GNSS measurement model.
%
%   [pr_corr, sat_pos_tx, sat_clk] = corrected_pseudorange( ...
%        pr_raw, prn, constellation, t_rx, rec_approx, nav, cfg)
%
% Forms a fully corrected pseudorange and returns the satellite position
% evaluated at SIGNAL TRANSMIT TIME, so callers use geometry consistent with
% the correction. Replaces the previous pattern of calling sat_position at
% RECEPTION time and then pseudorange_correct, which left an uncorrected
% transit-time satellite-motion error of tens of metres per satellite.
%
% MODEL (normal path, rec_approx valid):
%   1. Iterate transmit time on GEOMETRIC range (3 passes):
%        tau   = pr_raw / c               (seed)
%        t_tx  = t_rx - tau
%        sat   = sat_position(t_tx)
%        tau   = ||sat - rec_approx|| / c
%   2. pr_corr = pseudorange_correct(pr_raw, sat_pos_tx, sat_clk, ...)
%        (existing chain: sat clock + Sagnac + iono(Klobuchar) + tropo)
%   3. Single-frequency (L1) only: pr_corr = pr_corr - get_group_delay(...)
%
% COLD START (norm(rec_approx) < 1):
%   No receiver position yet, so transmit-time geometry, Sagnac, elevation,
%   and atmosphere cannot be computed. Evaluate the satellite at RECEPTION
%   time and return a clock-only corrected pseudorange, exactly matching the
%   existing pseudorange_correct early-return behaviour. Group delay is also
%   skipped at cold start (consistent with skipping the rest of the chain).
%
% FREQUENCY MODE:
%   Defaults to single-frequency L1 (this pipeline uses pseudorange_L1).
%   If cfg.receiver.freq_mode exists and equals 'IF', group delay is NOT
%   applied (the broadcast clock is already ionosphere-free referenced).
%   Absence of the field => 'L1' (safe default, never errors).
%
% OUTPUTS:
%   pr_corr     corrected pseudorange (metres), NaN if rejected (e.g. below
%               elevation mask, inherited from pseudorange_correct)
%   sat_pos_tx  [3x1] satellite ECEF position at transmit time (metres)
%   sat_clk     satellite clock correction at transmit time (metres)
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

    C_LIGHT = 299792458.0;

    % --- Frequency mode (default L1, never error on missing field) ---------
    freq_mode = 'L1';
    if isfield(cfg, 'receiver') && isfield(cfg.receiver, 'freq_mode')
        freq_mode = cfg.receiver.freq_mode;
    end

    % --- Cold start: no receiver position -> reception-time, clock only ----
    if norm(rec_approx) < 1
        [sat_pos_tx, sat_clk] = sat_position(nav, prn, constellation, t_rx);
        if any(isnan(sat_pos_tx))
            pr_corr = NaN; return;
        end
        % Clock-only corrected pseudorange (matches pseudorange_correct's
        % early return when no approximate position is available).
        pr_corr = pseudorange_correct(pr_raw, sat_pos_tx, sat_clk, ...
                                      rec_approx, t_rx, nav, constellation, cfg);
        return;
    end

    % --- Normal path: iterate transmit time on geometric range -------------
    tau = pr_raw / C_LIGHT;                 % seed (~0.07 s)
    sat_pos_tx = [NaN; NaN; NaN];
    sat_clk    = NaN;
    for iter = 1:3
        t_tx = t_rx - seconds(tau);
        [sat_pos_tx, sat_clk] = sat_position(nav, prn, constellation, t_tx);
        if any(isnan(sat_pos_tx))
            pr_corr = NaN; return;
        end
        tau = norm(sat_pos_tx - rec_approx) / C_LIGHT;
    end

    % --- Existing correction chain, using the transmit-time satellite ------
    % (sat clock + Sagnac + ionosphere + troposphere; may return NaN if the
    %  satellite is below the elevation mask)
    pr_corr = pseudorange_correct(pr_raw, sat_pos_tx, sat_clk, ...
                                  rec_approx, t_rx, nav, constellation, cfg);
    if isnan(pr_corr)
        return;   % below mask or otherwise rejected — propagate NaN
    end

    % --- Group delay (single-frequency L1 only) ----------------------------
    % Returned in metres, signed; SUBTRACTED (see get_group_delay header).
    if ~strcmpi(freq_mode, 'IF')
        gd = get_group_delay(nav, prn, constellation, t_tx);
        pr_corr = pr_corr - gd;
    end
end