function gd_metres = get_group_delay(nav, prn, constellation, t)
% GET_GROUP_DELAY  Broadcast group-delay correction for single-frequency users.
%
%   gd_metres = get_group_delay(nav, prn, constellation, t)
%
% Returns the group-delay correction in METRES, as a SIGNED quantity, to be
% SUBTRACTED from the corrected pseudorange by the caller:
%
%       pr_corr = pr_corr - get_group_delay(...)
%
% RATIONALE (IS-GPS-200, Section 20.3.3.3.3.2):
%   The broadcast satellite clock polynomial is referenced to the dual-
%   frequency ionosphere-free combination. A single-frequency L1 user must
%   apply (dt_SV)_L1 = dt_SV - T_GD. Since this pipeline forms the satellite
%   clock as sat_clk = c*dt_SV and uses pr_corr = pr_raw + sat_clk, the L1
%   correction enters as an ADDITIONAL SUBTRACTION of c*T_GD. Hence this
%   function returns c*T_GD (signed) and the caller subtracts it.
%
% PER-CONSTELLATION TERM (mapped to this project's L1 observable, verified
% against the authentic.obs SYS/OBS TYPES header):
%   GPS     C1C = L1 C/A  -> c * TGD                  (column 'TGD')
%   Galileo C1C = E1      -> c * BGD (band-selected)  (see Galileo note)
%   BeiDou  C1P = B1C     -> 0  (see note)            (TGD1/TGD2 are B1I/B2I)
%   GLONASS C1C = G1 C/A  -> 0  (no broadcast group delay in this ephemeris)
%
% GALILEO NOTE (band selection, RINEX 3.02/4.0x):
%   The broadcast clock is referenced to a specific iono-free pair depending
%   on the navigation message source:
%     F/NAV clock -> E5a,E1 pair -> apply BGDE5aE1
%     I/NAV clock -> E5b,E1 pair -> apply BGDE5bE1
%   The source pair is encoded in 'DataSources' (Broadcast Orbit 5), bits 8/9:
%     bit 9 (512) set -> E5b,E1 (I/NAV)  e.g. 0x205=517, 0x201=513, 0x204=516
%     bit 8 (256) set -> E5a,E1 (F/NAV)  e.g. 0x102=258
%   Using a single hardcoded band would misapply ~0.2 m on roughly half the
%   Galileo records in a mixed I/NAV+F/NAV file. The correct term is selected
%   per record from DataSources.
%
% BeiDou NOTE: the tracked observable is B1C (C1P), but the broadcast TGD1/
%   TGD2 parameters are referenced to B1I/B2I. Applying TGD1 to a B1C
%   pseudorange would be the wrong correction, so it is omitted. To enable a
%   correct BeiDou group delay, switch the BeiDou L1 mapping in rinex_read_obs
%   from C1P (B1C) to C2I (B1I); then c*TGD1 becomes correct.
%
% IONOSPHERE-FREE MODE: if the caller forms an L1/L2 iono-free combination,
%   group delay must NOT be applied (the broadcast clock is already IF-
%   referenced). This function is only invoked by corrected_pseudorange in
%   single-frequency (L1) mode.
%
%   Mirrors sat_position/select_ephemeris so the group delay comes from the
%   SAME broadcast record used for satellite position and clock.
%

    C_LIGHT = 299792458.0;
    gd_metres = 0.0;

    nav_const = nav.(constellation);
    if isempty(nav_const.data)
        return;
    end

    % --- Select the same ephemeris row sat_position would use --------------
    % (closest |t - Toe| within a 4-hour validity window; fallback closest)
    prns = nav_const.prn;
    toes = nav_const.toe;
    prn_mask = (prns == prn);
    if ~any(prn_mask)
        return;
    end

    idx_prn   = find(prn_mask);
    toes_prn  = toes(prn_mask);
    dt        = seconds(t - toes_prn);
    abs_dt    = abs(dt);
    valid     = abs_dt < 14400;
    if ~any(valid)
        [~, jrel] = min(abs_dt);
    else
        abs_dt_valid = abs_dt;
        abs_dt_valid(~valid) = Inf;
        [~, jrel] = min(abs_dt_valid);
    end
    row = nav_const.data(idx_prn(jrel), :);
    vars = row.Properties.VariableNames;

    % --- Per-constellation group-delay term (metres) ----------------------
    switch constellation
        case 'GPS'
            if any(strcmp(vars, 'TGD'))
                gd_metres = C_LIGHT * row.TGD(1);
            end

        case 'Galileo'
            % Single-frequency E1 user: select the BGD term whose frequency
            % pair matches the CLOCK SOURCE of this navigation record.
            % bit 9 (512) -> E5b,E1 (I/NAV) -> BGDE5bE1
            % bit 8 (256) -> E5a,E1 (F/NAV) -> BGDE5aE1
            % (See header GALILEO NOTE. Verified vs RINEX 3.02 + authentic.nav.)
            ds_val = NaN;
            if any(strcmp(vars, 'DataSources'))
                ds_val = row.DataSources(1);
            end

            use_e5b = false;
            if ~isnan(ds_val)
                if bitand(uint32(ds_val), 512) ~= 0
                    use_e5b = true;            % I/NAV: E5b,E1 clock
                elseif bitand(uint32(ds_val), 256) ~= 0
                    use_e5b = false;           % F/NAV: E5a,E1 clock
                end
            end

            if use_e5b && any(strcmp(vars, 'BGDE5bE1'))
                gd_metres = C_LIGHT * row.BGDE5bE1(1);
            elseif any(strcmp(vars, 'BGDE5aE1'))
                gd_metres = C_LIGHT * row.BGDE5aE1(1);   % F/NAV or fallback
            end

        case 'BeiDou'
            % B1C observable: TGD1/TGD2 (B1I/B2I) do not apply. Omitted.
            gd_metres = 0.0;

        case 'GLONASS'
            % No broadcast group delay in this ephemeris format.
            gd_metres = 0.0;

        otherwise
            gd_metres = 0.0;
    end

    if isnan(gd_metres)
        gd_metres = 0.0;
    end
end
