function pr_corr = pseudorange_correct(pr_raw, sat_pos, sat_clk, rec_pos_approx, t, nav, constellation, cfg)
% pseudorange_correct  Apply corrections to raw pseudorange measurements.
%
%   pr_corr = pseudorange_correct(pr_raw, sat_pos, sat_clk,
%                                 rec_pos_approx, t, nav,
%                                 constellation, cfg)
%
%   INPUT:
%     pr_raw          - raw pseudorange (metres)
%     sat_pos         - satellite ECEF position [3x1] (metres)
%     sat_clk         - satellite clock correction (metres)
%     rec_pos_approx  - approximate receiver ECEF position [3x1] (metres)
%                       (use [0;0;0] for first iteration)
%     t               - observation epoch (datetime)
%     nav             - navigation struct from rinex_read_nav()
%     constellation   - 'GPS', 'Galileo', 'BeiDou', or 'GLONASS'
%     cfg             - configuration struct from config.m
%
%   OUTPUT:
%     pr_corr - corrected pseudorange (metres)
%
%   Corrections applied:
%     1. Satellite clock correction
%     2. Earth rotation correction (Sagnac effect) — SCALAR range term
%     3. Ionospheric delay (Klobuchar model) — GPS/GLONASS
%        or NeQuick approximation           — Galileo/BeiDou
%     4. Tropospheric delay (Saastamoinen model)
%
% =========================================================================
% BUG FIX HISTORY — Sagnac correction (this revision):
%
% PREVIOUS IMPLEMENTATION (incorrect):
%   The previous version rotated sat_pos by theta = Omega_E * signal_time
%   to produce sat_pos_corr, computed geo_range = norm(sat_pos_corr -
%   rec_pos_approx), and then DISCARDED both sat_pos_corr and geo_range.
%   The returned pr_corr never included any Sagnac term, while the
%   geometric range used downstream by wls_solver/ekf_runner was computed
%   from the UNROTATED sat_pos. The Sagnac effect (~10-40m for GPS MEO
%   satellites, satellite-position-dependent) was therefore never applied
%   anywhere in the pipeline. This was identified as the dominant
%   contributor to the 50-100m position error found in Stage 4 validation
%   (see HANDOVERSTAGE4.md, Sagnac investigation).
%
% CURRENT IMPLEMENTATION (fixed):
%   Applies the standard SCALAR Sagnac range correction directly to the
%   pseudorange, using the existing (uncorrected) sat_pos and
%   rec_pos_approx. No rotation matrix, no sat_pos_corr, nothing for
%   downstream callers to propagate -- the correction is folded entirely
%   into pr_corr, exactly like the clock/iono/tropo corrections.
%
%   Formula (Kaplan & Hegarty, 2017, "Understanding GPS/GNSS: Principles
%   and Applications", 3rd ed., Artech House, Section 5.4.1, Eq. 5.13):
%
%     dt_sagnac = (Omega_E / c) * (x_sat * y_rec - y_sat * x_rec)
%     pr_corr   = pr_corr - c * dt_sagnac
%               = pr_corr - (Omega_E / c) * (x_sat*y_rec - y_sat*x_rec)
%
%   This term represents the Sagnac (Earth-rotation) contribution to the
%   apparent range.  It is SUBTRACTED from the pseudorange here -- the
%   opposite of the satellite clock term -- because the Sagnac quantity
%   as conventionally defined represents an addition to the GEOMETRIC
%   RANGE MODEL (the rotating-to-inertial frame transformation effectively
%   lengthens the modeled range computed from a non-rotated sat_pos by
%   this amount for satellites east of the receiver).  To compensate the
%   MEASURED pseudorange so that it matches a geometric range computed
%   from the unrotated sat_pos, the term must be removed from pr_corr,
%   i.e. subtracted.
%
%   SIGN VERIFIED EMPIRICALLY (see test_sagnac_fix.m, HANDOVERSTAGE4.md):
%   an initial implementation with '+' increased mean position error from
%   68.9m to 90.1m over 50 epochs; '-' is the corrected sign. Any future
%   modification to this term MUST be re-validated against
%   test_sagnac_fix.m Test 3 (mean position error vs RTKLIB baseline)
%   before being accepted -- the sign is easy to get backwards and the
%   test result is the ground truth, not the formula derivation.
%
%   MAGNITUDE: for GPS MEO orbits (|sat_pos| ~ 26,000 km), this term is
%   typically ~10-40 m depending on satellite azimuth -- non-negligible
%   and satellite-position-dependent, which explains why the previous
%   bug produced satellite-specific biases of the magnitude observed
%   (PRN19 ~+40m, PRN15 ~-29m, etc.) rather than a uniform offset.
% =========================================================================

%% ── CONSTANTS ────────────────────────────────────────────────────────────
C_LIGHT = 299792458.0;   % speed of light (m/s)
OMEGA_E = 7.2921151467e-5;  % Earth rotation rate (rad/s), WGS84

%% ── VALIDATE INPUT ───────────────────────────────────────────────────────
if isnan(pr_raw) || pr_raw <= 0
    pr_corr = NaN;
    return;
end

if any(isnan(sat_pos))
    pr_corr = NaN;
    return;
end

%% ── 1. SATELLITE CLOCK CORRECTION ────────────────────────────────────────
% Add satellite clock error to pseudorange. PR_corrected = PR + c*dt_sv
pr_corr = pr_raw + sat_clk;

%% ── 2. EARTH ROTATION CORRECTION (Sagnac effect) — SCALAR RANGE TERM ─────
% Applied only once rec_pos_approx is available (see geometric range guard
% below) -- the term depends on rec_pos.  For the bootstrap case
% (rec_pos_approx ~ 0), this is skipped along with atmospheric corrections,
% consistent with the existing early-return structure.
if norm(rec_pos_approx) >= 1
    sagnac_term = (OMEGA_E / C_LIGHT) * ...
                  (sat_pos(1) * rec_pos_approx(2) - sat_pos(2) * rec_pos_approx(1));
    pr_corr = pr_corr - sagnac_term;
end

%% ── 3. GEOMETRIC RANGE GUARD ─────────────────────────────────────────────
if norm(rec_pos_approx) < 1
    % No approximate position yet — skip atmospheric corrections.
    % (Sagnac term above was also skipped for the same reason.)
    return;
end

%% ── 4. ELEVATION ANGLE ───────────────────────────────────────────────────
% Needed for atmospheric corrections.
% NOTE: elevation is computed from the UNROTATED sat_pos. The Sagnac
% displacement (~100m) is negligible for elevation-angle purposes (it
% changes elevation by a fraction of a degree at most), so using sat_pos
% directly here (rather than a rotated copy) is an acceptable
% approximation consistent with how elevation is used only for masking
% and as an input to the tropospheric mapping function.
elev = compute_elevation(rec_pos_approx, sat_pos);

% Skip atmospheric corrections for satellites below elevation mask
if elev < deg2rad(cfg.receiver.elev_mask)
    pr_corr = NaN;
    return;
end

%% ── 5. IONOSPHERIC CORRECTION (Klobuchar model) ──────────────────────────
iono_delay = klobuchar_model(rec_pos_approx, sat_pos, t, nav, constellation);
pr_corr = pr_corr - iono_delay;

%% ── 6. TROPOSPHERIC CORRECTION (Saastamoinen model) ─────────────────────
tropo_delay = saastamoinen_model(rec_pos_approx, elev);
pr_corr = pr_corr - tropo_delay;

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: compute_elevation
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function elev = compute_elevation(rec_pos, sat_pos)
% Compute satellite elevation angle from receiver position.
% Returns elevation in radians.

    % Vector from receiver to satellite
    los = sat_pos - rec_pos;

    % Receiver unit normal (up direction in ECEF)
    rec_norm = rec_pos / norm(rec_pos);

    % Elevation = angle between LOS and local horizontal plane
    sin_elev = dot(los, rec_norm) / norm(los);
    elev     = asin(max(-1, min(1, sin_elev)));

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: klobuchar_model
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function iono = klobuchar_model(rec_pos, sat_pos, t, nav, constellation)
% Klobuchar ionospheric correction model.
% IS-GPS-200 Section 20.3.3.5.2.5
%
% Uses alpha/beta coefficients from GPS navigation message.
% For Galileo/BeiDou, falls back to GPS coefficients as approximation.

    %% ── Try to get Klobuchar coefficients from nav ───────────────────────
    alpha = [0; 0; 0; 0];
    beta  = [0; 0; 0; 0];

    if isfield(nav, 'GPS') && isfield(nav.GPS, 'data') && ...
       ~isempty(nav.GPS.data)
        vars = nav.GPS.data.Properties.VariableNames;
        % Coefficient names vary by RINEX version
        if any(strcmp(vars, 'Alpha0'))
            row = nav.GPS.data(1,:);
            alpha = [row.Alpha0; row.Alpha1; row.Alpha2; row.Alpha3];
            beta  = [row.Beta0;  row.Beta1;  row.Beta2;  row.Beta3];
        end
    end

    %% ── Receiver geodetic coordinates ────────────────────────────────────
    [lat, lon, ~] = ecef2lla(rec_pos);
    lat_u = lat / pi;   % semi-circles
    lon_u = lon / pi;

    %% ── Satellite elevation and azimuth ──────────────────────────────────
    los      = sat_pos - rec_pos;
    los_norm = los / norm(los);
    rec_norm = rec_pos / norm(rec_pos);

    sin_elev = dot(los_norm, rec_norm);
    elev_sc  = asin(max(-1,min(1,sin_elev))) / pi; % semi-circles

    % Azimuth
    east  = [-sin(lon);  cos(lon);  0];
    north = [-sin(lat)*cos(lon); -sin(lat)*sin(lon); cos(lat)];
    az    = atan2(dot(los_norm, east), dot(los_norm, north));

    %% ── Earth-centred angle ──────────────────────────────────────────────
    psi = 0.0137 / (elev_sc + 0.11) - 0.022;

    %% ── Subionospheric latitude / longitude ──────────────────────────────
    lat_i = lat_u + psi * cos(az);
    lat_i = max(-0.416, min(0.416, lat_i));
    lon_i = lon_u + psi * sin(az) / cos(lat_i * pi);

    %% ── Geomagnetic latitude ─────────────────────────────────────────────
    lat_m = lat_i + 0.064 * cos((lon_i - 1.617) * pi);

    %% ── Local time ───────────────────────────────────────────────────────
    t_sec = second(t) + minute(t)*60 + hour(t)*3600;
    lt    = mod(4.32e4 * lon_i + t_sec, 86400);

    %% ── Period and amplitude ─────────────────────────────────────────────
    AMP = alpha(1) + alpha(2)*lat_m + alpha(3)*lat_m^2 + alpha(4)*lat_m^3;
    AMP = max(0, AMP);

    PER = beta(1) + beta(2)*lat_m + beta(3)*lat_m^2 + beta(4)*lat_m^3;
    PER = max(72000, PER);

    %% ── Ionospheric delay (seconds) ──────────────────────────────────────
    x = 2 * pi * (lt - 50400) / PER;

    F = 1 + 16 * (0.53 - elev_sc)^3;  % slant factor

    if abs(x) >= 1.57
        T_iono = F * 5e-9;
    else
        T_iono = F * (5e-9 + AMP * (1 - x^2/2 + x^4/24));
    end

    % Convert to metres
    iono = T_iono * 299792458.0;

end


%% LOCAL HELPER: saastamoinen_model
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function tropo = saastamoinen_model(rec_pos, elev)
% Saastamoinen tropospheric correction model.
% Uses standard atmosphere — no meteorological data required.

    %% ── Receiver height above ellipsoid ──────────────────────────────────
    [~, ~, alt] = ecef2lla(rec_pos);
    alt = max(0, alt); % clip to sea level minimum

    %% ── Standard atmosphere parameters at height h ───────────────────────
    % Pressure (hPa)
    P = 1013.25 * (1 - 2.2557e-5 * alt)^5.2568;

    % Temperature (K)
    T = 288.15 - 6.5e-3 * alt;

    % Partial pressure of water vapour (hPa) — standard humidity
    e = 6.108 * exp(17.15 * (T - 273.15) / (T - 38.65));
    e = 0.5 * e; % assume 50% relative humidity

    %% ── Zenith delays ────────────────────────────────────────────────────
    % Dry (hydrostatic) zenith delay
    d_dry = 0.002277 * P / (1 - 0.00266 * cos(2 * atan2(norm(rec_pos(1:2)), rec_pos(3))) ...
            - 0.00028 * alt / 1000);

    % Wet zenith delay
    d_wet = 0.002277 * (1255/T + 0.05) * e;

    %% ── Map to slant delay using elevation ───────────────────────────────
    % Simple 1/sin(elev) mapping function
    if elev < deg2rad(2)
        elev = deg2rad(2); % floor at 2 degrees
    end

    tropo = (d_dry + d_wet) / sin(elev);

end


%% LOCAL HELPER: ecef2lla
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function [lat, lon, alt] = ecef2lla(pos)
% Convert ECEF (metres) to geodetic lat/lon (radians) and altitude (metres).
% Bowring iterative method.

    a  = 6378137.0;          % WGS84 semi-major axis (m)
    f  = 1/298.257223563;    % WGS84 flattening
    e2 = 2*f - f^2;          % first eccentricity squared

    x = pos(1); y = pos(2); z = pos(3);

    lon = atan2(y, x);
    p   = sqrt(x^2 + y^2);
    lat = atan2(z, p * (1 - e2));

    % Iterate
    for i = 1:10
        N      = a / sqrt(1 - e2 * sin(lat)^2);
        lat_new = atan2(z + e2 * N * sin(lat), p);
        if abs(lat_new - lat) < 1e-12, break; end
        lat = lat_new;
    end
    lat = lat_new;

    N   = a / sqrt(1 - e2 * sin(lat)^2);
    alt = p / cos(lat) - N;

end
