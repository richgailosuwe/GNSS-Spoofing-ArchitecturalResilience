% this fcn takes the navigation file ephemeris & computes exactly where
% each satellite was in space (ECEF- earth centered earth fixed X,Y,Z coordinates) at any given moment
%helps cal. expected pseudoranges to compare against measured ones
function [sat_pos, sat_clk] = sat_position(nav, prn, constellation, t)
% sat_position  Compute satellite ECEF position and clock correction.
%
%   [sat_pos, sat_clk] = sat_position(nav, prn, constellation, t)
%
%   INPUT:
%     nav           - navigation struct from rinex_read_nav()
%     prn           - satellite PRN number (scalar)
%     constellation - 'GPS', 'Galileo', 'BeiDou', or 'GLONASS' (string)
%     t             - observation time (datetime scalar)
%
%   OUTPUT:
%     sat_pos - [3x1] satellite ECEF position (metres) [X; Y; Z]
%     sat_clk - satellite clock correction (metres)
%               (multiply by speed of light to convert to seconds)
%
%   Returns [NaN; NaN; NaN] and NaN if ephemeris not found.
%
%   Constants follow IS-GPS-200 and Galileo OS-SIS-ICD.

%% ── CONSTANTS ────────────────────────────────────────────────────────────
GM_GPS     = 3.986005e14;    % Earth gravitational constant GPS (m^3/s^2)
GM_GAL     = 3.986004418e14; % Earth gravitational constant Galileo
GM_BDS     = 3.986004418e14; % Earth gravitational constant BeiDou
GM_GLO     = 3.9860044e14;   % Earth gravitational constant GLONASS
OMEGA_E    = 7.2921151467e-5;% Earth rotation rate (rad/s)
C_LIGHT    = 299792458.0;    % Speed of light (m/s)
F          = -4.442807633e-10; % Relativistic correction constant (s/sqrt(m))

%% ── SELECT EPHEMERIS ─────────────────────────────────────────────────────
eph = select_ephemeris(nav.(constellation), prn, t);

if isempty(eph)
    sat_pos = [NaN; NaN; NaN];
    sat_clk = NaN;
    return;
end

%% ── ROUTE BY CONSTELLATION ───────────────────────────────────────────────
switch constellation
    case {'GPS', 'Galileo', 'BeiDou'}
        [sat_pos, sat_clk] = kepler_position(eph, t, constellation, ...
            GM_GPS, OMEGA_E, C_LIGHT, F);
    case 'GLONASS'
        [sat_pos, sat_clk] = glonass_position(eph, t, OMEGA_E, C_LIGHT);
    otherwise
        error('sat_position: unknown constellation: %s', constellation);
end

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: select_ephemeris
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function eph = select_ephemeris(nav_const, prn, t)
% Find the most appropriate ephemeris for this PRN at time t.
% Selects the record with minimum |tk| within a 4-hour validity window.

    if isempty(nav_const.data)
        eph = [];
        return;
    end

    tt   = nav_const.data;
    prns = nav_const.prn;
    toes = nav_const.toe;

    % Filter by PRN
    prn_mask = (prns == prn);
    if ~any(prn_mask)
        eph = [];
        return;
    end

    tt_prn   = tt(prn_mask, :);
    toes_prn = toes(prn_mask);

    % Time difference from each ephemeris epoch
    dt = seconds(t - toes_prn);

    % Select ephemeris with minimum absolute time difference
    % within a 4-hour validity window (14400 seconds)
    abs_dt = abs(dt);
    valid  = abs_dt < 14400;

    if ~any(valid)
        % Fallback — use closest regardless of age
        [~, idx] = min(abs_dt);
    else
        abs_dt_valid = abs_dt;
        abs_dt_valid(~valid) = Inf;
        [~, idx] = min(abs_dt_valid);
    end

    eph = tt_prn(idx, :);

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: kepler_position
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function [pos, clk] = kepler_position(eph, t, constellation, GM, OMEGA_E, C_LIGHT, F)
% Compute satellite position from Keplerian orbital elements.
% Field names verified against BUCU00ROU RINEX navigation file.

    %% ── Select GM by constellation ───────────────────────────────────────
    if strcmp(constellation, 'GPS')
        GM = 3.986005e14;
    else
        GM = 3.986004418e14;
    end

    %% ── Pull ephemeris parameters ────────────────────────────────────────
    A     = eph.sqrtA^2;
    n0    = sqrt(GM / A^3);
    n     = n0 + eph.Delta_n;

    % Time from ephemeris reference epoch
    % Each constellation uses its own time system epoch:
    %   GPS:     1980-01-06 00:00:00 UTC  (IS-GPS-200, Section 3.3.4)
    %   Galileo: 1999-08-22 00:00:00 UTC  (Galileo OS-SIS-ICD, Section 5.1.3)
    %             Note: GST and GPS time differ by a fixed offset (19 leap seconds
    %             at Galileo epoch), but Toe in RINEX nav is already aligned to
    %             GPS week seconds for Galileo — GPS epoch is correct here.
    %   BeiDou:  2006-01-01 00:00:00 UTC  (BDS-SIS-ICD-2.1, Section 4.1)
    %             BDT is offset from GPS time by 14 seconds (GPST = BDT + 14).
    %             Toe in BeiDou RINEX nav is in BDT seconds-of-week, so we must
    %             use the BDT epoch and remove the GPST-BDT offset for tk.
    %             Using GPS epoch here causes tk errors of ~600,000 s — fatal.
    t_utc = datetime(t, 'TimeZone', 'UTC');

    if strcmp(constellation, 'BeiDou')
        % BeiDou Time (BDT) epoch — BDS-SIS-ICD-2.1, Section 4.1.
        % RINEX mixed observation epochs are GPST; BeiDou Toe is BDT SOW.
        bdt_epoch = datetime(2006, 1, 1, 0, 0, 0, 'TimeZone', 'UTC');
        t_sow     = mod(seconds(t_utc - bdt_epoch) - 14, 604800);
    else
        % GPS Time epoch — used for GPS and Galileo (RINEX convention)
        gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
        t_sow     = mod(seconds(t_utc - gps_epoch), 604800);
    end

% Time from ephemeris reference epoch (seconds)
tk = t_sow - eph.Toe;

% Handle week crossover (±half-week boundary)
if tk >  302400, tk = tk - 604800; end
if tk < -302400, tk = tk + 604800; end

    % Mean anomaly
    Mk    = eph.M0 + n * tk;

    %% ── Solve Kepler's equation iteratively ──────────────────────────────
    e  = eph.Eccentricity;
    Ek = Mk;
    for iter = 1:10
        Ek_new = Mk + e * sin(Ek);
        if abs(Ek_new - Ek) < 1e-12, break; end
        Ek = Ek_new;
    end
    Ek = Ek_new;

    %% ── True anomaly ─────────────────────────────────────────────────────
    sin_nu = sqrt(1 - e^2) * sin(Ek) / (1 - e * cos(Ek));
    cos_nu = (cos(Ek) - e)            / (1 - e * cos(Ek));
    nu     = atan2(sin_nu, cos_nu);

    %% ── Argument of latitude ─────────────────────────────────────────────
    phi = nu + eph.omega;

    %% ── Second-order corrections ─────────────────────────────────────────
    du = eph.Cus * sin(2*phi) + eph.Cuc * cos(2*phi);
    dr = eph.Crs * sin(2*phi) + eph.Crc * cos(2*phi);
    di = eph.Cis * sin(2*phi) + eph.Cic * cos(2*phi);

    u  = phi + du;
    r  = A * (1 - e * cos(Ek)) + dr;
    i  = eph.i0 + eph.IDOT * tk + di;

    %% ── Position in orbital plane ────────────────────────────────────────
    x_orb = r * cos(u);
    y_orb = r * sin(u);

    %% ── Corrected longitude of ascending node ────────────────────────────
    if strcmp(constellation, 'BeiDou')
        OMEGA_E_use = 7.2921150e-5;
    else
        OMEGA_E_use = OMEGA_E;
    end

    Omega = eph.OMEGA0 + (eph.OMEGA_DOT - OMEGA_E_use) * tk ...
            - OMEGA_E_use * eph.Toe;

    %% ── ECEF position ────────────────────────────────────────────────────
    pos = [
        x_orb * cos(Omega) - y_orb * cos(i) * sin(Omega);
        x_orb * sin(Omega) + y_orb * cos(i) * cos(Omega);
        y_orb * sin(i)
    ];

    %% ── Satellite clock correction ───────────────────────────────────────
    dt_r = F * e * eph.sqrtA * sin(Ek);
    clk  = (eph.SVClockBias + eph.SVClockDrift * tk + ...
            eph.SVClockDriftRate * tk^2 + dt_r) * C_LIGHT;

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: glonass_position
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function [pos, clk] = glonass_position(eph, t, OMEGA_E, C_LIGHT)
% Compute GLONASS satellite position using numerical integration (RK4).
% Field names verified against BUCU00ROU RINEX navigation file.

    %% ── Initial state from ephemeris ─────────────────────────────────────
    % Position (km -> m) and velocity (km/s -> m/s)
    x0 = [eph.PositionX; eph.PositionY; eph.PositionZ] * 1e3;
    v0 = [eph.VelocityX; eph.VelocityY; eph.VelocityZ] * 1e3;
    a0 = [eph.AccelerationX; eph.AccelerationY; eph.AccelerationZ] * 1e3;

    %% ── Time difference from ephemeris epoch ─────────────────────────────
    % RINEX mixed observation epochs are GPST, while GLONASS broadcast
    % ephemerides are referenced to UTC(SU). For this 2026 dataset,
    % GPST = UTC + 18 s, so convert the observation epoch before propagation.
    t_utc_glo = datetime(t, 'TimeZone', 'UTC') - seconds(18);
    eph_time_utc = datetime(eph.Time, 'TimeZone', 'UTC');
    dt_total = seconds(t_utc_glo - eph_time_utc);

    %% ── Clock correction (GLONASS ICD-2008, Section 3.3.3) ───────────────
    % pr_corr = pr_raw + sat_clk  (pseudorange_correct convention)
    % sat_clk = (+SVClockBias - SVFrequencyBias * dt) * C
    % SVClockBias > 0 means satellite clock is ahead of GLONASS system time,
    % so pseudorange is too short -> add positive correction.
    clk = (eph.SVClockBias - eph.SVFrequencyBias * dt_total) * C_LIGHT;

    %% ── RK4 numerical integration ────────────────────────────────────────
    step = 60; % seconds
    n_steps = ceil(abs(dt_total) / step);
    if n_steps == 0
        pos = x0;
        return;
    end

    h  = dt_total / n_steps;
    xv = [x0; v0];

    for i = 1:n_steps
        k1 = h * glonass_deriv(xv,        a0, OMEGA_E);
        k2 = h * glonass_deriv(xv + k1/2, a0, OMEGA_E);
        k3 = h * glonass_deriv(xv + k2/2, a0, OMEGA_E);
        k4 = h * glonass_deriv(xv + k3,   a0, OMEGA_E);
        xv = xv + (k1 + 2*k2 + 2*k3 + k4) / 6;
    end

    pos = xv(1:3);

end
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: glonass_deriv
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function dxv = glonass_deriv(xv, a_ls, OMEGA_E)
% GLONASS equations of motion for RK4 integration.

    ae    = 6378136.0;   % Earth radius (m)
    mu    = 3.9860044e14;% GM
    J2    = 1.0826257e-3;% second zonal harmonic

    x = xv(1); y = xv(2); z = xv(3);
    r = norm([x; y; z]);

    % Gravitational acceleration with J2 correction
    factor = 1.5 * J2 * (ae/r)^2;
    z_r2   = (z/r)^2;

    ax = -mu/r^3 * x * (1 - factor*(5*z_r2 - 1)) ...
         + OMEGA_E^2 * x + 2*OMEGA_E*xv(5) + a_ls(1);
    ay = -mu/r^3 * y * (1 - factor*(5*z_r2 - 1)) ...
         + OMEGA_E^2 * y - 2*OMEGA_E*xv(4) + a_ls(2);
    az = -mu/r^3 * z * (1 - factor*(5*z_r2 - 3)) + a_ls(3);

    dxv = [xv(4); xv(5); xv(6); ax; ay; az];

end
