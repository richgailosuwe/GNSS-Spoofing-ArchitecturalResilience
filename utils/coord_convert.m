%This file centralises all coordinate conversions in one place. Every other
% file that needs to convert between ECEF, geodetic, or local ENU coordinates 
% calls this instead of doing it inline.
function varargout = coord_convert(mode, varargin)
% coord_convert  Coordinate system conversions for GNSS processing.
%
%   Supported modes:
%
%   [lat, lon, alt] = coord_convert('ecef2lla', pos_ecef)
%       ECEF [3x1] metres → geodetic (lat/lon radians, alt metres)
%
%   pos_ecef = coord_convert('lla2ecef', lat, lon, alt)
%       Geodetic (lat/lon radians, alt metres) → ECEF [3x1] metres
%
%   [e, n, u] = coord_convert('ecef2enu', pos_ecef, ref_ecef)
%       ECEF [3x1] → local East-North-Up [3x1] relative to ref_ecef
%
%   dist = coord_convert('ecef_dist', pos1, pos2)
%       Distance between two ECEF positions (metres)
%
%   [lat_deg, lon_deg, alt] = coord_convert('ecef2lla_deg', pos_ecef)
%       ECEF [3x1] metres → geodetic (lat/lon DEGREES, alt metres)

%% ── WGS84 CONSTANTS ──────────────────────────────────────────────────────
a  = 6378137.0;           % semi-major axis (m)
f  = 1/298.257223563;     % flattening
e2 = 2*f - f^2;           % first eccentricity squared
b  = a * (1 - f);         % semi-minor axis (m)

switch lower(mode)

%% ── ECEF → LLA (radians) ─────────────────────────────────────────────────
    case 'ecef2lla'
        pos = varargin{1};
        x = pos(1); y = pos(2); z = pos(3);

        lon = atan2(y, x);
        p   = sqrt(x^2 + y^2);
        lat = atan2(z, p * (1 - e2));

        for i = 1:10
            N       = a / sqrt(1 - e2 * sin(lat)^2);
            lat_new = atan2(z + e2 * N * sin(lat), p);
            if abs(lat_new - lat) < 1e-12, break; end
            lat = lat_new;
        end
        lat = lat_new;
        N   = a / sqrt(1 - e2 * sin(lat)^2);
        alt = p / cos(lat) - N;

        varargout{1} = lat;
        varargout{2} = lon;
        varargout{3} = alt;

%% ── ECEF → LLA (degrees) ─────────────────────────────────────────────────
    case 'ecef2lla_deg'
        pos = varargin{1};
        [lat, lon, alt] = coord_convert('ecef2lla', pos);
        varargout{1} = rad2deg(lat);
        varargout{2} = rad2deg(lon);
        varargout{3} = alt;

%% ── LLA → ECEF ───────────────────────────────────────────────────────────
    case 'lla2ecef'
        lat = varargin{1};
        lon = varargin{2};
        alt = varargin{3};

        N = a / sqrt(1 - e2 * sin(lat)^2);

        X = (N + alt) * cos(lat) * cos(lon);
        Y = (N + alt) * cos(lat) * sin(lon);
        Z = (N * (1 - e2) + alt) * sin(lat);

        varargout{1} = [X; Y; Z];

%% ── ECEF → ENU ───────────────────────────────────────────────────────────
    case 'ecef2enu'
        pos = varargin{1};
        ref = varargin{2};

        % Get reference geodetic coordinates
        [lat0, lon0, ~] = coord_convert('ecef2lla', ref);

        % Difference vector
        dx = pos - ref;

        % Rotation matrix ECEF → ENU
        R = [-sin(lon0),               cos(lon0),              0;
             -sin(lat0)*cos(lon0), -sin(lat0)*sin(lon0),  cos(lat0);
              cos(lat0)*cos(lon0),  cos(lat0)*sin(lon0),  sin(lat0)];

        enu = R * dx;

        varargout{1} = enu(1);  % East
        varargout{2} = enu(2);  % North
        varargout{3} = enu(3);  % Up

%% ── ECEF DISTANCE ────────────────────────────────────────────────────────
    case 'ecef_dist'
        pos1 = varargin{1};
        pos2 = varargin{2};
        varargout{1} = norm(pos1 - pos2);

    otherwise
        error('coord_convert: unknown mode: %s', mode);
end

end

%testing on cmd
% % Test all coord_convert modes
% rec_ecef = cfg.ref_pos;
% 
% % 1 - ECEF to LLA degrees
% [lat_d, lon_d, alt] = coord_convert('ecef2lla_deg', rec_ecef);
% fprintf('[1] ecef2lla_deg: Lat=%.4f Lon=%.4f Alt=%.1fm\n', lat_d, lon_d, alt);
% 
% % 2 - LLA back to ECEF
% rec_back = coord_convert('lla2ecef', deg2rad(lat_d), deg2rad(lon_d), alt);
% fprintf('[2] lla2ecef roundtrip error: %.4f m\n', norm(rec_back - rec_ecef));
% 
% % 3 - ECEF to ENU (should give [0,0,0] for same point)
% [e, n, u] = coord_convert('ecef2enu', rec_ecef, rec_ecef);
% fprintf('[3] ecef2enu self: E=%.6f N=%.6f U=%.6f\n', e, n, u);
% 
% % 4 - ECEF to ENU for WLS solution
% [e2, n2, u2] = coord_convert('ecef2enu', pos_wls, rec_ecef);
% fprintf('[4] WLS error in ENU: E=%.1fm N=%.1fm U=%.1fm\n', e2, n2, u2);
% 
% % 5 - Distance
% d = coord_convert('ecef_dist', pos_wls, rec_ecef);
% fprintf('[5] ecef_dist WLS vs ref: %.1f m\n', d);