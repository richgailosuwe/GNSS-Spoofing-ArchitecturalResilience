function [lat, lon, alt] = ecef2lla_simple(pos)
% ECEF2LLA_SIMPLE  Convert ECEF position to WGS84 geodetic latitude/longitude/altitude.
%
% Converts a position from Earth-Centred Earth-Fixed (ECEF) coordinates
% (X, Y, Z in metres) to WGS84 geodetic coordinates using the iterative
% Bowring method. Returns geodetic latitude and longitude in RADIANS and
% altitude above the WGS84 ellipsoid in metres.
%
% This is an authoritative coordinate-conversion utility used across the
% pipeline (e.g. by compute_hpl.m to build the local ENU frame). Geodetic
% (not geocentric) latitude is required so that the local vertical (Up)
% is normal to the WGS84 ellipsoid, as assumed by the ENU transformation.
%
% INPUT
%   pos   [3x1] or [1x3] ECEF position [x; y; z] in metres
%
% OUTPUT
%   lat   geodetic latitude  [rad]
%   lon   geodetic longitude [rad]
%   alt   altitude above WGS84 ellipsoid [m]
%
% REFERENCE
%   WGS84 datum; Bowring (1976) iterative geodetic latitude solution.
%

    % --- WGS84 ellipsoid constants ---
    a  = 6378137.0;             % semi-major axis [m]
    f  = 1/298.257223563;       % flattening
    e2 = 2*f - f^2;             % first eccentricity squared

    x = pos(1); y = pos(2); z = pos(3);

    lon = atan2(y, x);
    p   = sqrt(x^2 + y^2);      % distance from the Z (rotation) axis

    % Initial geodetic latitude estimate.
    lat = atan2(z, p * (1 - e2));

    % Iterative refinement (Bowring). Update lat at the END of each
    % iteration so the variable always holds the latest estimate, then
    % break once the change falls below tolerance. This avoids relying on
    % a post-loop fix-up and is well defined even if the loop runs once.
    for i = 1:10
        N       = a / sqrt(1 - e2 * sin(lat)^2);   % prime vertical radius
        lat_new = atan2(z + e2 * N * sin(lat), p);
        converged = abs(lat_new - lat) < 1e-12;
        lat = lat_new;
        if converged, break; end
    end

    % Altitude above the ellipsoid.
    N   = a / sqrt(1 - e2 * sin(lat)^2);
    alt = p / cos(lat) - N;
end
