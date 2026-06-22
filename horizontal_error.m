function e_horiz = horizontal_error(pos, ref)
% HORIZONTAL_ERROR  Horizontal EN error from ECEF positions and reference.
%
%   e_horiz = horizontal_error(pos, ref)
%
% INPUTS
%   pos  [n x 3] ECEF positions [m]
%   ref  [3 x 1] or [1 x 3] ECEF reference [m]
%
% OUTPUT
%   e_horiz [n x 1] sqrt(E^2 + N^2) in the local WGS-84 ENU frame [m]
%
% This helper is used when comparing position error with HPL, because HPL
% is horizontal. It deliberately does not use ekf.pos_error, which is the
% full 3D ECEF distance from cfg.ref_pos.

    validateattributes(pos, {'numeric'}, {'2d','ncols',3,'real'}, ...
        mfilename, 'pos', 1);
    validateattributes(ref, {'numeric'}, {'vector','numel',3,'real','finite'}, ...
        mfilename, 'ref', 2);

    ref = ref(:);
    [lat, lon, ~] = ecef2lla_simple(ref);  % geodetic radians

    R_ecef_to_enu = [ ...
        -sin(lon),             cos(lon),            0; ...
        -sin(lat)*cos(lon),   -sin(lat)*sin(lon),   cos(lat); ...
         cos(lat)*cos(lon),    cos(lat)*sin(lon),   sin(lat)];

    delta_ecef = pos - ref';
    enu = (R_ecef_to_enu * delta_ecef')';
    e_horiz = vecnorm(enu(:,1:2), 2, 2);
end
