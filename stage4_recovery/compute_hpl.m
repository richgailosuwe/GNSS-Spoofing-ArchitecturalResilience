function hpl = compute_hpl(pos_ecef, P_pos_ecef, K_H)
% COMPUTE_HPL  Fault-free horizontal protection level from position covariance.
%
% Computes the horizontal protection level (HPL) as the scaled major
% semi-axis of the horizontal position-error covariance ellipse. This is
% the fault-free (H0) protection level: it bounds the horizontal position
% error given the post-exclusion covariance, to the integrity probability
% encoded by the multiplier K_H.
%
% METHOD (WAAS MOPS / DO-229 horizontal PL structure):
%   1. Compute the receiver geodetic latitude/longitude (WGS84) from the
%      ECEF position, via ecef2lla_simple.
%   2. Rotate the 3x3 ECEF position covariance into the local ENU frame.
%   3. Form the horizontal (East-North) 2x2 sub-block.
%   4. Take the major semi-axis of the error ellipse:
%        d_major = sqrt( (dE2+dN2)/2 + sqrt( ((dE2-dN2)/2)^2 + dEN^2 ) )
%   5. HPL = K_H * d_major.
%
% SCOPE / HONESTY:
%   This is a fault-free protection level only. The full fault-tolerant
%   form HPL = max_j{HPL_j} over fault hypotheses (the ARAIM extension) is
%   NOT implemented; faults are removed by the upstream exclusion stage
%   before this covariance is formed. K_H is a conservative RAIM/NPA-style
%   multiplier and this is NOT a certified RNP AR / SBAS / GBAS compliance
%   implementation.
%
% INPUTS
%   pos_ecef     [3x1] receiver ECEF position [m]
%   P_pos_ecef   [3x3] ECEF position-error covariance [m^2]
%   K_H          (optional) horizontal multiplier. Default 6.18
%                (WAAS MOPS / DO-229 NPA-mode value; conservative).
%
% OUTPUT
%   hpl          scalar horizontal protection level [m]
%
% REFERENCE
%   RTCA DO-229 (WAAS MOPS), horizontal protection level (d_major form);
%   Walter & Enge (1995), Weighted RAIM for Precision Approach.
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery (integrity metric)

    if nargin < 3 || isempty(K_H)
        K_H = 6.18;   % conservative RAIM/NPA-style horizontal PL multiplier
    end

    % --- 1. Geodetic lat/lon (WGS84) for the local ENU frame ---
    [lat, lon, ~] = ecef2lla_simple(pos_ecef);

    % --- 2. ECEF -> ENU rotation (v_enu = R * v_ecef) ---
    sl = sin(lat); cl = cos(lat);
    so = sin(lon); co = cos(lon);
    R = [ -so,      co,     0;
          -sl*co,  -sl*so,  cl;
           cl*co,   cl*so,  sl ];

    % Rotate the position covariance into ENU.
    P_enu = R * P_pos_ecef * R';

    % --- 3. Horizontal (East-North) sub-block ---
    dE2 = P_enu(1,1);
    dN2 = P_enu(2,2);
    dEN = P_enu(1,2);

    % --- 4. Major semi-axis of the horizontal error ellipse ---
    d_major = sqrt( (dE2 + dN2)/2 + sqrt( ((dE2 - dN2)/2)^2 + dEN^2 ) );

    % --- 5. Scaled protection level ---
    hpl = K_H * d_major;
end