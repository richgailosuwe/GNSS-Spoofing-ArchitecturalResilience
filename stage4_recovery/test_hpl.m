function test_hpl()
% TEST_HPL  Test suite for compute_hpl.m (Stage 4 integrity metric).
%
% Run from project root after config:
%   config
%   run('stage4_recovery/test_hpl.m')   % or wherever compute_hpl lives
%
% Written BEFORE compute_hpl.m per the project debugging protocol: every
% test defines a property that must hold, and the suite FAILS until the
% function is implemented correctly. Tests do not depend on the EKF; they
% feed compute_hpl known covariances with analytically-known answers.
%
% TRUTH ANCHORS (independent of the function under test):
%   BUCU geodetic coordinates were computed by an independent reference
%   implementation from the BUCU ECEF [4093761.206; 2007793.576; 4445129.764]:
%       lat = 44.463942216 deg,  lon = 26.125736143 deg,  alt = 143.206 m
%   These are used to check ecef2lla_simple without re-using its own logic.
%
% CONSTANT UNDER TEST:
%   K_H = 6.18  (conservative RAIM/NPA-style horizontal PL multiplier,
%                WAAS MOPS / DO-229 NPA value; NOT a certified precision-
%                approach or RNP AR allocation).
%
% STAGE:    4 — EKF Position Recovery (integrity metric)

    fprintf('\n=== TEST_HPL ===\n');
    n_pass = 0; n_fail = 0;

    K_H      = 6.18;        % must match compute_hpl
    BUCU     = [4093761.206; 2007793.576; 4445129.764];
    lat_true = deg2rad(44.463942216);
    lon_true = deg2rad(26.125736143);
    alt_true = 143.206;

    % ------------------------------------------------------------------
    % TEST 0 — ecef2lla_simple recovers the independent BUCU truth anchor
    %          (guards against a rotation/coordinate bug being verified
    %           against itself later).
    % ------------------------------------------------------------------
    [lat, lon, alt] = ecef2lla_simple(BUCU);
    ok_lat = abs(lat - lat_true) < 1e-7;     % ~1e-7 rad ~ 0.6 mm of arc
    ok_lon = abs(lon - lon_true) < 1e-7;
    ok_alt = abs(alt - alt_true) < 1e-2;     % 1 cm
    if ok_lat && ok_lon && ok_alt
        fprintf('  [PASS] Test 0: ecef2lla_simple matches BUCU truth anchor\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] Test 0: lat err=%.3e rad, lon err=%.3e rad, alt err=%.3e m\n', ...
            abs(lat-lat_true), abs(lon-lon_true), abs(alt-alt_true));
        n_fail = n_fail + 1;
    end

    % ------------------------------------------------------------------
    % TEST 1 — Isotropic horizontal covariance.
    %   If the position-error covariance is isotropic with horizontal
    %   sigma s in the LOCAL frame, then d_east=d_north=s, d_EN=0, so
    %   d_major = s and HPL = K_H * s exactly.
    %   We build the covariance in ENU and rotate it INTO ECEF so that
    %   compute_hpl must rotate it back. Truth is known: HPL = K_H * s.
    % ------------------------------------------------------------------
    s = 3.0;                                  % horizontal sigma [m]
    sigma_up = 5.0;                           % vertical sigma (must not affect HPL)
    P_enu = diag([s^2, s^2, sigma_up^2]);
    R = enu_rotation(lat_true, lon_true);     % ENU = R * ECEF  =>  P_enu = R*P_ecef*R'
    P_ecef = R' * P_enu * R;                  % rotate ENU-defined cov into ECEF
    hpl = compute_hpl(BUCU, P_ecef);
    expected = K_H * s;
    if abs(hpl - expected) < 1e-6
        fprintf('  [PASS] Test 1: isotropic -> HPL = K_H*s = %.4f m\n', hpl);
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] Test 1: HPL=%.6f, expected %.6f\n', hpl, expected);
        n_fail = n_fail + 1;
    end

    % ------------------------------------------------------------------
    % TEST 2 — Known anisotropic ENU covariance (off-diagonal present).
    %   Construct a 2x2 EN covariance with known eigen-structure, embed in
    %   3x3 ENU, rotate to ECEF, and check compute_hpl recovers the exact
    %   major semi-axis. Truth d_major computed here independently.
    % ------------------------------------------------------------------
    dE2 = 9.0; dN2 = 4.0; dEN = 1.5;          % [m^2]
    d_major_true = sqrt( (dE2+dN2)/2 + sqrt( ((dE2-dN2)/2)^2 + dEN^2 ) );
    P_enu2 = [dE2,  dEN, 0;
              dEN,  dN2, 0;
              0,    0,   25];
    P_ecef2 = R' * P_enu2 * R;
    hpl2 = compute_hpl(BUCU, P_ecef2);
    expected2 = K_H * d_major_true;
    if abs(hpl2 - expected2) < 1e-6
        fprintf('  [PASS] Test 2: anisotropic -> HPL = K_H*d_major = %.4f m (d_major=%.4f)\n', ...
            hpl2, d_major_true);
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] Test 2: HPL=%.6f, expected %.6f\n', hpl2, expected2);
        n_fail = n_fail + 1;
    end

    % ------------------------------------------------------------------
    % TEST 3 — Vertical covariance must not affect HPL (horizontal only).
    %   Same EN block as Test 2 but a very different Up variance; HPL
    %   must be unchanged.
    % ------------------------------------------------------------------
    P_enu3 = [dE2, dEN, 0; dEN, dN2, 0; 0, 0, 1e6];
    P_ecef3 = R' * P_enu3 * R;
    hpl3 = compute_hpl(BUCU, P_ecef3);
    if abs(hpl3 - hpl2) < 1e-6
        fprintf('  [PASS] Test 3: HPL independent of vertical covariance\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] Test 3: HPL changed with Up variance (%.6f vs %.6f)\n', hpl3, hpl2);
        n_fail = n_fail + 1;
    end

    % ------------------------------------------------------------------
    % TEST 4 — Sanity bounds: HPL > 0, finite, and HPL >= d_major (K_H>=1).
    % ------------------------------------------------------------------
    ok4 = isfinite(hpl2) && hpl2 > 0 && hpl2 >= d_major_true;
    if ok4
        fprintf('  [PASS] Test 4: HPL positive, finite, >= d_major\n');
        n_pass = n_pass + 1;
    else
        fprintf('  [FAIL] Test 4: HPL=%.6f d_major=%.6f\n', hpl2, d_major_true);
        n_fail = n_fail + 1;
    end

    % ------------------------------------------------------------------
    fprintf('\n  RESULT: %d/%d passed\n', n_pass, n_pass + n_fail);
    if n_fail == 0
        fprintf('  === ALL TESTS PASSED ===\n\n');
    else
        fprintf('  === %d TEST(S) FAILED ===\n\n', n_fail);
    end
end

function R = enu_rotation(lat, lon)
% ENU_ROTATION  Rotation matrix mapping an ECEF vector to local ENU.
%   v_enu = R * v_ecef.  Standard geodetic ENU basis at (lat, lon).
    sl = sin(lat); cl = cos(lat);
    so = sin(lon); co = cos(lon);
    R = [ -so,      co,     0;
          -sl*co,  -sl*so,  cl;
           cl*co,   cl*so,  sl ];
end
