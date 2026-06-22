function pr_if = ionofree_combination(pr_L1, pr_L2, constellation)
% IONOFREE_COMBINATION  Dual-frequency ionosphere-free pseudorange combination.
%
% Computes the ionosphere-free linear combination of L1 and L2 (or
% equivalent second-frequency) pseudoranges, which cancels the first-order
% ionospheric delay term WITHOUT requiring a model (Klobuchar) or a
% calibration table.
%
% FORMULA:
%   PR_IF = (f1^2 * PR_f1 - f2^2 * PR_f2) / (f1^2 - f2^2)
%
% DERIVATION:
%   The ionospheric group delay is frequency-dependent:
%     I_f = K / f^2   (K = constant proportional to total electron content)
%   A measured pseudorange is:
%     PR_f = rho + I_f + (other non-dispersive errors)
%          = rho + K/f^2 + ...
%   Forming the combination above eliminates the K/f^2 term exactly
%   (to first order in TEC), leaving only the geometric range plus
%   non-dispersive errors (troposphere, clock, multipath, noise).
%
% Source: Hofmann-Wellenhof, B., Lichtenegger, H., & Wasle, E. (2008).
% GNSS - Global Navigation Satellite Systems: GPS, GLONASS, Galileo and
% more. Springer, Section 6.2.1 "Ionosphere-Free Linear Combination".
%
% FREQUENCIES (Hz), per constellation:
%   GPS:     L1 = 1575.42 MHz, L2 = 1227.60 MHz
%            Source: IS-GPS-200, Table 3-I.
%   Galileo: E1 = 1575.42 MHz, E5b = 1207.140 MHz (matches RINEX C7Q observable)
%            Source: Galileo OS SIS ICD, Table 3.
%   BeiDou:  B1I = 1561.098 MHz, B2I = 1207.140 MHz (matches RINEX C2I observable)
%            Source: BDS-SIS-ICD-2.1, Table 3-1.
%   GLONASS: G1 = 1602.0 MHz, G2 = 1246.0 MHz (FDMA channel-dependent in
%            reality, but nominal frequencies used here for the IF
%            combination coefficient — the +/- channel offset is small
%            relative to f^2 weighting and is neglected).
%            Source: GLONASS ICD, Edition 5.1, Table 3.2.
%
% NOTE ON L2 OBSERVABLE CODES (from CLAUDE.md field table):
%   GPS L2:     C2W (encrypted P(Y)-code tracking, semi-codeless)
%   Galileo L2: C7Q (E5b)
%   BeiDou L2:  C2I (B2I)
%   GLONASS L2: C2P
%   These are the .pseudorange_L2 fields produced by rinex_read_obs.
%
% TRADE-OFF — NOISE AMPLIFICATION:
%   The IF combination amplifies measurement noise relative to either raw
%   observable.  For GPS:
%     alpha = f1^2/(f1^2-f2^2) = 2.5457
%     beta  = f2^2/(f1^2-f2^2) = 1.5457
%     Var(PR_IF) = alpha^2 * Var(PR_L1) + beta^2 * Var(PR_L2)
%                 ~ 9.0 * Var(PR_L1)   (if Var(PR_L1) ~ Var(PR_L2))
%   i.e. noise standard deviation increases by a factor of ~3.
%   This is the standard, well-documented trade-off (Hofmann-Wellenhof
%   2008, Section 6.2.1): the IF combination removes a large systematic
%   error (ionosphere, often tens of metres) at the cost of amplifying
%   the random noise (typically a few metres before, ~3x after).  The net
%   effect is normally a substantial improvement for single-point
%   positioning, since the systematic error removed is much larger than
%   the noise added.
%
% INPUTS
%   pr_L1          double or [Nx1] — L1 (or primary-frequency) pseudorange [m]
%   pr_L2          double or [Nx1] — L2 (or secondary-frequency) pseudorange [m]
%   constellation  char — 'GPS' | 'Galileo' | 'BeiDou' | 'GLONASS'
%
% OUTPUTS
%   pr_if          double or [Nx1] — ionosphere-free pseudorange [m]
%                  Returns NaN if either input is NaN (propagates missing data).
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery (measurement model enhancement)

    switch constellation
        case 'GPS'
            f1 = 1575.42e6;
            f2 = 1227.60e6;
        case 'Galileo'
            f1 = 1575.42e6;   % E1
            f2 = 1207.140e6;  % E5b
        case 'BeiDou'
            f1 = 1561.098e6;  % B1I
            f2 = 1207.140e6;  % B2I
        case 'GLONASS'
            f1 = 1602.0e6;    % G1 (nominal)
            f2 = 1246.0e6;    % G2 (nominal)
        otherwise
            error('ionofree_combination: unknown constellation ''%s''', constellation);
    end

    f1_sq = f1^2;
    f2_sq = f2^2;
    denom = f1_sq - f2_sq;

    alpha = f1_sq / denom;
    beta  = f2_sq / denom;

    pr_if = alpha .* pr_L1 - beta .* pr_L2;

    % Propagate NaN: if either input is NaN, output is NaN.
    nan_mask = isnan(pr_L1) | isnan(pr_L2);
    pr_if(nan_mask) = NaN;

end