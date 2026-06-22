function pr_debiased = apply_sat_bias(pr_corrected, prn, bias_table)
% APPLY_SAT_BIAS  Apply a calibrated per-satellite bias correction.
%
% Subtracts the calibrated bias for satellite `prn` from a corrected
% pseudorange.  If `prn` is not present in bias_table (e.g. a satellite
% that was not visible during calibration), the pseudorange is returned
% unchanged — this is a conservative default: an uncalibrated satellite
% is treated as having zero known bias rather than guessing.
%
% bias_table.bias is defined such that:
%   pr_debiased = pr_corrected - bias_table.bias(idx)
%
% (bias_table.bias(idx) = mean(pr_corrected - geometric_model) during
%  calibration, so subtracting it removes the systematic offset.)
%
% INPUTS
%   pr_corrected  double — pseudorange after pseudorange_correct
%   prn           double — satellite PRN
%   bias_table    struct from calibrate_sat_bias.m
%
% OUTPUTS
%   pr_debiased   double — bias-corrected pseudorange
% STAGE:    4 — EKF Position Recovery (calibration utility)

    idx = find(bias_table.prn == prn, 1);

    if isempty(idx)
        pr_debiased = pr_corrected;  % uncalibrated satellite: no correction
        return
    end

    pr_debiased = pr_corrected - bias_table.bias(idx);

end