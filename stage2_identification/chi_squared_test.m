function result = chi_squared_test(residuals, weights, n_unknowns, cfg)
% chi_squared_test  Chi-squared consistency test on pseudorange residuals.
%
%   result = chi_squared_test(residuals, weights, n_unknowns, cfg)
%
%   THEORY:
%     Under normal conditions, weighted squared pseudorange residuals
%     follow a chi-squared distribution with degrees of freedom:
%       dof = n_measurements - n_unknowns
%
%     n_unknowns = 4 for standard GNSS (x, y, z, clock bias)
%
%     The test statistic is:
%       T = sum(residuals_i^2 / sigma_i^2)
%
%     If T exceeds the chi-squared threshold for the chosen false alarm
%     probability, the null hypothesis (all measurements consistent) is
%     rejected — indicating a fault or spoofing.
%
%     Reference: Parkinson & Spilker (1996) "Global Positioning System:
%     Theory and Applications", Chapter on RAIM.
%
%   INPUT:
%     residuals  - [n x 1] post-fit pseudorange residuals (metres)
%     weights    - [n x 1] measurement weights (1/sigma^2)
%     n_unknowns - number of unknowns in position solution (usually 4)
%     cfg        - configuration struct from config.m
%
%   OUTPUT:
%     result - struct with fields:
%       .test_stat   chi-squared test statistic
%       .threshold   chi-squared threshold at chosen false alarm prob
%       .dof         degrees of freedom
%       .passed      true = consistent (no fault), false = fault detected
%       .p_value     probability of observing this statistic under H0

%% ── VALIDATE INPUTS ──────────────────────────────────────────────────────
% Remove NaN entries
valid      = ~isnan(residuals) & ~isnan(weights) & weights > 0;
res_valid  = residuals(valid);
w_valid    = weights(valid);
n_meas     = length(res_valid);
dof        = n_meas - n_unknowns;

result.test_stat = NaN;
result.threshold = NaN;
result.dof       = dof;
result.passed    = true;
result.p_value   = 1.0;

if dof <= 0
    % Not enough measurements for a meaningful test
    return;
end

%% ── COMPUTE TEST STATISTIC ───────────────────────────────────────────────
% Weighted sum of squared residuals
% T = r' * W * r  where W = diag(weights)
T = sum(w_valid .* res_valid.^2);
result.test_stat = T;

%% ── COMPUTE THRESHOLD ────────────────────────────────────────────────────
% Chi-squared threshold for chosen false alarm probability
% chi2inv(1 - P_fa, dof) = threshold
P_fa           = cfg.identify.false_alarm_prob;
threshold      = chi2inv(1 - P_fa, dof);
result.threshold = threshold;

%% ── COMPUTE P-VALUE ──────────────────────────────────────────────────────
% Probability of observing T or larger under H0 (no fault)
result.p_value = 1 - chi2cdf(T, dof);

%% ── DECISION ─────────────────────────────────────────────────────────────
% passed = true means measurements are consistent (no spoofing detected)
% passed = false means fault detected — proceed to RAIM-FDE
result.passed = T <= threshold;

end

%% Test
% clear functions
% config
% w_mat_new = ones(n_v, 1) / cfg.ekf.meas_noise_GPS;
% r_auth = chi_squared_test(res_post, w_mat_new, 4, cfg);
% fprintf('AUTHENTIC post-fit chi-squared:\n');
% fprintf('  Test stat: %.4f\n', r_auth.test_stat);
% fprintf('  Threshold: %.4f (dof=%d)\n', r_auth.threshold, r_auth.dof);
% fprintf('  P-value:   %.6f\n', r_auth.p_value);
% fprintf('  Passed:    %d\n', r_auth.passed);

% [pos_spoof, clk_spoof, ~, ~, ~] = wls_solver(pr_mat2, sat_mat2, w_mat2, cfg.ref_pos);
% [pos_auth,  clk_auth,  ~, ~, ~] = wls_solver(pr_mat, sat_mat, w_mat, cfg.ref_pos);
% 
% [lat_s, lon_s, alt_s] = coord_convert('ecef2lla_deg', pos_spoof);
% [lat_a, lon_a, alt_a] = coord_convert('ecef2lla_deg', pos_auth);
% 
% fprintf('Authentic position:  Lat=%.4f Lon=%.4f Alt=%.1fm\n', lat_a, lon_a, alt_a);
% fprintf('Spoofed position:    Lat=%.4f Lon=%.4f Alt=%.1fm\n', lat_s, lon_s, alt_s);
% fprintf('Position difference: %.1f m\n', norm(pos_spoof - pos_auth));