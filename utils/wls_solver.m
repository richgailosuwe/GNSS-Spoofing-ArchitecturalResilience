% Weighted Least Squares position solver. It takes corrected pseudoranges 
% and satellite positions and computes where the receiver is. This is the 
% core mathematics that everything else depends on 
% RAIM-FDE calls it repeatedly, and the EKF uses it for initialisation.

function [pos, clk_bias, residuals, H, W] = wls_solver(pseudoranges, sat_positions, weights, pos_init)
% wls_solver  Weighted Least Squares GNSS position solver.
%
%   [pos, clk_bias, residuals, H, W] = wls_solver(pseudoranges, sat_positions, weights, pos_init)
%
%   INPUT:
%     pseudoranges   - [n x 1] corrected pseudoranges (metres)
%     sat_positions  - [n x 3] satellite ECEF positions (metres)
%     weights        - [n x 1] measurement weights (1/sigma^2 per satellite)
%     pos_init       - [3 x 1] initial receiver ECEF position (metres)
%                      use [0;0;0] for first call — solver iterates to truth
%
%   OUTPUT:
%     pos            - [3 x 1] estimated receiver ECEF position (metres)
%     clk_bias       - receiver clock bias (metres)
%     residuals      - [n x 1] post-fit pseudorange residuals (metres)
%     H              - [n x 4] geometry matrix (for chi-squared test)
%     W              - [n x n] weight matrix (for chi-squared test)
%
%   Returns NaN outputs if solution does not converge or insufficient sats.
%
%   Algorithm: iterative linearised WLS, IS-GPS-200 Section 20.3.3.4.3

%% ── CONSTANTS ────────────────────────────────────────────────────────────
MAX_ITER   = 10;
CONV_THRESH = 1e-4;   % convergence threshold (metres)
MIN_SATS   = 4;       % minimum satellites needed for a solution

%% ── INPUT VALIDATION ─────────────────────────────────────────────────────
n = length(pseudoranges);

if n < MIN_SATS
    pos      = nan(3,1);
    clk_bias = NaN;
    residuals= nan(n,1);
    H        = nan(n,4);
    W        = nan(n,n);
    return;
end

% Remove NaN measurements
valid = ~isnan(pseudoranges) & ~any(isnan(sat_positions), 2);
if sum(valid) < MIN_SATS
    pos      = nan(3,1);
    clk_bias = NaN;
    residuals= nan(n,1);
    H        = nan(n,4);
    W        = nan(n,n);
    return;
end

pr   = pseudoranges(valid);
spos = sat_positions(valid, :);
w    = weights(valid);
n    = length(pr);

%% ── BUILD WEIGHT MATRIX ──────────────────────────────────────────────────
W_sub = diag(w);

%% ── INITIALISE STATE ─────────────────────────────────────────────────────
% State vector: [x, y, z, clock_bias]
if norm(pos_init) < 1
    % No initial position — start at Earth centre (will converge)
    x_est = [0; 0; 0; 0];
else
    x_est = [pos_init(1); pos_init(2); pos_init(3); 0];
end

%% ── ITERATIVE WLS ────────────────────────────────────────────────────────
H_sub = zeros(n, 4);

for iter = 1:MAX_ITER

    pos_now = x_est(1:3);
    clk_now = x_est(4);

    %% ── Build geometry matrix H and predicted ranges ─────────────────
    rho_pred = zeros(n, 1);

    for k = 1:n
        % Geometric range from current estimate to satellite
        diff     = spos(k,:)' - pos_now;
        rho_k    = norm(diff);
        rho_pred(k) = rho_k + clk_now;

        % Row of H: unit vector from receiver to satellite + clock column
        H_sub(k,:) = [-diff(1)/rho_k, -diff(2)/rho_k, -diff(3)/rho_k, 1];
    end

    %% ── Innovation vector ────────────────────────────────────────────
    delta_rho = pr - rho_pred;

    %% ── WLS update ───────────────────────────────────────────────────
    % dx = (H'*W*H)^-1 * H'*W * delta_rho
    HtW    = H_sub' * W_sub;
    HtWH   = HtW * H_sub;

    % Check for singularity
    if rcond(HtWH) < 1e-12
        pos      = nan(3,1);
        clk_bias = NaN;
        residuals= nan(n,1);
        H        = H_sub;
        W        = W_sub;
        return;
    end

    dx = HtWH \ (HtW * delta_rho);

    %% ── Update state ─────────────────────────────────────────────────
    x_est = x_est + dx;

    %% ── Check convergence ────────────────────────────────────────────
    if norm(dx(1:3)) < CONV_THRESH
        break;
    end

end % iteration loop

%% ── EXTRACT SOLUTION ─────────────────────────────────────────────────────
pos      = x_est(1:3);
clk_bias = x_est(4);

%% ── COMPUTE POST-FIT RESIDUALS ───────────────────────────────────────────
residuals_sub = zeros(n, 1);
for k = 1:n
    rho_k = norm(spos(k,:)' - pos) + clk_bias;
    residuals_sub(k) = pr(k) - rho_k;
end

%% ── MAP BACK TO FULL SIZE (including NaN slots) ──────────────────────────
residuals        = nan(length(pseudoranges), 1);
residuals(valid) = residuals_sub;

H        = nan(length(pseudoranges), 4);
H(valid,:) = H_sub;

W        = nan(length(pseudoranges), length(pseudoranges));
W(valid, valid) = W_sub;

end


%To test on cmd window, I had used this--lat and lon gave an error of 4 and
%2km respectively, alt error was 82m (as of may 25, 2026)
% clear functions
% config
% obs = rinex_read_obs('data/rinex/observation/authentic.obs', cfg);
% nav = rinex_read_nav('data/rinex/navigation/authentic.nav', cfg);
% 
% rec_true = [4097129.928; 2007384.921; 4442138.914];
% t_noon   = obs.epochs(1441);
% 
% % Build inputs
% sat_pos_mat = [];
% pr_vec      = [];
% w_vec       = [];
% 
% for prn_k = 1:32
%     mask = (obs.GPS.time == t_noon) & (obs.GPS.prn == prn_k);
%     idx  = find(mask, 1, 'first');
%     if isempty(idx), continue; end
%     pr_k = obs.GPS.pseudorange_L1(idx);
%     if isnan(pr_k), continue; end
%     [pos_k, clk_k] = sat_position(nav, prn_k, 'GPS', t_noon);
%     if any(isnan(pos_k)), continue; end
%     pr_corr = pseudorange_correct(pr_k, pos_k, clk_k, rec_true, t_noon, nav, 'GPS', cfg);
%     if isnan(pr_corr), continue; end
%     sat_pos_mat(end+1,:) = pos_k';
%     pr_vec(end+1)        = pr_corr;
%     w_vec(end+1)         = 1/cfg.ekf.meas_noise_GPS;
% end
% 
% fprintf('Using %d satellites\n', length(pr_vec));
% 
% [pos_wls, clk_wls, res_wls, ~, ~] = wls_solver(pr_vec', sat_pos_mat, w_vec', [0;0;0]);
% [lat, lon, alt] = ecef2lla_simple(pos_wls);
% 
% fprintf('WLS solution:\n');
% fprintf('  Lat = %.6f deg  (expected 44.4268)\n', rad2deg(lat));
% fprintf('  Lon = %.6f deg  (expected 26.1025)\n', rad2deg(lon));
% fprintf('  Alt = %.1f m    (expected ~80m)\n',    alt);
% fprintf('  Clock = %.3f m\n', clk_wls);
% fprintf('  Max residual = %.3f m\n', max(abs(res_wls(~isnan(res_wls)))));