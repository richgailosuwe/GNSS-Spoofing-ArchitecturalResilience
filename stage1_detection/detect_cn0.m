function result = detect_cn0(obs, cfg)
% detect_cn0  Detect spoofing via C/N0 temporal anomaly monitoring.
%
%   THEORY:
%     Under normal conditions, each satellite's C/N0 changes slowly and
%     smoothly as it moves across the sky. A spoofing attack causes a
%     sudden step change in C/N0 for the captured satellites at the moment
%     of capture — the spoofer signal is stronger than authentic.
%     This detector monitors each satellite's C/N0 over a sliding window
%     and flags sudden upward jumps inconsistent with natural variation.
%
%   INPUT:
%     obs - observation struct from rinex_read_obs()
%     cfg - configuration struct from config.m
%
%   OUTPUT:
%     result - struct with fields:
%       .flag        [n_epochs x 1] logical — true = anomaly detected
%       .cn0_std     [n_epochs x 1] C/N0 std across satellites per epoch
%       .cn0_mean    [n_epochs x 1] C/N0 mean across satellites per epoch
%       .n_jumped    [n_epochs x 1] number of satellites with C/N0 jump
%       .confidence  [n_epochs x 1] detection confidence 0-1
%       .n_epochs    scalar
%       .threshold   threshold used

%% ── PARAMETERS ───────────────────────────────────────────────────────────
WIN_SIZE    = 10;    % sliding window size (epochs) for baseline C/N0
JUMP_THRESH = 6.0;  % dB-Hz — sudden increase above baseline = suspicious
MIN_JUMPED  = 2;    % minimum satellites jumping simultaneously to flag

%% ── INITIALISE OUTPUT ────────────────────────────────────────────────────
n_epochs = obs.n_epochs;
epochs   = obs.epochs;

result.flag       = false(n_epochs, 1);
result.cn0_std    = nan(n_epochs, 1);
result.cn0_mean   = nan(n_epochs, 1);
result.n_jumped   = zeros(n_epochs, 1);
result.confidence = zeros(n_epochs, 1);
result.n_epochs   = n_epochs;
result.threshold  = cfg.detect.cn0_std_threshold;

if cfg.verbose
    fprintf('      Running C/N0 temporal anomaly detector...\n');
    fprintf('      Window: %d epochs | Jump threshold: %.1f dB-Hz\n', ...
        WIN_SIZE, JUMP_THRESH);
end

%% ── BUILD PER-SATELLITE C/N0 TIME SERIES ────────────────────────────────
% Focus on GPS since that is the primary spoofing target
% Build a matrix: rows = epochs, columns = satellite PRNs
gps_prns  = unique(obs.GPS.prn);
n_gps     = length(gps_prns);
cn0_matrix = nan(n_epochs, n_gps);  % [n_epochs x n_gps]

for e = 1:n_epochs
    t_e = epochs(e);
    for p = 1:n_gps
        prn_k = gps_prns(p);
        mask  = (obs.GPS.time == t_e) & (obs.GPS.prn == prn_k);
        idx   = find(mask, 1, 'first');
        if isempty(idx), continue; end
        val = obs.GPS.cn0(idx);
        if ~isnan(val) && val > 10 && val < 65
            cn0_matrix(e, p) = val;
        end
    end
end

%% ── DETECT TEMPORAL JUMPS ────────────────────────────────────────────────
for e = (WIN_SIZE + 1):n_epochs

    % Baseline: mean C/N0 over preceding window for each satellite
    window_cn0  = cn0_matrix(e-WIN_SIZE:e-1, :);
    baseline    = mean(window_cn0, 1, 'omitnan');
    current     = cn0_matrix(e, :);

    % Count satellites with sudden upward jump
    jump        = current - baseline;
    n_jumped    = sum(jump > JUMP_THRESH & ~isnan(jump));

    result.n_jumped(e) = n_jumped;

    % Also compute cross-satellite statistics for reporting
    all_cn0 = current(~isnan(current));
    if length(all_cn0) >= 4
        result.cn0_std(e)  = std(all_cn0);
        result.cn0_mean(e) = mean(all_cn0);
    end

    % Flag if enough satellites jumped simultaneously
    result.flag(e) = n_jumped >= MIN_JUMPED;

    % Confidence: how many satellites jumped, how large were the jumps
    if n_jumped >= MIN_JUMPED
        valid_jumps = jump(jump > JUMP_THRESH & ~isnan(jump));
        avg_jump    = mean(valid_jumps);
        result.confidence(e) = min(n_jumped / 5, 1.0) * ...
                                min((avg_jump - JUMP_THRESH) / 10.0 + 0.5, 1.0);
    end

end

%% ── SUMMARY ──────────────────────────────────────────────────────────────
n_flagged = sum(result.flag);
if cfg.verbose
    fprintf('      Flagged epochs: %d / %d (%.1f%%)\n', ...
        n_flagged, n_epochs, 100*n_flagged/n_epochs);
    fprintf('      Mean C/N0 std: %.2f dB-Hz\n', ...
        mean(result.cn0_std, 'omitnan'));
    fprintf('      Max simultaneous jumps: %d satellites\n', ...
        max(result.n_jumped));
end

end

%% to test on cmd
% % On authentic data
% result_auth = detect_cn0(obs, cfg);
% 
% % On any spoofed scenario
% s1 = load('results/simulated_scenarios/scenario_1_gps/spoofed_obs.mat');
% result_s1 = detect_cn0(s1.obs_spoofed, cfg);
% 
% % On random scenario
% % Generate random scenario first
% cfg.spoof.randomise   = true;
% cfg.spoof.random_seed = 2026;
% config  % triggers the random block in config.m
% inject_spoofing(obs, cfg);  % saves to results/simulated_scenarios/random_seed2026/
% 
% % Then load and test it
% rng_obs = load('results/simulated_scenarios/random_seed2026/spoofed_obs.mat');
% result_rng = detect_cn0(rng_obs.obs_spoofed, cfg);