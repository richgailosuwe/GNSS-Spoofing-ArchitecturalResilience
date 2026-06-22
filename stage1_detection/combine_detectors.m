function result = combine_detectors(res_cn0, res_pr, res_clk, res_agc, cfg)
% combine_detectors  Fuse all Stage 1 detector outputs into one decision.
%
%   result = combine_detectors(res_cn0, res_pr, res_clk, res_agc, cfg)
%
%   THEORY:
%     Each detector targets a different spoofing signature:
%       - C/N0:        captures the transition moment (signal power jump)
%       - Pseudorange: monitors sustained residual growth
%       - Clock:       monitors bias/drift consistency
%       - AGC:         monitors RF power level (live hardware only)
%
%     No single detector is perfect. Fusion via weighted voting gives a
%     combined decision that is more robust than any individual detector.
%     Weights reflect each detector's reliability in simulation mode.
%
%     Fusion rule: weighted sum of confidence scores.
%     Flag raised if weighted sum exceeds fusion threshold.
%
%   INPUT:
%     res_cn0  - output of detect_cn0()
%     res_pr   - output of detect_pseudorange()
%     res_clk  - output of detect_clock()
%     res_agc  - output of detect_agc()
%     cfg      - configuration struct from config.m
%
%   OUTPUT:
%     result - struct with fields:
%       .flag            [n_epochs x 1] logical — combined spoofing flag
%       .confidence      [n_epochs x 1] combined confidence score 0-1
%       .cn0_contrib     [n_epochs x 1] C/N0 weighted contribution
%       .pr_contrib      [n_epochs x 1] pseudorange weighted contribution
%       .clk_contrib     [n_epochs x 1] clock weighted contribution
%       .agc_contrib     [n_epochs x 1] AGC weighted contribution
%       .n_detectors     number of detectors that fired per epoch
%       .n_epochs        scalar

%% ── DETECTOR WEIGHTS ─────────────────────────────────────────────────────
% Weights reflect reliability in current operating mode
% Pseudorange gets highest weight — most reliable in simulation
% AGC gets zero weight in simulated mode — not meaningful without hardware
% Weights sum to 1.0

if strcmp(res_agc.mode, 'live')
    w_cn0 = 0.20;
    w_pr  = 0.40;
    w_clk = 0.20;
    w_agc = 0.20;
else
    % Simulated mode — AGC weight redistributed to other detectors
    w_cn0 = 0.25;
    w_pr  = 0.50;
    w_clk = 0.25;
    w_agc = 0.00;
end

% Fusion threshold — weighted confidence must exceed this to raise flag
FUSION_THRESH = 0.15;

if cfg.verbose
    fprintf('      Fusing detectors...\n');
    fprintf('      Weights: C/N0=%.2f PR=%.2f CLK=%.2f AGC=%.2f\n', ...
        w_cn0, w_pr, w_clk, w_agc);
    fprintf('      Fusion threshold: %.2f\n', FUSION_THRESH);
end

%% ── INITIALISE OUTPUT ────────────────────────────────────────────────────
n_epochs = res_pr.n_epochs;

result.flag        = false(n_epochs, 1);
result.confidence  = zeros(n_epochs, 1);
result.cn0_contrib = zeros(n_epochs, 1);
result.pr_contrib  = zeros(n_epochs, 1);
result.clk_contrib = zeros(n_epochs, 1);
result.agc_contrib = zeros(n_epochs, 1);
result.n_detectors = zeros(n_epochs, 1);
result.n_epochs    = n_epochs;

%% ── FUSE PER EPOCH ───────────────────────────────────────────────────────
for e = 1:n_epochs

    % Get confidence from each detector
    % Use 0 if epoch index out of range
    conf_cn0 = get_conf(res_cn0.confidence, e);
    conf_pr  = get_conf(res_pr.confidence,  e);
    conf_clk = get_conf(res_clk.confidence, e);
    if isfield(res_agc, 'confidence')
        conf_agc = get_conf(res_agc.confidence, e);
    else
        conf_agc = 0;
    end

    % Weighted contributions
    cn0_c = w_cn0 * conf_cn0;
    pr_c  = w_pr  * conf_pr;
    clk_c = w_clk * conf_clk;
    agc_c = w_agc * conf_agc;

    % Combined confidence
    combined = cn0_c + pr_c + clk_c + agc_c;

    result.cn0_contrib(e) = cn0_c;
    result.pr_contrib(e)  = pr_c;
    result.clk_contrib(e) = clk_c;
    result.agc_contrib(e) = agc_c;
    result.confidence(e)  = combined;

    % Count how many detectors flagged this epoch
    n_fired = 0;
    if e <= length(res_cn0.flag),  n_fired = n_fired + res_cn0.flag(e); end
    if e <= length(res_pr.flag),   n_fired = n_fired + res_pr.flag(e);  end
    if e <= length(res_clk.flag),  n_fired = n_fired + res_clk.flag(e); end
    if e <= length(res_agc.flag),  n_fired = n_fired + res_agc.flag(e); end
    result.n_detectors(e) = n_fired;

    % Combined flag
    result.flag(e) = combined > FUSION_THRESH;

end

%% ── SUMMARY ──────────────────────────────────────────────────────────────
n_flagged = sum(result.flag);
if cfg.verbose
    fprintf('      Flagged epochs: %d / %d (%.1f%%)\n', ...
        n_flagged, n_epochs, 100*n_flagged/n_epochs);
    fprintf('      Mean confidence (flagged): %.3f\n', ...
        mean(result.confidence(result.flag)));
    fprintf('      Max detectors firing simultaneously: %d\n', ...
        max(result.n_detectors));
end

end

%% ── LOCAL HELPER ─────────────────────────────────────────────────────────
function c = get_conf(conf_vec, e)
% Safely get confidence value — returns 0 if out of range or NaN
    if e <= length(conf_vec) && ~isnan(conf_vec(e))
        c = conf_vec(e);
    else
        c = 0;
    end
end

%% tested on cmd using;
% result_combined_auth = combine_detectors(result_auth, result_pr_auth, ...
%                                           result_clk_auth, result_agc_auth, cfg);
% 
% result_combined_spoof = combine_detectors(result_spoof, result_pr_spoof, ...
%                                            result_clk_spoof, result_agc_spoof, cfg);
% 
% fprintf('\n=== STAGE 1 COMBINED DETECTION RESULTS ===\n\n');
% fprintf('AUTHENTIC:\n');
% fprintf('  Flagged: %d/%d (%.1f%%) — false alarm rate\n', ...
%     sum(result_combined_auth.flag), result_combined_auth.n_epochs, ...
%     100*sum(result_combined_auth.flag)/result_combined_auth.n_epochs);
% fprintf('  Mean confidence: %.4f\n', mean(result_combined_auth.confidence));
% 
% fprintf('\nSPOOFED (Scenario 1 — GPS):\n');
% fprintf('  Flagged: %d/%d (%.1f%%) — detection rate\n', ...
%     sum(result_combined_spoof.flag), result_combined_spoof.n_epochs, ...
%     100*sum(result_combined_spoof.flag)/result_combined_spoof.n_epochs);
% fprintf('  Mean confidence: %.4f\n', mean(result_combined_spoof.confidence));
% 
% first_flag = find(result_combined_spoof.flag, 1, 'first');
% fprintf('  First detection: epoch %d (attack at epoch 120)\n', first_flag);
% fprintf('  Detection delay: %d epochs (%.1f minutes)\n', ...
%     first_flag - 120, (first_flag - 120) * 0.5);
% 
% fprintf('\nPer-detector contribution at first detection epoch %d:\n', first_flag);
% fprintf('  C/N0:        %.4f\n', result_combined_spoof.cn0_contrib(first_flag));
% fprintf('  Pseudorange: %.4f\n', result_combined_spoof.pr_contrib(first_flag));
% fprintf('  Clock:       %.4f\n', result_combined_spoof.clk_contrib(first_flag));
% fprintf('  AGC:         %.4f\n', result_combined_spoof.agc_contrib(first_flag));
% fprintf('  Combined:    %.4f\n', result_combined_spoof.confidence(first_flag));
