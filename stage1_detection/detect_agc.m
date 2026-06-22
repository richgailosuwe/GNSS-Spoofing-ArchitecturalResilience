%to be used when ZED-F9P is physically connected. 
function result = detect_agc(obs, cfg)
% detect_agc  Detect spoofing via AGC (Automatic Gain Control) monitoring.
%
%   result = detect_agc(obs, cfg)
%
%   THEORY:
%     The receiver front-end amplifier automatically adjusts its gain to
%     keep the total received signal power constant. When a spoofer
%     broadcasts stronger-than-authentic signals, the total RF power at
%     the antenna increases. The AGC compensates by reducing gain —
%     producing a measurable DROP in the AGC register value.
%
%     Normal AGC behaviour:
%       - Slowly varying with sky conditions and temperature
%       - Typical range: 30-50 dB (receiver-dependent)
%       - Variation rate: <1 dB per minute under normal conditions
%
%     Spoofing signature:
%       - Sudden drop of 3+ dB at moment of capture
%       - Sustained lower value during attack
%       - Recovery jump when spoofer transmits off
%
%     Reference: Mitch et al. (2011) "Signal Characteristics of Civil
%     GPS Jammers" ION GNSS 2011. Wildemeersch et al. (2012).
%
%   HARDWARE NOTE:
%     AGC values require a live ZED-F9P receiver via UBX-MON-RF messages.
%     This function operates in two modes:
%       Mode 1 — LIVE: reads AGC from obs.agc field (ZED-F9P connected)
%       Mode 2 — SIM:  simulates AGC from C/N0 data for pipeline testing
%
%   INPUT:
%     obs - observation struct from rinex_read_obs()
%     cfg - configuration struct from config.m
%
%   OUTPUT:
%     result - struct with fields:
%       .flag        [n_epochs x 1] logical — true = anomaly detected
%       .agc         [n_epochs x 1] AGC values (dB) — real or simulated
%       .agc_drop    [n_epochs x 1] AGC drop from baseline (dB)
%       .confidence  [n_epochs x 1] detection confidence 0-1
%       .mode        'live' or 'simulated'
%       .n_epochs    scalar
%       .threshold   threshold used

%% ── PARAMETERS ───────────────────────────────────────────────────────────
AGC_DROP_THRESH = cfg.detect.agc_drop_threshold;   % dB — from config.m
WIN_SIZE        = 20;   % epochs for AGC baseline estimation

%% ── INITIALISE OUTPUT ────────────────────────────────────────────────────
n_epochs = obs.n_epochs;

result.flag       = false(n_epochs, 1);
result.agc        = nan(n_epochs, 1);
result.agc_drop   = nan(n_epochs, 1);
result.confidence = zeros(n_epochs, 1);
result.n_epochs   = n_epochs;
result.threshold  = AGC_DROP_THRESH;

%% ── DETERMINE MODE ───────────────────────────────────────────────────────
% Check if real AGC data is available
if isfield(obs, 'agc') && ~isempty(obs.agc) && any(~isnan(obs.agc))
    result.mode = 'live';
    agc_raw     = obs.agc;
    if cfg.verbose
        fprintf('      Running AGC detector — LIVE mode (ZED-F9P data)\n');
    end
else
    result.mode = 'simulated';
    if cfg.verbose
        fprintf('      Running AGC detector — SIMULATED mode\n');
        fprintf('      (AGC unavailable without ZED-F9P — simulating from C/N0)\n');
    end

    %% ── Simulate AGC from C/N0 ───────────────────────────────────────
    % AGC is inversely related to total received power
    % When C/N0 increases (spoofer boost), AGC drops
    % Simulation: AGC = 40 - mean(C/N0 across GPS sats) * 0.3 + noise
    agc_raw = nan(n_epochs, 1);
    epochs  = obs.epochs;

    for e = 1:n_epochs
        t_e = epochs(e);
        mask = (obs.GPS.time == t_e);
        if ~any(mask), continue; end

        cn0_e = obs.GPS.cn0(mask);
        cn0_e = cn0_e(~isnan(cn0_e) & cn0_e > 10);
        if isempty(cn0_e), continue; end

        % Simulate AGC: inversely proportional to mean C/N0
        % Add small Gaussian noise to simulate real receiver behaviour
        mean_cn0    = mean(cn0_e);
        agc_raw(e)  = 42.0 - (mean_cn0 - 40.0) * 0.25 + ...
                      0.2 * randn();  % ±0.2 dB noise
    end
end

result.agc = agc_raw;

%% ── DETECT AGC DROPS ─────────────────────────────────────────────────────
for e = (WIN_SIZE + 1):n_epochs
    if isnan(agc_raw(e)), continue; end

    % Baseline: mean AGC over preceding window
    window_agc = agc_raw(e-WIN_SIZE:e-1);
    valid_win  = ~isnan(window_agc);
    if sum(valid_win) < WIN_SIZE/2, continue; end

    baseline = mean(window_agc(valid_win));

    % AGC drop = how much lower than baseline (positive = dropped)
    agc_drop = baseline - agc_raw(e);
    result.agc_drop(e) = agc_drop;

    % Flag if drop exceeds threshold
    result.flag(e) = agc_drop > AGC_DROP_THRESH;

    % Confidence
    if result.flag(e)
        result.confidence(e) = min(agc_drop / (3 * AGC_DROP_THRESH), 1.0);
    end
end

%% ── SUMMARY ──────────────────────────────────────────────────────────────
n_flagged = sum(result.flag);
if cfg.verbose
    fprintf('      Mode: %s\n', result.mode);
    fprintf('      Flagged epochs: %d / %d (%.1f%%)\n', ...
        n_flagged, n_epochs, 100*n_flagged/n_epochs);
    fprintf('      Max AGC drop: %.2f dB\n', ...
        max(result.agc_drop, [], 'omitnan'));
end

end

%% tested on cmd using 
% clear functions
% result_agc_auth  = detect_agc(obs, cfg);
% result_agc_spoof = detect_agc(obs_spoofed_s1, cfg);
% 
% fprintf('\nAUTHENTIC — flagged: %d/%d (%.1f%%)\n', ...
%     sum(result_agc_auth.flag), result_agc_auth.n_epochs, ...
%     100*sum(result_agc_auth.flag)/result_agc_auth.n_epochs);
% fprintf('SPOOFED   — flagged: %d/%d (%.1f%%)\n', ...
%     sum(result_agc_spoof.flag), result_agc_spoof.n_epochs, ...
%     100*sum(result_agc_spoof.flag)/result_agc_spoof.n_epochs);
% fprintf('Mode: %s\n', result_agc_spoof.mode);