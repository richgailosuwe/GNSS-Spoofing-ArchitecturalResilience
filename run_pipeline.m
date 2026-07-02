function pipeline_result = run_pipeline(scenario_id, verbose)
% RUN_PIPELINE  Run the full spoofing detection pipeline for one scenario.
%
% Usage:
%   run_pipeline(1)              % Scenario 1, quiet
%   run_pipeline(1, true)        % Scenario 1, verbose
%   run_pipeline('scenario_3_beidou')        % by name, quiet
%   run_pipeline('scenario_3_beidou', true)  % by name, verbose
%
% Inputs:
%   scenario_id  integer index (1-5) or scenario name string
%   verbose      logical, default false
%
% Output:
%   pipeline_result  struct saved to results/pvt/<scenario_name>_pipeline.mat
%
% Scenarios (from config.m):
%   1 = scenario_1_gps           GPS only
%   2 = scenario_2_galileo       Galileo only
%   3 = scenario_3_beidou        BeiDou only
%   4 = scenario_4_gps_glonass   GPS + GLONASS
%   5 = scenario_5_gps_galileo   GPS + Galileo (stress test)
%

%% ── ARGUMENTS ─────────────────────────────────────────────────────────────
if nargin < 1, scenario_id = 1; end
if nargin < 2, verbose = false; end

%% ── CONFIGURATION ─────────────────────────────────────────────────────────
run('config.m');
cfg.verbose = logical(verbose);

% Resolve scenario index
if ischar(scenario_id) || isstring(scenario_id)
    scenario_id = char(scenario_id);
    idx = find(strcmp(arrayfun(@(s) s.name, ...
        [cfg.scenarios{:}], 'UniformOutput', false), scenario_id), 1);
    if isempty(idx)
        error('run_pipeline: scenario ''%s'' not found in config.', scenario_id);
    end
else
    idx = scenario_id;
    if idx < 1 || idx > numel(cfg.scenarios)
        error('run_pipeline: scenario index %d out of range (1-%d).', ...
            idx, numel(cfg.scenarios));
    end
end

% Apply chosen scenario into cfg.spoof from cfg.scenarios{idx}
s = cfg.scenarios{idx};
cfg.spoof.scenario_name          = s.name;
cfg.spoof.spoofed_constellations = s.spoofed_constellations;
cfg.spoof.spoofed_PRNs           = s.spoofed_PRNs;
cfg.spoof.start_epoch            = s.start_epoch;
cfg.spoof.drift_rate             = s.drift_rate;
cfg.spoof.target_offset          = s.target_offset;
cfg.spoof.cn0_boost              = s.cn0_boost;

if cfg.verbose
    fprintf('\n========================================\n');
    fprintf('  GNSS Spoofing Detection Pipeline\n');
    fprintf('  Scenario %d: %s\n', idx, s.name);
    fprintf('  Constellations: %s\n', strjoin(s.spoofed_constellations, '+'));
    fprintf('========================================\n\n');
end

%% ── PATHS ─────────────────────────────────────────────────────────────────
addpath(fullfile(cfg.root, 'utils'));
addpath(fullfile(cfg.root, 'utils', 'calibration'));
addpath(fullfile(cfg.root, 'stage0_osnma'));
addpath(fullfile(cfg.root, 'stage1_detection'));
addpath(fullfile(cfg.root, 'stage2_identification'));
addpath(fullfile(cfg.root, 'stage3_exclusion'));
addpath(fullfile(cfg.root, 'stage4_recovery'));

%% ── STAGE 0: OSNMA ────────────────────────────────────────────────────────
% Operates on raw UBX hardware data, independently of the RINEX pipeline.
% Returns AUTH_UNKNOWN at current scope (cryptographic prerequisites
% validated against EUSPA Annex A test vectors; DSM-KROOT assembly deferred).
if cfg.verbose, fprintf('[Stage 0] OSNMA authentication...\n'); end

ubx_path = fullfile(cfg.root, 'Data', 'raw', 'hardware', 'june16rooftop.ubx');
keys_dir = fullfile(cfg.root, 'stage0_osnma', 'keys');

if isfile(ubx_path)
    osnma_result = osnma_verify(ubx_path, keys_dir, cfg);
else
    % No hardware UBX file — AUTH_UNKNOWN by default (simulation mode)
    osnma_result.auth_status    = 'AUTH_UNKNOWN';
    osnma_result.n_pages_parsed = 0;
    osnma_result.n_osnma_nonzero = 0;  % required by verbose print below
    if cfg.verbose
        fprintf('    No UBX file found — AUTH_UNKNOWN (simulation mode)\n');
    end
end

if cfg.verbose
    fprintf('    Status: %s | Pages: %d | OSNMA bits: %d\n', ...
        osnma_result.auth_status, ...
        osnma_result.n_pages_parsed, ...
        osnma_result.n_osnma_nonzero);
end

%% ── LOAD RINEX ────────────────────────────────────────────────────────────
if cfg.verbose, fprintf('[Data]  Loading RINEX files...\n'); end

obs_file = fullfile(cfg.paths.obs, 'authentic.obs');
nav_file = fullfile(cfg.paths.nav, 'authentic.nav');
if ~isfile(obs_file), error('Observation file not found: %s', obs_file); end
if ~isfile(nav_file), error('Navigation file not found: %s', nav_file); end

obs = rinex_read_obs(obs_file, cfg);
nav = rinex_read_nav(nav_file, cfg);

if cfg.verbose
    fprintf('    Loaded %d epochs, %d GPS satellites\n', ...
        numel(unique(obs.GPS.time)), numel(unique(obs.GPS.prn)));
end

%% ── INJECT SPOOFING ───────────────────────────────────────────────────────
if cfg.verbose
    fprintf('[Spoof] Injecting: %s...\n', s.name);
end
obs_spoofed = inject_spoofing(obs, nav, cfg);
if cfg.verbose
    fprintf('    Attack starts at epoch %d\n', s.start_epoch);
end

%% ── STAGE 1: DETECTION ────────────────────────────────────────────────────
if cfg.verbose, fprintf('[Stage 1] Signal anomaly detection...\n'); end

res_cn0 = detect_cn0(obs_spoofed, cfg);
res_pr  = detect_pseudorange(obs_spoofed, nav, cfg);
res_clk = detect_clock(obs_spoofed, nav, cfg);

% AGC: hardware-only channel (UBX-MON-RF, ZED-F9P physically connected).
% Disabled in RINEX-only simulation: UBX-MON-RF AGC data not available.
% A fake AGC derived from C/N0 would not be independent evidence.
% Full detector contract provided so combine_detectors needs no
% detector-specific mode logic. w_agc=0 when mode='simulation'.
n_ep_agc = numel(unique(obs_spoofed.GPS.time));
res_agc = struct( ...
    'flag',       false(n_ep_agc, 1), ...
    'confidence', zeros(n_ep_agc, 1), ...
    'agc',        nan(n_ep_agc, 1),   ...
    'agc_drop',   nan(n_ep_agc, 1),   ...
    'mode',       'simulation',        ...
    'n_epochs',   n_ep_agc,            ...
    'threshold',  cfg.detect.agc_drop_threshold);

% HARDWARE MODE (when ZED-F9P connected): uncomment and comment AGC stub
% res_agc = detect_agc(obs_spoofed, cfg);

detection_result = combine_detectors(res_cn0, res_pr, res_clk, res_agc, cfg);

if cfg.verbose
    n_flagged = sum(detection_result.flag);
    fprintf('    Flagged epochs: %d / %d\n', n_flagged, detection_result.n_epochs);
end

%% ── STAGE 2: IDENTIFICATION ───────────────────────────────────────────────
if cfg.verbose, fprintf('[Stage 2] Satellite identification...\n'); end

epochs_all = unique(obs_spoofed.GPS.time);
n_epochs   = numel(epochs_all);
classify_results = cell(n_epochs, 1);

for ei = 1:n_epochs
    t_e = epochs_all(ei);
    obs_epoch    = extract_epoch(obs_spoofed, t_e);
    raim_result  = raim_fde(obs_epoch, nav, cfg, t_e);
    inter_result = inter_constellation(obs_epoch, nav, cfg, t_e);
    classify_results{ei} = classify_spoofed_sats(raim_result, inter_result, obs_epoch, cfg);
end

if cfg.verbose
    n_spoof_epochs = sum(cellfun(@(r) ~isempty(r) && any(r.spoofed_mask), classify_results));
    fprintf('    Epochs with spoofed satellites identified: %d / %d\n', ...
        n_spoof_epochs, n_epochs);
end

%% ── STAGES 3+4: EXCLUSION + EKF ──────────────────────────────────────────
if cfg.verbose, fprintf('[Stage 3+4] Exclusion and EKF recovery...\n'); end

ekf_out = ekf_runner(obs_spoofed, nav, classify_results, cfg);

if cfg.verbose
    n_coasted = sum(ekf_out.coasted);
    final_err = norm(ekf_out.pos(end,:)' - cfg.ref_pos);
    fprintf('    EKF complete. Coasted epochs: %d / %d\n', n_coasted, n_epochs);
    fprintf('    Final position error: %.2f m\n', final_err);
end

%% ── ASSEMBLE AND SAVE ─────────────────────────────────────────────────────
if cfg.verbose, fprintf('[Done]  Saving results...\n'); end

pipeline_result.scenario       = s.name;
pipeline_result.scenario_idx   = idx;
pipeline_result.osnma          = osnma_result;
pipeline_result.detection      = detection_result;
pipeline_result.classification = classify_results;
pipeline_result.ekf            = ekf_out;
pipeline_result.ref_pos        = cfg.ref_pos;
pipeline_result.epochs         = epochs_all;

if ~isfolder(cfg.paths.pvt), mkdir(cfg.paths.pvt); end
pvt_file = fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', s.name));
pipeline_result.config_snapshot = cfg;
pipeline_result.created = datetime('now');
pipeline_result.code_notes = 'post BGD-fix #11 + insufficient-geometry-fallback #12';
save(pvt_file, 'pipeline_result');

if cfg.verbose
    fprintf('    Saved: %s\n', pvt_file);
    fprintf('\n========================================\n');
    fprintf('  Pipeline complete: %s\n', s.name);
    fprintf('  OSNMA status:  %s\n', osnma_result.auth_status);
    fprintf('  Final pos err: %.2f m\n', norm(ekf_out.pos(end,:)' - cfg.ref_pos));
    fprintf('========================================\n\n');
end

end % function run_pipeline

%% ── LOCAL HELPER: extract_epoch ───────────────────────────────────────────
function obs_epoch = extract_epoch(obs, t_e)
% Extract all observations at time t_e into a flat per-epoch struct.
% Stage 2 contract: flat fields prn, constellation, pseudorange, cn0.
% Copied exactly from working main.m implementation.
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    obs_epoch.time          = t_e;
    obs_epoch.prn           = [];
    obs_epoch.constellation = {};
    obs_epoch.pseudorange   = [];
    obs_epoch.cn0           = [];

    for ci = 1:numel(constellations)
        c = constellations{ci};
        if ~isfield(obs, c), continue; end

        mask  = (obs.(c).time == t_e);
        prn_c = obs.(c).prn(mask);
        pr_c  = obs.(c).pseudorange_L1(mask);
        cn0_c = nan(size(prn_c));
        if isfield(obs.(c), 'cn0')
            cn0_c = obs.(c).cn0(mask);
        end

        valid = ~isnan(pr_c) & pr_c > 0;
        prn_c = prn_c(valid);
        pr_c  = pr_c(valid);
        cn0_c = cn0_c(valid);

        obs_epoch.prn           = [obs_epoch.prn,           prn_c(:)'           ];
        obs_epoch.constellation = [obs_epoch.constellation, repmat({c},1,numel(prn_c))];
        obs_epoch.pseudorange   = [obs_epoch.pseudorange,   pr_c(:)'            ];
        obs_epoch.cn0           = [obs_epoch.cn0,           cn0_c(:)'           ];
    end
end
