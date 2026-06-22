# Annex - Core MATLAB Source Listings

The listings below were generated directly from the MATLAB files in the project workspace. Code is reproduced verbatim in pipeline order, except that lines beginning with the project or author attribution comments requested for removal were omitted. Descriptions and section headings are editorial text and are not part of the source files.

## Configuration

### `config.m`

Defines project paths, calibrated parameters, estimator settings, integrity settings, and spoofing scenarios.

```matlab
% config.m
% Central configuration for the GNSS spoofing detection thesis.
% Every tunable parameter, file path, and setting lives here.
% No other file should hardcode any of these values.

%% ── PATHS ────────────────────────────────────────────────────────────────

% Root directory (auto-detected, do not change)
cfg.root = fileparts(mfilename('fullpath'));

% RTKLIB convbin executable 
cfg.convbin = 'C:\Users\RG\Downloads\RTKLIB_bin-rtklib_2.4.3\RTKLIB_bin-rtklib_2.4.3\bin\convbin.exe';

% Input data
cfg.paths.raw_authentic   = fullfile(cfg.root, 'data/raw/authentic');
cfg.paths.raw_spoofed     = fullfile(cfg.root, 'data/raw/spoofed');
cfg.paths.obs             = fullfile(cfg.root, 'data/rinex/observation');
cfg.paths.nav             = fullfile(cfg.root, 'data/rinex/navigation');
cfg.paths.reference       = fullfile(cfg.root, 'data/reference');

% Output
cfg.paths.figures         = fullfile(cfg.root, 'results/figures');
cfg.paths.logs            = fullfile(cfg.root, 'results/logs');
cfg.paths.pvt             = fullfile(cfg.root, 'results/pvt');
cfg.paths.scenarios       = fullfile(cfg.root, 'results/simulated_scenarios');

%% ── RECEIVER SETTINGS ────────────────────────────────────────────────────

% ZED-F9P serial port (I need to update to match the COM port in Device Manager)
cfg.receiver.port         = 'COM3';
cfg.receiver.baud         = 115200;

% Elevation mask: satellites below this angle are ignored (degrees)
%FAA Advisory Circular (AC 20-138), RTCA Performance Standards (DO-208 /
%DO-229), requires a degree mask angle of 5
%cfg.receiver.elev_mask    = 10;
cfg.receiver.elev_mask    = 5;

%% ── CONSTELLATION SELECTION ──────────────────────────────────────────────

% Which constellations to use (true = use, false = ignore)
% --to review later in the project
cfg.const.GPS             = true;
cfg.const.Galileo         = true;
cfg.const.BeiDou          = true;
cfg.const.GLONASS         = true;

%% ── STAGE 1 — DETECTION THRESHOLDS ──────────────────────────────────────

% AGC : flag if drop exceeds this value in one epoch (dB)
cfg.detect.agc_drop_threshold     = 3.0;

% C/N0 : flag if standard deviation across satellites drops below (dB-Hz)
cfg.detect.cn0_std_threshold      = 2.0;

% Clock : flag if bias/drift ratio deviates beyond this (metres)-15 was
% used because when detect clock was ran intially The authentic consistency error is tiny ..max 14.6 m, 99.9th percentile only 6.96 m. This means the threshold should be set at 15 m to give essentially zero false alarms while being sensitive to real anomalies.
cfg.detect.clock_consistency_threshold = 15.0; 

% Pseudorange residual :flag individual satellite if residual exceeds (metres)
% Threshold set at 120m — just above 99.9th percentile of authentic
% residuals observed at BUCU station (117.4m) per calibration analysis.
% Gives <0.1% false alarm probability on authentic data.
cfg.detect.residual_threshold     = 120.0;
%100m is initial val. clear val between noise and sth serious

%% ── STAGE 2 — IDENTIFICATION ─────────────────────────────────────────────

% Chi-squared false alarm probability (0.001 = 0.1%)
%The chi-squared false alarm probability of 0.001 is standard in RAIM
%literature (Autonomous GPS integrity monitoring using the pseudorange
%residual by Parkinson-1988)
cfg.identify.false_alarm_prob     = 0.001;

% Minimum satellites required after exclusion to attempt recovery- (for
% X,Y,Z + clock  bias, the 5th satellite is for 1 deg of redundancy
cfg.identify.min_sats             = 5;

% Inter-constellation consistency
cfg.identify.min_sats_per_constellation = 4;      % minimum sats to compute a standalone WLS solution

% Methodology: 99.9th percentile of authentic pairwise constellation-solution
% distances across all epochs. See calibrate_inter_const_threshold.m.
cfg.identify.inter_const_threshold = 195.0;
% 99.9th percentile of authentic inter-constellation pairwise distances.
% Source: BUCU00ROU authentic data, 17-May-2026, all 2880 epochs,
% 17280 pairwise distances. BeiDou PRN 33 included after verification
% as healthy (residual std 1.9 m, comparable to healthy BeiDou MEO).
% Supersedes earlier 180 m calibration after transmit-time/PRN set update.
% Detection sensitivity: spoofing offset must exceed ~195 m (epoch 159+ for 5 m/epoch drag-off).
cfg.identify.const_dist_threshold  = cfg.identify.inter_const_threshold;

%% ── INTEGRITY METRIC ─────────────────────────────────────────────────────
% Stage 4 HPL: conservative RAIM/NPA-style horizontal protection-level
% multiplier. Source: WAAS MOPS / RTCA DO-229 NPA-mode horizontal value
% (K_H,NPA = 6.18).
%
% This is NOT a certified RNP AR / SBAS / GBAS integrity allocation. It is
% an experimental RAIM-style integrity metric for thesis-level evaluation.
cfg.integrity.K_H = 6.18;

%% ── STAGE 4 — EXTENDED KALMAN FILTER ────────────────────────────────────
% All values empirically calibrated from 100 authentic BUCU epochs,
% 17-May-2026, unless otherwise noted.
% =========================================================================

cfg.stage3.max_cond_HtH = 1000;    % 40x worst authentic epoch cond(H^TH)=24.06

% ---- Stage 3: measurement exclusion / fallback tunables -----------------
% Surfaced for traceability; values match function-internal defaults (kept as
% defensive fallbacks). Full rationale in apply_exclusion_mask.m / ekf_runner.m.
cfg.stage3.spoof_weight_inflation       = 1e6;        % spoofed-tier variance inflation (numerical exclusion)
cfg.stage3.suspect_weight_inflation     = 5;          % suspect-tier inflation - PROVISIONAL, not BUCU-calibrated
cfg.stage3.use_innovation_gate          = true;       % scalar gate on; false = ablation C arm only
cfg.stage3.insufficient_geometry_policy = 'gate_only';% fallback when n_trusted<min_sats; or 'coast'

% Stage 4 EKF — base 8-state model: [x, y, z, vx, vy, vz, clk_GPS, clk_drift]
% Extended with one inter-system bias (ISB) state per ENABLED non-GPS
% constellation (Method 1: GPS master clock + N-1 ISB states).
% State layout: 1:3=pos, 4:6=vel, 7=clk_GPS, 8=clk_drift, 9..=ISB states.
cfg.ekf.dt              = 30.0;     % s — RINEX epoch interval

% Ordered list of non-GPS constellations that get an ISB state (enabled only).
% GPS is the master clock and never appears here. Read by build_meas_model_isb.
cfg.ekf.isb_order = {};
isb_candidates = {'Galileo','BeiDou','GLONASS'};
for ic = 1:numel(isb_candidates)
    cn = isb_candidates{ic};
    if isfield(cfg.const, cn) && cfg.const.(cn)
        cfg.ekf.isb_order{end+1} = cn;
    end
end
cfg.ekf.n_isb    = numel(cfg.ekf.isb_order);
cfg.ekf.n_states = 8 + cfg.ekf.n_isb;   % 8 (GPS-only) up to 11 (all four)


% --- Initial state covariance P0 ---
% P_init_pos: PDOP^2 x sigma^2_GPS = 1.17^2 x 333 = 455.84 m^2
%   Source: Groves (2013), Section 9.4.1
cfg.ekf.P_init_pos      = 456.0;   % m^2

% P_init_vel: conservative cold-start for velocity states.
%   No prior velocity information at startup; 10 m/s std covers typical
%   aviation speeds.  For static sessions velocity converges to zero within
%   a few epochs.
cfg.ekf.P_init_vel      = 100.0;   % (m/s)^2  (= 10 m/s std)

% P_init_clk: conservative cold-start uncertainty.
%   Source: IS-GPS-200, Section 20.3.3
cfg.ekf.P_init_clk      = 1e6;     % m^2

% P_init_drift: (std of smooth clock delta / dt)^2
%   std_smooth = 0.3818 m/epoch (100-epoch BUCU calib, jumps excluded)
%   P_init_drift = (0.3818/30)^2 = 1.62e-4 (m/s)^2
cfg.ekf.P_init_drift    = 1.62e-4; % (m/s)^2
% P_init_isb: initial inter-system bias uncertainty (per ISB state).
%   Reproduction test measured authentic ISBs of +17.05 m (BeiDou),
%   +5.90 m (GLONASS), +2.64 m (Galileo) vs GPS over 200 BUCU epochs.
%   30 m std (900 m^2) bounds all three at cold start.
cfg.ekf.P_init_isb      = 900.0;   % m^2  (= 30 m std)

% Q_isb: inter-system bias random-walk process noise.
%   ISB drifts slowly with receiver temperature and broadcast time-offset
%   updates. STARTING VALUE — not yet calibrated. Calibrate later from the
%   ISB time-series stability (same MAD methodology as Q_clk), analogous to
%   the uncalibrated Stage 3 suspect-tier inflation placeholder.
cfg.ekf.Q_isb           = 1e-4;    % m^2 per epoch (slow random walk, provisional)

% --- Process noise Q ---
cfg.ekf.Q_pos           = 0.01;    % m^2  — static station stabiliser
% Q_vel: for static sessions, small value keeps velocity near zero.
%   For aviation, increase to cover typical manoeuvre accelerations.
%   0.01 (m/s)^2 corresponds to ~0.1 m/s^2 std acceleration per epoch,
%   appropriate for commercial aviation cruise.
%   Source: Groves (2013), Section 9.4.2 — process noise for airborne GNSS.
cfg.ekf.Q_vel           = 0.01;    % (m/s)^2

% Q_clk: (1.4826 x MAD)^2, robust 100-epoch BUCU calibration.
%   MAD = 0.2167 m/epoch -> robust sigma = 0.3213 m -> Q = 0.1032 m^2.
%   Raw std (3.08 m) excluded — inflated by constellation-change WLS artefacts.
cfg.ekf.Q_clk           = 0.10;    % m^2

% Q_clk_drift: TCXO oscillator frequency stability at 30s.
%   Source: u-blox ZED-F9P Integration Manual, Section 3.1.5
cfg.ekf.Q_clk_drift     = 1e-6;    % (m/s)^2

% --- Initial clock drift state ---
% Effective GPS clock-bias drift initial state.
% Calibrated with GPS-only snapshot WLS using the final transmit-time model.
% Full-day drift is statistically indistinguishable from zero
% (mean ~= +0.0001 m/epoch, slope ~= +0.0004 m/epoch over 2880 epochs;
%  1:100 mean -0.0140 m/epoch is only 0.034*sigma, i.e. noise, not a trend).
% This is an estimator initial condition, not a claim of zero oscillator drift.
% The clock-drift process noise Q_clk_drift lets the EKF estimate residual drift.
% Supersedes -0.02454 m/s, which was calibrated before transmit-time correction
% and was contaminated by reception-time satellite-position error.
% Source: stage4_recovery/calibrate_clock_drift.m, BUCU 17-May-2026.
cfg.ekf.clk_drift_init  = 0.0; % m/s

%% ── MEASUREMENT NOISE (sigma^2 in metres^2) ─────────────────────────────
% GPS: calibrated from BUCU 24hr dataset post-fit residuals
% std = 18.26m measured directly → sigma^2 = 333 m^2
cfg.ekf.meas_noise_GPS = 333.0;

% Galileo: superior to GPS due to Passive Hydrogen Maser clocks and faster
% navigation message updates. Broadcast SISRE ~12cm vs GPS ~50cm.
% Reference: Carlin et al. (2021) GPS Solutions, DOI:10.1007/s10291-021-01111-4
% Practical 3D positioning: Galileo ~39cm vs GPS ~41cm broadcast ephemeris.
% Reference: Springer GPS Solutions (2021), Table 3.
% Applied correction factor: (39/41)^2 = 0.905 → 333 * 0.905 = 301 m^2
cfg.ekf.meas_noise_Galileo = 301.0;

% BeiDou: calibrated from BUCU authentic mixed-constellation residuals.
% Max authentic residual at the RAIM test epoch = 70.51 m.
% sigma^2 = max_residual^2 = 4971.7 m^2.
cfg.ekf.meas_noise_BeiDou = 4972.0;

% GLONASS: calibrated from BUCU authentic mixed-constellation residuals.
% Max authentic residual at the RAIM test epoch = 58.96 m.
% sigma^2 = max_residual^2 = 3476.3 m^2.
cfg.ekf.meas_noise_GLONASS = 3476.0;

% Innovation gate — reject measurement if Mahalanobis distance exceeds this
cfg.ekf.innov_gate                = 3.0;    % 3-sigma gate

%% ── SPOOFING SCENARIOS ───────────────────────────────────────────────────
% Select which scenario to run (1-5). Change this number to switch scenarios.
% 1 = GPS only, 2 = Galileo only, 3 = BeiDou only,
% 4 = GPS+GLONASS, 5 = GPS+Galileo (stress test)
cfg.spoof.active_scenario = 1;

% ── Scenario 1: GPS only ──────────────────────────────────────────────────
% Most common real-world attack — targets the primary aviation constellation
cfg.scenarios{1}.name                   = 'scenario_1_gps';
% Which constellation(s) the spoofer is attacking
cfg.scenarios{1}.spoofed_constellations = {'GPS'};
% Which specific satellite PRNs within GPS are spoofed
% PRN = Pseudo-Random Noise code number — the satellite's unique identifier
cfg.scenarios{1}.spoofed_PRNs.GPS       = [14, 22, 31];
% Epoch number at which the attack begins (epoch 120 = 60 minutes into data)
cfg.scenarios{1}.start_epoch            = 120;
% How fast the spoofer drags pseudoranges away from truth (metres per epoch)
% Gradual drag-off model — Humphreys et al. 2008
cfg.scenarios{1}.drift_rate             = 5.0;
% Final fake position offset the spoofer is aiming for (metres, ECEF XYZ)
% norm([500,300,-100]) = ~591m total displacement from true position
cfg.scenarios{1}.target_offset          = [500, 300, -100];
% How much the spoofer boosts signal strength above authentic (dB)
% Spoofer must overpower authentic signal to capture receiver tracking loops
cfg.scenarios{1}.cn0_boost              = 8.0;

% ── Scenario 2: Galileo only ──────────────────────────────────────────────
% Tests whether system can detect spoofing on the OSNMA-protected constellation
% Realistic as Galileo adoption in aviation grows under EASA mandates
cfg.scenarios{2}.name                      = 'scenario_2_galileo';
cfg.scenarios{2}.spoofed_constellations    = {'Galileo'};
% E prefix = Galileo satellite PRNs
cfg.scenarios{2}.spoofed_PRNs.Galileo      = [1, 3, 5];
cfg.scenarios{2}.start_epoch               = 120;
cfg.scenarios{2}.drift_rate                = 5.0;
cfg.scenarios{2}.target_offset             = [500, 300, -100];
cfg.scenarios{2}.cn0_boost                 = 8.0;

% ── Scenario 3: BeiDou only ───────────────────────────────────────────────
% Tests detection of attack on Chinese constellation
% Relevant as BeiDou achieves full ICAO recognition for civil aviation
cfg.scenarios{3}.name                   = 'scenario_3_beidou';
cfg.scenarios{3}.spoofed_constellations = {'BeiDou'};
% C prefix = BeiDou satellite PRNs
cfg.scenarios{3}.spoofed_PRNs.BeiDou    = [1, 3, 6];
cfg.scenarios{3}.start_epoch            = 120;
cfg.scenarios{3}.drift_rate             = 5.0;
cfg.scenarios{3}.target_offset          = [500, 300, -100];
cfg.scenarios{3}.cn0_boost              = 8.0;

% ── Scenario 4: GPS + GLONASS ─────────────────────────────────────────────
% Documented in real-world incidents (Black Sea, Eastern Mediterranean)
% Russian military spoofing systems known to target both simultaneously
% Reference: EASA Safety Information Bulletin 2019-02
cfg.scenarios{4}.name                      = 'scenario_4_gps_glonass';
cfg.scenarios{4}.spoofed_constellations    = {'GPS', 'GLONASS'};
cfg.scenarios{4}.spoofed_PRNs.GPS          = [14, 22, 31];
% R prefix = GLONASS satellite PRNs
cfg.scenarios{4}.spoofed_PRNs.GLONASS      = [1, 3, 5];
cfg.scenarios{4}.start_epoch               = 120;
cfg.scenarios{4}.drift_rate                = 5.0;
cfg.scenarios{4}.target_offset             = [500, 300, -100];
cfg.scenarios{4}.cn0_boost                 = 8.0;

% ── Scenario 5: GPS + Galileo (stress test) ───────────────────────────────
% Most dangerous attack for this architecture specifically
% Removes GPS (primary navigation) AND Galileo (OSNMA cryptographic anchor)
% simultaneously — leaving only BeiDou + GLONASS as authentic references
% Tests the absolute limits of the inter-constellation voting scheme
cfg.scenarios{5}.name                      = 'scenario_5_gps_galileo';
cfg.scenarios{5}.spoofed_constellations    = {'GPS', 'Galileo'};
cfg.scenarios{5}.spoofed_PRNs.GPS          = [14, 22, 31];
cfg.scenarios{5}.spoofed_PRNs.Galileo      = [1, 3, 5];
cfg.scenarios{5}.start_epoch               = 120;
cfg.scenarios{5}.drift_rate                = 5.0;
cfg.scenarios{5}.target_offset             = [500, 300, -100];
cfg.scenarios{5}.cn0_boost                 = 8.0;

% ── Active scenario shortcut ──────────────────────────────────────────────
% Maps the selected scenario number above into cfg.spoof
% All pipeline code reads from cfg.spoof — never from cfg.scenarios directly
% To switch scenario: change cfg.spoof.active_scenario above and re-run config
s = cfg.spoof.active_scenario;
cfg.spoof.scenario_name          = cfg.scenarios{s}.name;
cfg.spoof.spoofed_constellations = cfg.scenarios{s}.spoofed_constellations;
cfg.spoof.spoofed_PRNs           = cfg.scenarios{s}.spoofed_PRNs;
cfg.spoof.start_epoch            = cfg.scenarios{s}.start_epoch;
cfg.spoof.drift_rate             = cfg.scenarios{s}.drift_rate;
cfg.spoof.target_offset          = cfg.scenarios{s}.target_offset;
cfg.spoof.cn0_boost              = cfg.scenarios{s}.cn0_boost;

% ── Random stress test mode ───────────────────────────────────────────────
% When randomise = true, ignores the fixed scenarios above and generates
% a random attack. Fixed seed ensures the same random scenario is produced
% every run — making it reproducible even though parameters are random.
% Set randomise = false for thesis main results (fixed scenarios above).
cfg.spoof.randomise  = false;
cfg.spoof.random_seed = 2026;   % change seed to get a different random scenario..% thesis year — non-arbitrary, fully documented

% If randomise is true, overwrite cfg.spoof with random parameters
if cfg.spoof.randomise
    rng(cfg.spoof.random_seed);

    % All four constellations available to attack
    all_constellations = {'GPS', 'Galileo', 'BeiDou', 'GLONASS'};

    % Randomly pick 1 or 2 constellations to spoof
    n_spoofed = randi([1 2]);
    const_order = randperm(4);
    picked = all_constellations(const_order(1:n_spoofed));
    cfg.spoof.spoofed_constellations = picked;

    % PRN pools per constellation
    prn_pool.GPS     = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 ...
                        16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32];
    prn_pool.Galileo = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 ...
                        16 17 18 19 20 21 22 23 24 25 26 27 28 29 30];
    prn_pool.BeiDou  = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 ...
                        16 17 18 19 20 21 22 23 24 25 26 27 28 29 30];
    prn_pool.GLONASS = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 ...
                        15 16 17 18 19 20 21 22 23 24];

    % Randomly pick 2-4 PRNs per spoofed constellation
    cfg.spoof.spoofed_PRNs = struct();
    for rc = 1:length(picked)
        cname    = picked{rc};
        pool     = prn_pool.(cname);
        n_prns   = randi([2 4]);
        perm     = randperm(length(pool));
        cfg.spoof.spoofed_PRNs.(cname) = pool(perm(1:n_prns));
    end

    % Randomly pick attack start epoch (between epoch 60 and 300)
    cfg.spoof.start_epoch  = randi([60 300]);

    % Randomly pick drift rate (between 2 and 10 m/epoch)
    cfg.spoof.drift_rate   = 2 + 8 * rand();

    % Randomly pick target offset magnitude (between 200 and 1000m)
    mag = 200 + 800 * rand();
    az  = 2 * pi * rand();
    cfg.spoof.target_offset = [mag*cos(az), mag*sin(az), -50 + 100*rand()];

    % Randomly pick C/N0 boost (between 5 and 12 dB)
    cfg.spoof.cn0_boost = 5 + 7 * rand();

    % Generate a unique name for this random scenario
    cfg.spoof.scenario_name = sprintf('random_seed%d', cfg.spoof.random_seed);

    % Print what was randomly selected
    if cfg.verbose
        fprintf('  [RANDOM MODE] seed=%d\n', cfg.spoof.random_seed);
        fprintf('  Spoofed constellations: ');
        fprintf('%s ', cfg.spoof.spoofed_constellations{:});
        fprintf('\n');
        fprintf('  Start epoch: %d\n', cfg.spoof.start_epoch);
        fprintf('  Drift rate:  %.2f m/epoch\n', cfg.spoof.drift_rate);
        fprintf('  Target mag:  %.1f m\n', norm(cfg.spoof.target_offset));
        fprintf('  C/N0 boost:  %.1f dB\n', cfg.spoof.cn0_boost);
    end
end

%% ── DISPLAY SETTINGS ─────────────────────────────────────────────────────

% Print progress messages to Command Window (true/false)
cfg.verbose                       = true

%to be able to read the files in  the utils & other folders--to avoid having to manually
%change folders in the cmd window
addpath(fullfile(cfg.root, 'utils'));
addpath(fullfile(cfg.root, 'utils', 'calibration'))
addpath(fullfile(cfg.root, 'stage0_osnma'));
addpath(fullfile(cfg.root, 'stage1_detection'));
addpath(fullfile(cfg.root, 'stage2_identification'));
addpath(fullfile(cfg.root, 'stage3_exclusion'));
addpath(fullfile(cfg.root, 'stage4_recovery'));
addpath(fullfile(cfg.root, 'stage4_recovery','ionofree/'));

%% ── REFERENCE POSITION ───────────────────────────────────────────────────
% BUCU RINEX header approximate ECEF position (metres)
% From authentic.obs APPROX POSITION XYZ.
% Lat: 44.4636 N, Lon: 26.1256 E, Alt: ~96 m
cfg.ref_pos = [4093761.206; 2007793.576; 4445129.764];
```

## Measurement Model

### `utils/corrected_pseudorange.m`

Forms the final corrected pseudorange using transmit-time satellite geometry and constellation-specific group delay.

```matlab
function [pr_corr, sat_pos_tx, sat_clk] = corrected_pseudorange( ...
        pr_raw, prn, constellation, t_rx, rec_approx, nav, cfg)
% CORRECTED_PSEUDORANGE  Single owner of the GNSS measurement model.
%
%   [pr_corr, sat_pos_tx, sat_clk] = corrected_pseudorange( ...
%        pr_raw, prn, constellation, t_rx, rec_approx, nav, cfg)
%
% Forms a fully corrected pseudorange and returns the satellite position
% evaluated at SIGNAL TRANSMIT TIME, so callers use geometry consistent with
% the correction. Replaces the previous pattern of calling sat_position at
% RECEPTION time and then pseudorange_correct, which left an uncorrected
% transit-time satellite-motion error of tens of metres per satellite.
%
% MODEL (normal path, rec_approx valid):
%   1. Iterate transmit time on GEOMETRIC range (3 passes):
%        tau   = pr_raw / c               (seed)
%        t_tx  = t_rx - tau
%        sat   = sat_position(t_tx)
%        tau   = ||sat - rec_approx|| / c
%   2. pr_corr = pseudorange_correct(pr_raw, sat_pos_tx, sat_clk, ...)
%        (existing chain: sat clock + Sagnac + iono(Klobuchar) + tropo)
%   3. Single-frequency (L1) only: pr_corr = pr_corr - get_group_delay(...)
%
% COLD START (norm(rec_approx) < 1):
%   No receiver position yet, so transmit-time geometry, Sagnac, elevation,
%   and atmosphere cannot be computed. Evaluate the satellite at RECEPTION
%   time and return a clock-only corrected pseudorange, exactly matching the
%   existing pseudorange_correct early-return behaviour. Group delay is also
%   skipped at cold start (consistent with skipping the rest of the chain).
%
% FREQUENCY MODE:
%   Defaults to single-frequency L1 (this pipeline uses pseudorange_L1).
%   If cfg.receiver.freq_mode exists and equals 'IF', group delay is NOT
%   applied (the broadcast clock is already ionosphere-free referenced).
%   Absence of the field => 'L1' (safe default, never errors).
%
% OUTPUTS:
%   pr_corr     corrected pseudorange (metres), NaN if rejected (e.g. below
%               elevation mask, inherited from pseudorange_correct)
%   sat_pos_tx  [3x1] satellite ECEF position at transmit time (metres)
%   sat_clk     satellite clock correction at transmit time (metres)
%

    C_LIGHT = 299792458.0;

    % --- Frequency mode (default L1, never error on missing field) ---------
    freq_mode = 'L1';
    if isfield(cfg, 'receiver') && isfield(cfg.receiver, 'freq_mode')
        freq_mode = cfg.receiver.freq_mode;
    end

    % --- Cold start: no receiver position -> reception-time, clock only ----
    if norm(rec_approx) < 1
        [sat_pos_tx, sat_clk] = sat_position(nav, prn, constellation, t_rx);
        if any(isnan(sat_pos_tx))
            pr_corr = NaN; return;
        end
        % Clock-only corrected pseudorange (matches pseudorange_correct's
        % early return when no approximate position is available).
        pr_corr = pseudorange_correct(pr_raw, sat_pos_tx, sat_clk, ...
                                      rec_approx, t_rx, nav, constellation, cfg);
        return;
    end

    % --- Normal path: iterate transmit time on geometric range -------------
    tau = pr_raw / C_LIGHT;                 % seed (~0.07 s)
    sat_pos_tx = [NaN; NaN; NaN];
    sat_clk    = NaN;
    for iter = 1:3
        t_tx = t_rx - seconds(tau);
        [sat_pos_tx, sat_clk] = sat_position(nav, prn, constellation, t_tx);
        if any(isnan(sat_pos_tx))
            pr_corr = NaN; return;
        end
        tau = norm(sat_pos_tx - rec_approx) / C_LIGHT;
    end

    % --- Existing correction chain, using the transmit-time satellite ------
    % (sat clock + Sagnac + ionosphere + troposphere; may return NaN if the
    %  satellite is below the elevation mask)
    pr_corr = pseudorange_correct(pr_raw, sat_pos_tx, sat_clk, ...
                                  rec_approx, t_rx, nav, constellation, cfg);
    if isnan(pr_corr)
        return;   % below mask or otherwise rejected — propagate NaN
    end

    % --- Group delay (single-frequency L1 only) ----------------------------
    % Returned in metres, signed; SUBTRACTED (see get_group_delay header).
    if ~strcmpi(freq_mode, 'IF')
        gd = get_group_delay(nav, prn, constellation, t_tx);
        pr_corr = pr_corr - gd;
    end
end
```

### `utils/pseudorange_correct.m`

Applies satellite-clock, Earth-rotation, ionospheric, tropospheric, and elevation-mask corrections.

```matlab
function pr_corr = pseudorange_correct(pr_raw, sat_pos, sat_clk, rec_pos_approx, t, nav, constellation, cfg)
% pseudorange_correct  Apply corrections to raw pseudorange measurements.
%
%   pr_corr = pseudorange_correct(pr_raw, sat_pos, sat_clk,
%                                 rec_pos_approx, t, nav,
%                                 constellation, cfg)
%
%   INPUT:
%     pr_raw          - raw pseudorange (metres)
%     sat_pos         - satellite ECEF position [3x1] (metres)
%     sat_clk         - satellite clock correction (metres)
%     rec_pos_approx  - approximate receiver ECEF position [3x1] (metres)
%                       (use [0;0;0] for first iteration)
%     t               - observation epoch (datetime)
%     nav             - navigation struct from rinex_read_nav()
%     constellation   - 'GPS', 'Galileo', 'BeiDou', or 'GLONASS'
%     cfg             - configuration struct from config.m
%
%   OUTPUT:
%     pr_corr - corrected pseudorange (metres)
%
%   Corrections applied:
%     1. Satellite clock correction
%     2. Earth rotation correction (Sagnac effect) — SCALAR range term
%     3. Ionospheric delay (Klobuchar model) — GPS/GLONASS
%        or NeQuick approximation           — Galileo/BeiDou
%     4. Tropospheric delay (Saastamoinen model)
%
% =========================================================================
% BUG FIX HISTORY — Sagnac correction (this revision):
%
% PREVIOUS IMPLEMENTATION (incorrect):
%   The previous version rotated sat_pos by theta = Omega_E * signal_time
%   to produce sat_pos_corr, computed geo_range = norm(sat_pos_corr -
%   rec_pos_approx), and then DISCARDED both sat_pos_corr and geo_range.
%   The returned pr_corr never included any Sagnac term, while the
%   geometric range used downstream by wls_solver/ekf_runner was computed
%   from the UNROTATED sat_pos. The Sagnac effect (~10-40m for GPS MEO
%   satellites, satellite-position-dependent) was therefore never applied
%   anywhere in the pipeline. This was identified as the dominant
%   contributor to the 50-100m position error found in Stage 4 validation
%   (see HANDOVERSTAGE4.md, Sagnac investigation).
%
% CURRENT IMPLEMENTATION (fixed):
%   Applies the standard SCALAR Sagnac range correction directly to the
%   pseudorange, using the existing (uncorrected) sat_pos and
%   rec_pos_approx. No rotation matrix, no sat_pos_corr, nothing for
%   downstream callers to propagate -- the correction is folded entirely
%   into pr_corr, exactly like the clock/iono/tropo corrections.
%
%   Formula (Kaplan & Hegarty, 2017, "Understanding GPS/GNSS: Principles
%   and Applications", 3rd ed., Artech House, Section 5.4.1, Eq. 5.13):
%
%     dt_sagnac = (Omega_E / c) * (x_sat * y_rec - y_sat * x_rec)
%     pr_corr   = pr_corr - c * dt_sagnac
%               = pr_corr - (Omega_E / c) * (x_sat*y_rec - y_sat*x_rec)
%
%   This term represents the Sagnac (Earth-rotation) contribution to the
%   apparent range.  It is SUBTRACTED from the pseudorange here -- the
%   opposite of the satellite clock term -- because the Sagnac quantity
%   as conventionally defined represents an addition to the GEOMETRIC
%   RANGE MODEL (the rotating-to-inertial frame transformation effectively
%   lengthens the modeled range computed from a non-rotated sat_pos by
%   this amount for satellites east of the receiver).  To compensate the
%   MEASURED pseudorange so that it matches a geometric range computed
%   from the unrotated sat_pos, the term must be removed from pr_corr,
%   i.e. subtracted.
%
%   SIGN VERIFIED EMPIRICALLY (see test_sagnac_fix.m, HANDOVERSTAGE4.md):
%   an initial implementation with '+' increased mean position error from
%   68.9m to 90.1m over 50 epochs; '-' is the corrected sign. Any future
%   modification to this term MUST be re-validated against
%   test_sagnac_fix.m Test 3 (mean position error vs RTKLIB baseline)
%   before being accepted -- the sign is easy to get backwards and the
%   test result is the ground truth, not the formula derivation.
%
%   MAGNITUDE: for GPS MEO orbits (|sat_pos| ~ 26,000 km), this term is
%   typically ~10-40 m depending on satellite azimuth -- non-negligible
%   and satellite-position-dependent, which explains why the previous
%   bug produced satellite-specific biases of the magnitude observed
%   (PRN19 ~+40m, PRN15 ~-29m, etc.) rather than a uniform offset.
% =========================================================================

%% ── CONSTANTS ────────────────────────────────────────────────────────────
C_LIGHT = 299792458.0;   % speed of light (m/s)
OMEGA_E = 7.2921151467e-5;  % Earth rotation rate (rad/s), WGS84

%% ── VALIDATE INPUT ───────────────────────────────────────────────────────
if isnan(pr_raw) || pr_raw <= 0
    pr_corr = NaN;
    return;
end

if any(isnan(sat_pos))
    pr_corr = NaN;
    return;
end

%% ── 1. SATELLITE CLOCK CORRECTION ────────────────────────────────────────
% Add satellite clock error to pseudorange. PR_corrected = PR + c*dt_sv
pr_corr = pr_raw + sat_clk;

%% ── 2. EARTH ROTATION CORRECTION (Sagnac effect) — SCALAR RANGE TERM ─────
% Applied only once rec_pos_approx is available (see geometric range guard
% below) -- the term depends on rec_pos.  For the bootstrap case
% (rec_pos_approx ~ 0), this is skipped along with atmospheric corrections,
% consistent with the existing early-return structure.
if norm(rec_pos_approx) >= 1
    sagnac_term = (OMEGA_E / C_LIGHT) * ...
                  (sat_pos(1) * rec_pos_approx(2) - sat_pos(2) * rec_pos_approx(1));
    pr_corr = pr_corr - sagnac_term;
end

%% ── 3. GEOMETRIC RANGE GUARD ─────────────────────────────────────────────
if norm(rec_pos_approx) < 1
    % No approximate position yet — skip atmospheric corrections.
    % (Sagnac term above was also skipped for the same reason.)
    return;
end

%% ── 4. ELEVATION ANGLE ───────────────────────────────────────────────────
% Needed for atmospheric corrections.
% NOTE: elevation is computed from the UNROTATED sat_pos. The Sagnac
% displacement (~100m) is negligible for elevation-angle purposes (it
% changes elevation by a fraction of a degree at most), so using sat_pos
% directly here (rather than a rotated copy) is an acceptable
% approximation consistent with how elevation is used only for masking
% and as an input to the tropospheric mapping function.
elev = compute_elevation(rec_pos_approx, sat_pos);

% Skip atmospheric corrections for satellites below elevation mask
if elev < deg2rad(cfg.receiver.elev_mask)
    pr_corr = NaN;
    return;
end

%% ── 5. IONOSPHERIC CORRECTION (Klobuchar model) ──────────────────────────
iono_delay = klobuchar_model(rec_pos_approx, sat_pos, t, nav, constellation);
pr_corr = pr_corr - iono_delay;

%% ── 6. TROPOSPHERIC CORRECTION (Saastamoinen model) ─────────────────────
tropo_delay = saastamoinen_model(rec_pos_approx, elev);
pr_corr = pr_corr - tropo_delay;

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: compute_elevation
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function elev = compute_elevation(rec_pos, sat_pos)
% Compute satellite elevation angle from receiver position.
% Returns elevation in radians.

    % Vector from receiver to satellite
    los = sat_pos - rec_pos;

    % Receiver unit normal (up direction in ECEF)
    rec_norm = rec_pos / norm(rec_pos);

    % Elevation = angle between LOS and local horizontal plane
    sin_elev = dot(los, rec_norm) / norm(los);
    elev     = asin(max(-1, min(1, sin_elev)));

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: klobuchar_model
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function iono = klobuchar_model(rec_pos, sat_pos, t, nav, constellation)
% Klobuchar ionospheric correction model.
% IS-GPS-200 Section 20.3.3.5.2.5
%
% Uses alpha/beta coefficients from GPS navigation message.
% For Galileo/BeiDou, falls back to GPS coefficients as approximation.

    %% ── Try to get Klobuchar coefficients from nav ───────────────────────
    alpha = [0; 0; 0; 0];
    beta  = [0; 0; 0; 0];

    if isfield(nav, 'GPS') && isfield(nav.GPS, 'data') && ...
       ~isempty(nav.GPS.data)
        vars = nav.GPS.data.Properties.VariableNames;
        % Coefficient names vary by RINEX version
        if any(strcmp(vars, 'Alpha0'))
            row = nav.GPS.data(1,:);
            alpha = [row.Alpha0; row.Alpha1; row.Alpha2; row.Alpha3];
            beta  = [row.Beta0;  row.Beta1;  row.Beta2;  row.Beta3];
        end
    end

    %% ── Receiver geodetic coordinates ────────────────────────────────────
    [lat, lon, ~] = ecef2lla(rec_pos);
    lat_u = lat / pi;   % semi-circles
    lon_u = lon / pi;

    %% ── Satellite elevation and azimuth ──────────────────────────────────
    los      = sat_pos - rec_pos;
    los_norm = los / norm(los);
    rec_norm = rec_pos / norm(rec_pos);

    sin_elev = dot(los_norm, rec_norm);
    elev_sc  = asin(max(-1,min(1,sin_elev))) / pi; % semi-circles

    % Azimuth
    east  = [-sin(lon);  cos(lon);  0];
    north = [-sin(lat)*cos(lon); -sin(lat)*sin(lon); cos(lat)];
    az    = atan2(dot(los_norm, east), dot(los_norm, north));

    %% ── Earth-centred angle ──────────────────────────────────────────────
    psi = 0.0137 / (elev_sc + 0.11) - 0.022;

    %% ── Subionospheric latitude / longitude ──────────────────────────────
    lat_i = lat_u + psi * cos(az);
    lat_i = max(-0.416, min(0.416, lat_i));
    lon_i = lon_u + psi * sin(az) / cos(lat_i * pi);

    %% ── Geomagnetic latitude ─────────────────────────────────────────────
    lat_m = lat_i + 0.064 * cos((lon_i - 1.617) * pi);

    %% ── Local time ───────────────────────────────────────────────────────
    t_sec = second(t) + minute(t)*60 + hour(t)*3600;
    lt    = mod(4.32e4 * lon_i + t_sec, 86400);

    %% ── Period and amplitude ─────────────────────────────────────────────
    AMP = alpha(1) + alpha(2)*lat_m + alpha(3)*lat_m^2 + alpha(4)*lat_m^3;
    AMP = max(0, AMP);

    PER = beta(1) + beta(2)*lat_m + beta(3)*lat_m^2 + beta(4)*lat_m^3;
    PER = max(72000, PER);

    %% ── Ionospheric delay (seconds) ──────────────────────────────────────
    x = 2 * pi * (lt - 50400) / PER;

    F = 1 + 16 * (0.53 - elev_sc)^3;  % slant factor

    if abs(x) >= 1.57
        T_iono = F * 5e-9;
    else
        T_iono = F * (5e-9 + AMP * (1 - x^2/2 + x^4/24));
    end

    % Convert to metres
    iono = T_iono * 299792458.0;

end


%% LOCAL HELPER: saastamoinen_model
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function tropo = saastamoinen_model(rec_pos, elev)
% Saastamoinen tropospheric correction model.
% Uses standard atmosphere — no meteorological data required.

    %% ── Receiver height above ellipsoid ──────────────────────────────────
    [~, ~, alt] = ecef2lla(rec_pos);
    alt = max(0, alt); % clip to sea level minimum

    %% ── Standard atmosphere parameters at height h ───────────────────────
    % Pressure (hPa)
    P = 1013.25 * (1 - 2.2557e-5 * alt)^5.2568;

    % Temperature (K)
    T = 288.15 - 6.5e-3 * alt;

    % Partial pressure of water vapour (hPa) — standard humidity
    e = 6.108 * exp(17.15 * (T - 273.15) / (T - 38.65));
    e = 0.5 * e; % assume 50% relative humidity

    %% ── Zenith delays ────────────────────────────────────────────────────
    % Dry (hydrostatic) zenith delay
    d_dry = 0.002277 * P / (1 - 0.00266 * cos(2 * atan2(norm(rec_pos(1:2)), rec_pos(3))) ...
            - 0.00028 * alt / 1000);

    % Wet zenith delay
    d_wet = 0.002277 * (1255/T + 0.05) * e;

    %% ── Map to slant delay using elevation ───────────────────────────────
    % Simple 1/sin(elev) mapping function
    if elev < deg2rad(2)
        elev = deg2rad(2); % floor at 2 degrees
    end

    tropo = (d_dry + d_wet) / sin(elev);

end


%% LOCAL HELPER: ecef2lla
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function [lat, lon, alt] = ecef2lla(pos)
% Convert ECEF (metres) to geodetic lat/lon (radians) and altitude (metres).
% Bowring iterative method.

    a  = 6378137.0;          % WGS84 semi-major axis (m)
    f  = 1/298.257223563;    % WGS84 flattening
    e2 = 2*f - f^2;          % first eccentricity squared

    x = pos(1); y = pos(2); z = pos(3);

    lon = atan2(y, x);
    p   = sqrt(x^2 + y^2);
    lat = atan2(z, p * (1 - e2));

    % Iterate
    for i = 1:10
        N      = a / sqrt(1 - e2 * sin(lat)^2);
        lat_new = atan2(z + e2 * N * sin(lat), p);
        if abs(lat_new - lat) < 1e-12, break; end
        lat = lat_new;
    end
    lat = lat_new;

    N   = a / sqrt(1 - e2 * sin(lat)^2);
    alt = p / cos(lat) - N;

end
```

### `utils/get_group_delay.m`

Selects the matching broadcast ephemeris and returns the applicable GPS TGD or Galileo/BeiDou broadcast group delay.

```matlab
function gd_metres = get_group_delay(nav, prn, constellation, t)
% GET_GROUP_DELAY  Broadcast group-delay correction for single-frequency users.
%
%   gd_metres = get_group_delay(nav, prn, constellation, t)
%
% Returns the group-delay correction in METRES, as a SIGNED quantity, to be
% SUBTRACTED from the corrected pseudorange by the caller:
%
%       pr_corr = pr_corr - get_group_delay(...)
%
% RATIONALE (IS-GPS-200, Section 20.3.3.3.3.2):
%   The broadcast satellite clock polynomial is referenced to the dual-
%   frequency ionosphere-free combination. A single-frequency L1 user must
%   apply (dt_SV)_L1 = dt_SV - T_GD. Since this pipeline forms the satellite
%   clock as sat_clk = c*dt_SV and uses pr_corr = pr_raw + sat_clk, the L1
%   correction enters as an ADDITIONAL SUBTRACTION of c*T_GD. Hence this
%   function returns c*T_GD (signed) and the caller subtracts it.
%
% PER-CONSTELLATION TERM (mapped to this project's L1 observable, verified
% against the authentic.obs SYS/OBS TYPES header):
%   GPS     C1C = L1 C/A  -> c * TGD                  (column 'TGD')
%   Galileo C1C = E1      -> c * BGD (band-selected)  (see Galileo note)
%   BeiDou  C1P = B1C     -> 0  (see note)            (TGD1/TGD2 are B1I/B2I)
%   GLONASS C1C = G1 C/A  -> 0  (no broadcast group delay in this ephemeris)
%
% GALILEO NOTE (band selection, RINEX 3.02/4.0x):
%   The broadcast clock is referenced to a specific iono-free pair depending
%   on the navigation message source:
%     F/NAV clock -> E5a,E1 pair -> apply BGDE5aE1
%     I/NAV clock -> E5b,E1 pair -> apply BGDE5bE1
%   The source pair is encoded in 'DataSources' (Broadcast Orbit 5), bits 8/9:
%     bit 9 (512) set -> E5b,E1 (I/NAV)  e.g. 0x205=517, 0x201=513, 0x204=516
%     bit 8 (256) set -> E5a,E1 (F/NAV)  e.g. 0x102=258
%   Using a single hardcoded band would misapply ~0.2 m on roughly half the
%   Galileo records in a mixed I/NAV+F/NAV file. The correct term is selected
%   per record from DataSources.
%
% BeiDou NOTE: the tracked observable is B1C (C1P), but the broadcast TGD1/
%   TGD2 parameters are referenced to B1I/B2I. Applying TGD1 to a B1C
%   pseudorange would be the wrong correction, so it is omitted. To enable a
%   correct BeiDou group delay, switch the BeiDou L1 mapping in rinex_read_obs
%   from C1P (B1C) to C2I (B1I); then c*TGD1 becomes correct.
%
% IONOSPHERE-FREE MODE: if the caller forms an L1/L2 iono-free combination,
%   group delay must NOT be applied (the broadcast clock is already IF-
%   referenced). This function is only invoked by corrected_pseudorange in
%   single-frequency (L1) mode.
%
%   Mirrors sat_position/select_ephemeris so the group delay comes from the
%   SAME broadcast record used for satellite position and clock.
%

    C_LIGHT = 299792458.0;
    gd_metres = 0.0;

    nav_const = nav.(constellation);
    if isempty(nav_const.data)
        return;
    end

    % --- Select the same ephemeris row sat_position would use --------------
    % (closest |t - Toe| within a 4-hour validity window; fallback closest)
    prns = nav_const.prn;
    toes = nav_const.toe;
    prn_mask = (prns == prn);
    if ~any(prn_mask)
        return;
    end

    idx_prn   = find(prn_mask);
    toes_prn  = toes(prn_mask);
    dt        = seconds(t - toes_prn);
    abs_dt    = abs(dt);
    valid     = abs_dt < 14400;
    if ~any(valid)
        [~, jrel] = min(abs_dt);
    else
        abs_dt_valid = abs_dt;
        abs_dt_valid(~valid) = Inf;
        [~, jrel] = min(abs_dt_valid);
    end
    row = nav_const.data(idx_prn(jrel), :);
    vars = row.Properties.VariableNames;

    % --- Per-constellation group-delay term (metres) ----------------------
    switch constellation
        case 'GPS'
            if any(strcmp(vars, 'TGD'))
                gd_metres = C_LIGHT * row.TGD(1);
            end

        case 'Galileo'
            % Single-frequency E1 user: select the BGD term whose frequency
            % pair matches the CLOCK SOURCE of this navigation record.
            % bit 9 (512) -> E5b,E1 (I/NAV) -> BGDE5bE1
            % bit 8 (256) -> E5a,E1 (F/NAV) -> BGDE5aE1
            % (See header GALILEO NOTE. Verified vs RINEX 3.02 + authentic.nav.)
            ds_val = NaN;
            if any(strcmp(vars, 'DataSources'))
                ds_val = row.DataSources(1);
            end

            use_e5b = false;
            if ~isnan(ds_val)
                if bitand(uint32(ds_val), 512) ~= 0
                    use_e5b = true;            % I/NAV: E5b,E1 clock
                elseif bitand(uint32(ds_val), 256) ~= 0
                    use_e5b = false;           % F/NAV: E5a,E1 clock
                end
            end

            if use_e5b && any(strcmp(vars, 'BGDE5bE1'))
                gd_metres = C_LIGHT * row.BGDE5bE1(1);
            elseif any(strcmp(vars, 'BGDE5aE1'))
                gd_metres = C_LIGHT * row.BGDE5aE1(1);   % F/NAV or fallback
            end

        case 'BeiDou'
            % B1C observable: TGD1/TGD2 (B1I/B2I) do not apply. Omitted.
            gd_metres = 0.0;

        case 'GLONASS'
            % No broadcast group delay in this ephemeris format.
            gd_metres = 0.0;

        otherwise
            gd_metres = 0.0;
    end

    if isnan(gd_metres)
        gd_metres = 0.0;
    end
end
```

### `utils/sat_position.m`

Propagates GPS, Galileo, and BeiDou broadcast orbits and integrates GLONASS state vectors to obtain satellite position and clock correction.

```matlab
% this fcn takes the navigation file ephemeris & computes exactly where
% each satellite was in space (ECEF- earth centered earth fixed X,Y,Z coordinates) at any given moment
%helps cal. expected pseudoranges to compare against measured ones
function [sat_pos, sat_clk] = sat_position(nav, prn, constellation, t)
% sat_position  Compute satellite ECEF position and clock correction.
%
%   [sat_pos, sat_clk] = sat_position(nav, prn, constellation, t)
%
%   INPUT:
%     nav           - navigation struct from rinex_read_nav()
%     prn           - satellite PRN number (scalar)
%     constellation - 'GPS', 'Galileo', 'BeiDou', or 'GLONASS' (string)
%     t             - observation time (datetime scalar)
%
%   OUTPUT:
%     sat_pos - [3x1] satellite ECEF position (metres) [X; Y; Z]
%     sat_clk - satellite clock correction (metres)
%               (multiply by speed of light to convert to seconds)
%
%   Returns [NaN; NaN; NaN] and NaN if ephemeris not found.
%
%   Constants follow IS-GPS-200 and Galileo OS-SIS-ICD.

%% ── CONSTANTS ────────────────────────────────────────────────────────────
GM_GPS     = 3.986005e14;    % Earth gravitational constant GPS (m^3/s^2)
GM_GAL     = 3.986004418e14; % Earth gravitational constant Galileo
GM_BDS     = 3.986004418e14; % Earth gravitational constant BeiDou
GM_GLO     = 3.9860044e14;   % Earth gravitational constant GLONASS
OMEGA_E    = 7.2921151467e-5;% Earth rotation rate (rad/s)
C_LIGHT    = 299792458.0;    % Speed of light (m/s)
F          = -4.442807633e-10; % Relativistic correction constant (s/sqrt(m))

%% ── SELECT EPHEMERIS ─────────────────────────────────────────────────────
eph = select_ephemeris(nav.(constellation), prn, t);

if isempty(eph)
    sat_pos = [NaN; NaN; NaN];
    sat_clk = NaN;
    return;
end

%% ── ROUTE BY CONSTELLATION ───────────────────────────────────────────────
switch constellation
    case {'GPS', 'Galileo', 'BeiDou'}
        [sat_pos, sat_clk] = kepler_position(eph, t, constellation, ...
            GM_GPS, OMEGA_E, C_LIGHT, F);
    case 'GLONASS'
        [sat_pos, sat_clk] = glonass_position(eph, t, OMEGA_E, C_LIGHT);
    otherwise
        error('sat_position: unknown constellation: %s', constellation);
end

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: select_ephemeris
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function eph = select_ephemeris(nav_const, prn, t)
% Find the most appropriate ephemeris for this PRN at time t.
% Selects the record with minimum |tk| within a 4-hour validity window.

    if isempty(nav_const.data)
        eph = [];
        return;
    end

    tt   = nav_const.data;
    prns = nav_const.prn;
    toes = nav_const.toe;

    % Filter by PRN
    prn_mask = (prns == prn);
    if ~any(prn_mask)
        eph = [];
        return;
    end

    tt_prn   = tt(prn_mask, :);
    toes_prn = toes(prn_mask);

    % Time difference from each ephemeris epoch
    dt = seconds(t - toes_prn);

    % Select ephemeris with minimum absolute time difference
    % within a 4-hour validity window (14400 seconds)
    abs_dt = abs(dt);
    valid  = abs_dt < 14400;

    if ~any(valid)
        % Fallback — use closest regardless of age
        [~, idx] = min(abs_dt);
    else
        abs_dt_valid = abs_dt;
        abs_dt_valid(~valid) = Inf;
        [~, idx] = min(abs_dt_valid);
    end

    eph = tt_prn(idx, :);

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: kepler_position
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function [pos, clk] = kepler_position(eph, t, constellation, GM, OMEGA_E, C_LIGHT, F)
% Compute satellite position from Keplerian orbital elements.
% Field names verified against BUCU00ROU RINEX navigation file.

    %% ── Select GM by constellation ───────────────────────────────────────
    if strcmp(constellation, 'GPS')
        GM = 3.986005e14;
    else
        GM = 3.986004418e14;
    end

    %% ── Pull ephemeris parameters ────────────────────────────────────────
    A     = eph.sqrtA^2;
    n0    = sqrt(GM / A^3);
    n     = n0 + eph.Delta_n;

    % Time from ephemeris reference epoch
    % Each constellation uses its own time system epoch:
    %   GPS:     1980-01-06 00:00:00 UTC  (IS-GPS-200, Section 3.3.4)
    %   Galileo: 1999-08-22 00:00:00 UTC  (Galileo OS-SIS-ICD, Section 5.1.3)
    %             Note: GST and GPS time differ by a fixed offset (19 leap seconds
    %             at Galileo epoch), but Toe in RINEX nav is already aligned to
    %             GPS week seconds for Galileo — GPS epoch is correct here.
    %   BeiDou:  2006-01-01 00:00:00 UTC  (BDS-SIS-ICD-2.1, Section 4.1)
    %             BDT is offset from GPS time by 14 seconds (GPST = BDT + 14).
    %             Toe in BeiDou RINEX nav is in BDT seconds-of-week, so we must
    %             use the BDT epoch and remove the GPST-BDT offset for tk.
    %             Using GPS epoch here causes tk errors of ~600,000 s — fatal.
    t_utc = datetime(t, 'TimeZone', 'UTC');

    if strcmp(constellation, 'BeiDou')
        % BeiDou Time (BDT) epoch — BDS-SIS-ICD-2.1, Section 4.1.
        % RINEX mixed observation epochs are GPST; BeiDou Toe is BDT SOW.
        bdt_epoch = datetime(2006, 1, 1, 0, 0, 0, 'TimeZone', 'UTC');
        t_sow     = mod(seconds(t_utc - bdt_epoch) - 14, 604800);
    else
        % GPS Time epoch — used for GPS and Galileo (RINEX convention)
        gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
        t_sow     = mod(seconds(t_utc - gps_epoch), 604800);
    end

% Time from ephemeris reference epoch (seconds)
tk = t_sow - eph.Toe;

% Handle week crossover (±half-week boundary)
if tk >  302400, tk = tk - 604800; end
if tk < -302400, tk = tk + 604800; end

    % Mean anomaly
    Mk    = eph.M0 + n * tk;

    %% ── Solve Kepler's equation iteratively ──────────────────────────────
    e  = eph.Eccentricity;
    Ek = Mk;
    for iter = 1:10
        Ek_new = Mk + e * sin(Ek);
        if abs(Ek_new - Ek) < 1e-12, break; end
        Ek = Ek_new;
    end
    Ek = Ek_new;

    %% ── True anomaly ─────────────────────────────────────────────────────
    sin_nu = sqrt(1 - e^2) * sin(Ek) / (1 - e * cos(Ek));
    cos_nu = (cos(Ek) - e)            / (1 - e * cos(Ek));
    nu     = atan2(sin_nu, cos_nu);

    %% ── Argument of latitude ─────────────────────────────────────────────
    phi = nu + eph.omega;

    %% ── Second-order corrections ─────────────────────────────────────────
    du = eph.Cus * sin(2*phi) + eph.Cuc * cos(2*phi);
    dr = eph.Crs * sin(2*phi) + eph.Crc * cos(2*phi);
    di = eph.Cis * sin(2*phi) + eph.Cic * cos(2*phi);

    u  = phi + du;
    r  = A * (1 - e * cos(Ek)) + dr;
    i  = eph.i0 + eph.IDOT * tk + di;

    %% ── Position in orbital plane ────────────────────────────────────────
    x_orb = r * cos(u);
    y_orb = r * sin(u);

    %% ── Corrected longitude of ascending node ────────────────────────────
    if strcmp(constellation, 'BeiDou')
        OMEGA_E_use = 7.2921150e-5;
    else
        OMEGA_E_use = OMEGA_E;
    end

    Omega = eph.OMEGA0 + (eph.OMEGA_DOT - OMEGA_E_use) * tk ...
            - OMEGA_E_use * eph.Toe;

    %% ── ECEF position ────────────────────────────────────────────────────
    pos = [
        x_orb * cos(Omega) - y_orb * cos(i) * sin(Omega);
        x_orb * sin(Omega) + y_orb * cos(i) * cos(Omega);
        y_orb * sin(i)
    ];

    %% ── Satellite clock correction ───────────────────────────────────────
    dt_r = F * e * eph.sqrtA * sin(Ek);
    clk  = (eph.SVClockBias + eph.SVClockDrift * tk + ...
            eph.SVClockDriftRate * tk^2 + dt_r) * C_LIGHT;

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: glonass_position
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function [pos, clk] = glonass_position(eph, t, OMEGA_E, C_LIGHT)
% Compute GLONASS satellite position using numerical integration (RK4).
% Field names verified against BUCU00ROU RINEX navigation file.

    %% ── Initial state from ephemeris ─────────────────────────────────────
    % Position (km -> m) and velocity (km/s -> m/s)
    x0 = [eph.PositionX; eph.PositionY; eph.PositionZ] * 1e3;
    v0 = [eph.VelocityX; eph.VelocityY; eph.VelocityZ] * 1e3;
    a0 = [eph.AccelerationX; eph.AccelerationY; eph.AccelerationZ] * 1e3;

    %% ── Time difference from ephemeris epoch ─────────────────────────────
    % RINEX mixed observation epochs are GPST, while GLONASS broadcast
    % ephemerides are referenced to UTC(SU). For this 2026 dataset,
    % GPST = UTC + 18 s, so convert the observation epoch before propagation.
    t_utc_glo = datetime(t, 'TimeZone', 'UTC') - seconds(18);
    eph_time_utc = datetime(eph.Time, 'TimeZone', 'UTC');
    dt_total = seconds(t_utc_glo - eph_time_utc);

    %% ── Clock correction (GLONASS ICD-2008, Section 3.3.3) ───────────────
    % pr_corr = pr_raw + sat_clk  (pseudorange_correct convention)
    % sat_clk = (+SVClockBias - SVFrequencyBias * dt) * C
    % SVClockBias > 0 means satellite clock is ahead of GLONASS system time,
    % so pseudorange is too short -> add positive correction.
    clk = (eph.SVClockBias - eph.SVFrequencyBias * dt_total) * C_LIGHT;

    %% ── RK4 numerical integration ────────────────────────────────────────
    step = 60; % seconds
    n_steps = ceil(abs(dt_total) / step);
    if n_steps == 0
        pos = x0;
        return;
    end

    h  = dt_total / n_steps;
    xv = [x0; v0];

    for i = 1:n_steps
        k1 = h * glonass_deriv(xv,        a0, OMEGA_E);
        k2 = h * glonass_deriv(xv + k1/2, a0, OMEGA_E);
        k3 = h * glonass_deriv(xv + k2/2, a0, OMEGA_E);
        k4 = h * glonass_deriv(xv + k3,   a0, OMEGA_E);
        xv = xv + (k1 + 2*k2 + 2*k3 + k4) / 6;
    end

    pos = xv(1:3);

end
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: glonass_deriv
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function dxv = glonass_deriv(xv, a_ls, OMEGA_E)
% GLONASS equations of motion for RK4 integration.

    ae    = 6378136.0;   % Earth radius (m)
    mu    = 3.9860044e14;% GM
    J2    = 1.0826257e-3;% second zonal harmonic

    x = xv(1); y = xv(2); z = xv(3);
    r = norm([x; y; z]);

    % Gravitational acceleration with J2 correction
    factor = 1.5 * J2 * (ae/r)^2;
    z_r2   = (z/r)^2;

    ax = -mu/r^3 * x * (1 - factor*(5*z_r2 - 1)) ...
         + OMEGA_E^2 * x + 2*OMEGA_E*xv(5) + a_ls(1);
    ay = -mu/r^3 * y * (1 - factor*(5*z_r2 - 1)) ...
         + OMEGA_E^2 * y - 2*OMEGA_E*xv(4) + a_ls(2);
    az = -mu/r^3 * z * (1 - factor*(5*z_r2 - 3)) + a_ls(3);

    dxv = [xv(4); xv(5); xv(6); ax; ay; az];

end
```

## Stage 2 - Identification

### `stage2_identification/chi_squared_test.m`

Computes the weighted post-fit chi-squared consistency statistic and its degree-of-freedom-dependent threshold.

```matlab
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
```

### `stage2_identification/raim_fde.m`

Performs iterative RAIM fault detection and exclusion using weighted least-squares residual consistency.

```matlab
function result = raim_fde(obs_epoch, nav, cfg, t)
% RAIM_FDE  Receiver Autonomous Integrity Monitoring — Fault Detection & Exclusion
%
% Iterative satellite exclusion loop for identifying spoofed/faulty satellites.
% Uses wls_solver and chi_squared_test as building blocks.
%
% DESIGN NOTE (from chi_squared_test validation, epoch 500):
%   The chi-squared test alone CANNOT detect sophisticated multi-satellite
%   spoofing because the WLS solver absorbs the spoofing offset into its
%   position/clock estimate, making post-fit residuals internally consistent.
%   raim_fde therefore provides a CANDIDATE LIST, not a definitive verdict.
%   inter_constellation.m provides the definitive identification step.
%
% ALGORITHM:
%   1. Compute full WLS solution with all N satellites → chi-squared test
%   2. If test passes → return all satellites as trusted (no fault detected)
%   3. If test fails → exclusion loop:
%      a. For each satellite i, exclude it and recompute WLS
%      b. Record chi-squared statistic of the reduced solution
%      c. Satellite whose removal MOST REDUCES the statistic is flagged
%   4. Remove flagged satellite, repeat until test passes or min_sats reached
%
% INPUTS:
%   obs_epoch  struct  — single-epoch observables (all constellations)
%                        fields: .prn, .constellation, .pseudorange, .cn0
%   nav        struct  — navigation/ephemeris data (output of rinex_read_nav)
%   cfg        struct  — configuration (output of config.m)
%   t          datetime — current epoch timestamp (UTC)
%
% OUTPUT:
%   result     struct with fields:
%     .trusted_sats     [Nx1 cell]  — {constellation, prn} pairs declared healthy
%     .spoofed_sats     [Mx1 cell]  — {constellation, prn} pairs excluded by FDE
%     .fault_detected   logical     — true if chi-squared failed on full set
%     .final_chi2_stat  double      — chi-squared statistic of final solution
%     .final_chi2_thresh double     — threshold used
%     .final_pos_ecef   [3x1]       — ECEF position from trusted sats
%     .final_clk_bias   double      — receiver clock bias (m) from trusted sats
%     .n_trusted        int         — number of trusted satellites
%     .n_excluded       int         — number of satellites excluded by FDE
%     .iterations       int         — number of FDE iterations performed
%     .exclusion_log    struct array — per-iteration exclusion records

% -------------------------------------------------------------------------
% 0. Input validation
% -------------------------------------------------------------------------
if nargin < 4
    error('raim_fde: requires obs_epoch, nav, cfg, t');
end

n_obs = length(obs_epoch.prn);
if n_obs < cfg.identify.min_sats
    warning('raim_fde: only %d satellites — below minimum %d. Returning empty.', ...
        n_obs, cfg.identify.min_sats);
    result = empty_result(obs_epoch);
    return;
end

% -------------------------------------------------------------------------
% 1. Build measurement matrix from obs_epoch
%    Each row: [prn, constellation_id, pseudorange, cn0]
% -------------------------------------------------------------------------
sats = build_sat_list(obs_epoch, nav, cfg, t);

% Remove satellites for which we could not compute a valid position
valid_mask = [sats.valid];
sats = sats(valid_mask);
n_valid = length(sats);

if n_valid < cfg.identify.min_sats
    warning('raim_fde: only %d valid satellite positions — below minimum %d.', ...
        n_valid, cfg.identify.min_sats);
    result = empty_result(obs_epoch);
    return;
end

% -------------------------------------------------------------------------
% 2. Initial full-set WLS solution + chi-squared test
% -------------------------------------------------------------------------
[pos0, clk0, residuals0, weights0] = run_wls(sats, cfg);
chi2_result0 = chi_squared_test(residuals0, weights0, 4, cfg);

result.fault_detected   = ~chi2_result0.passed;
result.final_chi2_stat  = chi2_result0.test_stat;
result.final_chi2_thresh = chi2_result0.threshold;
result.iterations       = 0;
result.exclusion_log    = struct([]);

if chi2_result0.passed
    % No fault detected — all valid satellites are trusted
    result.trusted_sats   = extract_sat_ids(sats);
    result.spoofed_sats   = {};
    result.final_pos_ecef = pos0;
    result.final_clk_bias = clk0;
    result.n_trusted      = n_valid;
    result.n_excluded     = 0;
    return;
end

% -------------------------------------------------------------------------
% 3. FDE exclusion loop
% -------------------------------------------------------------------------
% Working satellite set — starts full, satellites removed iteratively
active_sats   = sats;
excluded_sats = {};   % accumulates excluded {constellation, prn} pairs

max_iterations = n_valid - cfg.identify.min_sats;   % can't go below min_sats

for iter = 1:max_iterations

    n_active = length(active_sats);

    if n_active < cfg.identify.min_sats + 1
        % Cannot exclude any more without going below minimum
        break;
    end

    % --- Try excluding each satellite and record chi-squared statistic ---
    chi2_stats = nan(n_active, 1);

    for i = 1:n_active
        subset = active_sats([1:i-1, i+1:end]);   % all except satellite i
        if length(subset) < cfg.identify.min_sats
            continue;   % skip: would drop below minimum
        end
        [~, ~, res_i, weights_i] = run_wls(subset, cfg);
        chi2_i = chi_squared_test(res_i, weights_i, 4, cfg);
        chi2_stats(i) = chi2_i.test_stat;
    end

    % --- Find satellite whose removal most reduces the test statistic ---
    [min_stat, best_idx] = min(chi2_stats);

    if isnan(min_stat)
        % All subsets were too small
        break;
    end

    % Log this exclusion
    log_entry.iteration       = iter;
    log_entry.excluded_prn    = active_sats(best_idx).prn;
    log_entry.excluded_const  = active_sats(best_idx).constellation;
    log_entry.chi2_before     = chi2_result0.test_stat;   % full set statistic
    log_entry.chi2_after      = min_stat;
    log_entry.threshold       = chi2_result0.threshold;

    if isempty(result.exclusion_log)
        result.exclusion_log = log_entry;
    else
        result.exclusion_log(end+1) = log_entry;
    end

    % Record excluded satellite
    excluded_sats{end+1} = struct( ...
        'prn',           active_sats(best_idx).prn, ...
        'constellation', active_sats(best_idx).constellation ...
    ); %#ok<AGROW>

    % Remove it from active set
    active_sats(best_idx) = [];
    result.iterations = iter;

    % --- Re-test with reduced set ---
    [pos_iter, clk_iter, res_iter, weights_iter] = run_wls(active_sats, cfg);
    chi2_iter = chi_squared_test(res_iter, weights_iter, 4, cfg);

    result.final_chi2_stat   = chi2_iter.test_stat;
    result.final_chi2_thresh = chi2_iter.threshold;
    result.final_pos_ecef    = pos_iter;
    result.final_clk_bias    = clk_iter;

    if chi2_iter.passed
        % Test passes — stop excluding
        break;
    end

    % Update reference statistic for next iteration log
    chi2_result0 = chi2_iter;

end

% -------------------------------------------------------------------------
% 4. Assemble final result
% -------------------------------------------------------------------------
result.trusted_sats   = extract_sat_ids(active_sats);
result.spoofed_sats   = excluded_sats;
result.n_trusted      = length(active_sats);
result.n_excluded     = length(excluded_sats);

% Guard: if we never set final_pos_ecef (e.g. loop never ran WLS), use full set
if ~isfield(result, 'final_pos_ecef') || isempty(result.final_pos_ecef)
    result.final_pos_ecef = pos0;
    result.final_clk_bias = clk0;
end

% -------------------------------------------------------------------------
% 5. Verbose logging
% -------------------------------------------------------------------------
if cfg.verbose
    fprintf('[RAIM-FDE] t=%s | fault_detected=%d | excluded=%d | trusted=%d\n', ...
        string(datetime(t, 'Format', 'HH:mm:ss')), result.fault_detected, result.n_excluded, result.n_trusted);
    if result.n_excluded > 0
        fprintf('[RAIM-FDE]   Chi2: %.2f → %.2f (thresh=%.2f)\n', ...
            result.exclusion_log(1).chi2_before, ...
            result.final_chi2_stat, ...
            result.final_chi2_thresh);
        for k = 1:length(excluded_sats)
            fprintf('[RAIM-FDE]   Excluded: %s PRN %d\n', ...
                excluded_sats{k}.constellation, excluded_sats{k}.prn);
        end
    end
end

end % main function


%% =========================================================================
%  LOCAL HELPER: build_sat_list
%  Computes satellite ECEF positions and corrected pseudoranges for all
%  satellites in obs_epoch. Returns struct array with one entry per satellite.
% =========================================================================
function sats = build_sat_list(obs_epoch, nav, cfg, t)
% Calls sat_position(nav, prn, constellation, t) and
% pseudorange_correct(pr_raw, sat_pos, sat_clk, rec_pos, t, nav, constellation, cfg)

n = length(obs_epoch.prn);
sats = struct( ...
    'prn',           cell(1,n), ...
    'constellation', cell(1,n), ...
    'pseudorange',   cell(1,n), ...
    'pos_ecef',      cell(1,n), ...
    'clk_corr',      cell(1,n), ...
    'weight',        cell(1,n), ...
    'valid',         cell(1,n)  ...
);

for i = 1:n
    prn   = obs_epoch.prn(i);
    const = obs_epoch.constellation{i};
    pr    = obs_epoch.pseudorange(i);

    sats(i).prn           = prn;
    sats(i).constellation = const;
    sats(i).pseudorange   = pr;
    sats(i).valid         = false;

    % NOTE: BeiDou PRN 33 (BDS-3 IGSO) was previously flagged as corrupted, but
    % residual analysis over 843 epochs (std 1.9 m, comparable to healthy PRN 24)
    % confirms it is healthy in this 17-May-2026 dataset. It is included.
    % IGSO ephemeris is genuinely less accurate than MEO and can degrade near
    % maneuvers; verified clean here. See thesis Ch4/Ch6.

    if isnan(pr) || pr <= 0
        continue;
    end

    try
        % Transmit-time corrected measurement. Returns sat_pos at TRANSMIT
        % time and sat_clk for traceability; geometry below is consistent
        % with the correction. rec_approx = cfg.ref_pos.
        [pr_corr, sat_pos, sat_clk] = corrected_pseudorange(pr, prn, const, ...
                                          t, cfg.ref_pos(:), nav, cfg);
        if isnan(pr_corr)
            continue;   % elevation-masked or invalid — skip
        end

        switch const
            case 'GPS';     sigma2 = cfg.ekf.meas_noise_GPS;
            case 'Galileo'; sigma2 = cfg.ekf.meas_noise_Galileo;
            case 'BeiDou';  sigma2 = cfg.ekf.meas_noise_BeiDou;
            case 'GLONASS'; sigma2 = cfg.ekf.meas_noise_GLONASS;
            otherwise;      sigma2 = 500.0;
        end

        sats(i).pos_ecef    = sat_pos;
        sats(i).clk_corr    = sat_clk;
        sats(i).pseudorange = pr_corr;
        sats(i).weight      = 1.0 / sigma2;
        sats(i).valid       = true;

    catch ME
        if cfg.verbose
            fprintf('[RAIM-FDE] Skipping %s PRN %d: %s\n', const, prn, ME.message);
        end
    end
end

end % build_sat_list


%% =========================================================================
%  LOCAL HELPER: get_ephemeris
%  Retrieves the closest valid ephemeris for a given satellite and epoch.
% =========================================================================
function eph = get_ephemeris(nav, constellation, prn, t)
% nav.(constellation) is a single struct with fields:
%   .prn  [N×1 double]   — PRN for each record
%   .toe  [N×1 datetime] — time of ephemeris for each record
%   .data [N×32 timetable] — all ephemeris parameters, one row per record

eph = [];

switch constellation
    case 'GPS';     field = 'GPS';
    case 'Galileo'; field = 'Galileo';
    case 'BeiDou';  field = 'BeiDou';
    case 'GLONASS'; field = 'GLONASS';
    otherwise;      return;
end

if ~isfield(nav, field) || isempty(nav.(field))
    return;
end

nav_const = nav.(field);   % single struct

% Find rows matching this PRN
prn_mask = (nav_const.prn == prn);
if ~any(prn_mask)
    return;
end

% Among matching rows, find the one whose toe is closest to t
toe_candidates = nav_const.toe(prn_mask);       % datetime vector
t_dt = datetime(t, 'TimeZone', 'UTC');
try
    toe_utc = datetime(toe_candidates, 'TimeZone', 'UTC');
catch
    toe_utc = toe_candidates;
end
dt_sec = abs(seconds(toe_utc - t_dt));
[~, best_local] = min(dt_sec);

% Map back to global row index
global_indices = find(prn_mask);
best_row = global_indices(best_local);

% Extract that row from the timetable as a struct
row = nav_const.data(best_row, :);
vars = row.Properties.VariableNames;

eph = struct();
eph.prn = prn;
eph.toe = nav_const.toe(best_row);

for v = 1:length(vars)
    val = row.(vars{v});
    if iscell(val), val = val{1}; end
    eph.(vars{v}) = val;
end

% Expose Toe as GPS seconds-of-week (expected by sat_position)
gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
try
    toe_utc2 = datetime(eph.toe, 'TimeZone', 'UTC');
catch
    toe_utc2 = eph.toe;
end
eph.Toe = mod(seconds(toe_utc2 - gps_epoch), 604800);

end % get_ephemeris


%% =========================================================================
%  LOCAL HELPER: run_wls
%  Runs WLS solver on a struct array of satellites.
%  Returns position, clock bias, post-fit residuals, H matrix, and W matrix.
% =========================================================================
function [pos_ecef, clk_bias, residuals, weights] = run_wls(sats, cfg)
% Returns weights vector (not H/W matrices) for use with chi_squared_test

n = length(sats);

pr_vec  = zeros(n, 1);
sat_pos = zeros(n, 3);
weights = zeros(n, 1);

for i = 1:n
    pr_vec(i)    = sats(i).pseudorange;
    sat_pos(i,:) = sats(i).pos_ecef(:)';
    weights(i)   = sats(i).weight;
end

% Correct signature: wls_solver(pseudoranges, sat_positions, weights, pos_init)
[pos_ecef, clk_bias, residuals] = wls_solver(pr_vec, sat_pos, weights, cfg.ref_pos);

end % run_wls


%% =========================================================================
%  LOCAL HELPER: extract_sat_ids
%  Returns a cell array of structs {prn, constellation} from a sats array.
% =========================================================================
function ids = extract_sat_ids(sats)
ids = cell(1, length(sats));
for i = 1:length(sats)
    ids{i} = struct('prn', sats(i).prn, 'constellation', sats(i).constellation);
end
end % extract_sat_ids


%% =========================================================================
%  LOCAL HELPER: empty_result
%  Returns a well-formed empty result when insufficient satellites available.
% =========================================================================
function result = empty_result(obs_epoch)
result.trusted_sats    = {};
result.spoofed_sats    = {};
result.fault_detected  = false;
result.final_chi2_stat = NaN;
result.final_chi2_thresh = NaN;
result.final_pos_ecef  = [NaN; NaN; NaN];
result.final_clk_bias  = NaN;
result.n_trusted       = 0;
result.n_excluded      = 0;
result.iterations      = 0;
result.exclusion_log   = struct([]);
% Copy satellite IDs from input for traceability
for i = 1:length(obs_epoch.prn)
    result.trusted_sats{i} = struct( ...
        'prn', obs_epoch.prn(i), ...
        'constellation', obs_epoch.constellation{i});
end
end % empty_result
```

### `stage2_identification/inter_constellation.m`

Compares independently solved constellation positions to identify constellation-level disagreement.

```matlab
function result = inter_constellation(obs_epoch, nav, cfg, t)
% INTER_CONSTELLATION  Cross-constellation position consistency check
%
% The PRIMARY identification tool for multi-satellite spoofing.
%
% MOTIVATION (from chi_squared_test validation, epoch 500):
%   Sophisticated spoofing of an entire constellation shifts all pseudoranges
%   coherently, so the WLS solver absorbs the error into its position estimate.
%   Post-fit residuals look internally consistent → chi-squared test PASSES.
%   However, an independent constellation (e.g. Galileo, BeiDou) that was NOT
%   spoofed will yield a position solution displaced by 100m+ from the spoofed
%   GPS solution. This displacement is detectable and unambiguous.
%
% ALGORITHM:
%   1. For each constellation with >= min_sats satellites, compute an
%      independent WLS position solution.
%   2. Compute pairwise ECEF distances between constellation solutions.
%   3. Flag constellations whose solution is an outlier relative to the
%      majority — using a median-based robust consensus.
%   4. Return per-constellation flags and the consensus reference position.
%
% INPUTS:
%   obs_epoch  struct  — single-epoch observables (all constellations)
%                        fields: .prn, .constellation, .pseudorange
%   nav        struct  — navigation/ephemeris data
%   cfg        struct  — configuration (output of config.m)
%   t          datetime — current epoch timestamp (UTC)
%
% OUTPUT:
%   result     struct with fields:
%     .constellation_solutions  struct array — per-constellation position results
%     .pairwise_distances       [NxN double] — ECEF distances between solutions (m)
%     .outlier_flags            [Nx1 logical] — true = outlier constellation
%     .outlier_constellations   cell          — names of flagged constellations
%     .consensus_pos_ecef       [3x1]         — robust consensus position
%     .max_pairwise_dist        double        — largest pairwise distance (m)
%     .n_constellations_solved  int           — number of independent solutions
%     .spoofing_suspected       logical       — true if any outlier found

% -------------------------------------------------------------------------
% 0. Parameters
% -------------------------------------------------------------------------
% Position consistency threshold: disagreement beyond this → spoofing suspected
% Set relative to detection sensitivity (should be >> measurement noise but
% << expected spoofing offset of ~100m).
% Value: 3*sigma where sigma ~ 18.3m → ~55m; use 50m as conservative round number.
CONSISTENCY_THRESHOLD_M = cfg.identify.inter_const_threshold;   % default: 50.0 m

MIN_SATS_PER_CONST = cfg.identify.min_sats_per_constellation;   % default: 4

% -------------------------------------------------------------------------
% 1. Group satellites by constellation
% -------------------------------------------------------------------------
constellations = unique(obs_epoch.constellation);
n_const = length(constellations);

const_solutions = struct( ...
    'name',     {}, ...
    'pos_ecef', {}, ...
    'clk_bias', {}, ...
    'n_sats',   {}, ...
    'solved',   {} ...
);

n_solved = 0;

for c = 1:n_const
    cname = constellations{c};
    mask  = strcmp(obs_epoch.constellation, cname);

    % Extract this constellation's observables
    sub_obs.prn           = obs_epoch.prn(mask);
    sub_obs.constellation = obs_epoch.constellation(mask);
    sub_obs.pseudorange   = obs_epoch.pseudorange(mask);
    if isfield(obs_epoch, 'cn0')
        sub_obs.cn0 = obs_epoch.cn0(mask);
    end

    n_sats = sum(mask);
    if n_sats < MIN_SATS_PER_CONST
        if cfg.verbose
            fprintf('[INTER_CONST] %s: only %d sats — skipping (need %d)\n', ...
                cname, n_sats, MIN_SATS_PER_CONST);
        end
        continue;
    end

    % Build satellite list and compute positions
    sats = build_sat_list_const(sub_obs, nav, cfg, t, cname);
    valid_sats = sats([sats.valid]);

    if length(valid_sats) < MIN_SATS_PER_CONST
        if cfg.verbose
            fprintf('[INTER_CONST] %s: only %d valid positions — skipping\n', ...
                cname, length(valid_sats));
        end
        continue;
    end

    % Independent WLS solution for this constellation
    try
        [pos_c, clk_c] = run_const_wls(valid_sats, cfg);

        if any(isnan(pos_c))
            continue;
        end

        n_solved = n_solved + 1;
        const_solutions(n_solved).name     = cname;
        const_solutions(n_solved).pos_ecef = pos_c;
        const_solutions(n_solved).clk_bias = clk_c;
        const_solutions(n_solved).n_sats   = length(valid_sats);
        const_solutions(n_solved).solved   = true;

        if cfg.verbose
            % Keep verbose logging simple; test scripts print geodetic details.
            fprintf('[INTER_CONST] %s: solved clk=%.1fm n=%d\n', ...
                cname, clk_c, length(valid_sats));
        end

    catch ME
        if cfg.verbose
            fprintf('[INTER_CONST] %s: WLS failed — %s\n', cname, ME.message);
        end
    end
end

% -------------------------------------------------------------------------
% 2. Handle degenerate case: fewer than 2 constellations solved
% -------------------------------------------------------------------------
if n_solved < 2
    if cfg.verbose
        fprintf('[INTER_CONST] Only %d constellation(s) solved — cannot compare.\n', n_solved);
    end
    result = build_degenerate_result(const_solutions, n_solved);
    return;
end

% -------------------------------------------------------------------------
% 3. Pairwise ECEF distance matrix
% -------------------------------------------------------------------------
dist_matrix = zeros(n_solved, n_solved);
for i = 1:n_solved
    for j = i+1:n_solved
        d = norm(const_solutions(i).pos_ecef - const_solutions(j).pos_ecef);
        dist_matrix(i,j) = d;
        dist_matrix(j,i) = d;
    end
end

% -------------------------------------------------------------------------
% 4. Robust outlier detection via median pairwise distance
%
% For each constellation i, compute the MEDIAN distance to all other
% constellations. Outliers have systematically large median distances,
% meaning they disagree with the majority.
%
% This is robust against a single honest constellation disagring with
% multiple spoofed ones — the median naturally sides with the majority.
% -------------------------------------------------------------------------
median_dists = zeros(n_solved, 1);
for i = 1:n_solved
    other_dists = dist_matrix(i, [1:i-1, i+1:n_solved]);
    median_dists(i) = median(other_dists);
end

outlier_flags = median_dists > CONSISTENCY_THRESHOLD_M;
outlier_names = {};
for i = 1:n_solved
    if outlier_flags(i)
        outlier_names{end+1} = const_solutions(i).name; %#ok<AGROW>
    end
end

% -------------------------------------------------------------------------
% 5. Consensus position — average of non-outlier constellations (robust mean)
% -------------------------------------------------------------------------
consensus_indices = find(~outlier_flags);
if isempty(consensus_indices)
    % All constellations flagged as outliers — impossible to determine truth.
    % Fall back to the pair with the minimum pairwise distance (most agreement).
    [~, best_pair_lin] = min(dist_matrix(dist_matrix > 0));
    [r, ~] = ind2sub([n_solved, n_solved], best_pair_lin);
    consensus_indices = r;
    warning('inter_constellation: all constellations flagged — falling back to best pair.');
end

consensus_positions = zeros(3, length(consensus_indices));
for k = 1:length(consensus_indices)
    consensus_positions(:,k) = const_solutions(consensus_indices(k)).pos_ecef;
end
consensus_pos = mean(consensus_positions, 2);

% -------------------------------------------------------------------------
% 6. Assemble result
% -------------------------------------------------------------------------
result.constellation_solutions = const_solutions;
result.pairwise_distances      = dist_matrix;
result.outlier_flags           = outlier_flags;
result.outlier_constellations  = outlier_names;
result.consensus_pos_ecef      = consensus_pos;
result.max_pairwise_dist       = max(dist_matrix(:));
result.n_constellations_solved = n_solved;
result.spoofing_suspected      = any(outlier_flags);
result.consistency_threshold   = CONSISTENCY_THRESHOLD_M;

% -------------------------------------------------------------------------
% 7. Verbose summary
% -------------------------------------------------------------------------
if cfg.verbose
    fprintf('[INTER_CONST] t=%s | solved=%d | max_dist=%.1fm | threshold=%.1fm\n', ...
        string(datetime(t, 'Format', 'HH:mm:ss')), n_solved, result.max_pairwise_dist, CONSISTENCY_THRESHOLD_M);
    if result.spoofing_suspected
        fprintf('[INTER_CONST]   *** SPOOFING SUSPECTED — outlier constellation(s): %s\n', ...
            strjoin(outlier_names, ', '));
    else
        fprintf('[INTER_CONST]   All constellations consistent.\n');
    end
end

end % main function


%% =========================================================================
%  LOCAL HELPER: build_sat_list_const
%  Computes satellite positions for a single constellation's observables.
% =========================================================================
function sats = build_sat_list_const(sub_obs, nav, cfg, t, constellation)

n = length(sub_obs.prn);
sats = struct('prn', cell(1,n), 'pos_ecef', cell(1,n), ...
              'pseudorange', cell(1,n), 'weight', cell(1,n), 'valid', cell(1,n));

switch constellation
    case 'GPS';     sigma2 = cfg.ekf.meas_noise_GPS;
    case 'Galileo'; sigma2 = cfg.ekf.meas_noise_Galileo;
    case 'BeiDou';  sigma2 = cfg.ekf.meas_noise_BeiDou;
    case 'GLONASS'; sigma2 = cfg.ekf.meas_noise_GLONASS;
    otherwise;      sigma2 = 500.0;
end

for i = 1:n
    prn = sub_obs.prn(i);
    pr  = sub_obs.pseudorange(i);

    sats(i).prn         = prn;
    sats(i).pseudorange = pr;
    sats(i).weight      = 1.0 / sigma2;
    sats(i).valid       = false;

    if isnan(pr) || pr <= 0
        continue;
    end

    try
        % Transmit-time corrected measurement. Returns sat_pos at TRANSMIT
        % time, stored below so the per-constellation WLS geometry is
        % consistent with the correction. rec_approx = cfg.ref_pos.
        [pr_corr, sat_pos] = corrected_pseudorange(pr, prn, constellation, ...
                                 t, cfg.ref_pos(:), nav, cfg);
        if isnan(pr_corr), continue; end   % elevation-masked or invalid — skip

        sats(i).pos_ecef    = sat_pos;
        sats(i).pseudorange = pr_corr;
        sats(i).valid       = true;

    catch
        % silently skip — already filtered above
    end
end

end % build_sat_list_const


%% =========================================================================
%  LOCAL HELPER: get_ephemeris_const
% =========================================================================
function eph = get_ephemeris_const(nav, constellation, prn, t)
% nav.(constellation) is a single struct with fields:
%   .prn  [N×1 double]
%   .toe  [N×1 datetime]
%   .data [N×32 timetable]

eph = [];

switch constellation
    case 'GPS';     field = 'GPS';
    case 'Galileo'; field = 'Galileo';
    case 'BeiDou';  field = 'BeiDou';
    case 'GLONASS'; field = 'GLONASS';
    otherwise;      return;
end

if ~isfield(nav, field) || isempty(nav.(field))
    return;
end

nav_const = nav.(field);

prn_mask = (nav_const.prn == prn);
if ~any(prn_mask), return; end

toe_candidates = nav_const.toe(prn_mask);
t_dt = datetime(t, 'TimeZone', 'UTC');
try
    toe_utc = datetime(toe_candidates, 'TimeZone', 'UTC');
catch
    toe_utc = toe_candidates;
end
dt_sec = abs(seconds(toe_utc - t_dt));
[~, best_local] = min(dt_sec);

global_indices = find(prn_mask);
best_row = global_indices(best_local);

row  = nav_const.data(best_row, :);
vars = row.Properties.VariableNames;

eph = struct();
eph.prn = prn;
eph.toe = nav_const.toe(best_row);

for v = 1:length(vars)
    val = row.(vars{v});
    if iscell(val), val = val{1}; end
    eph.(vars{v}) = val;
end

gps_epoch = datetime(1980, 1, 6, 0, 0, 0, 'TimeZone', 'UTC');
try
    toe_utc2 = datetime(eph.toe, 'TimeZone', 'UTC');
catch
    toe_utc2 = eph.toe;
end
eph.Toe = mod(seconds(toe_utc2 - gps_epoch), 604800);

end % get_ephemeris_const


%% =========================================================================
%  LOCAL HELPER: run_const_wls
%  Runs WLS for a single constellation's valid satellite struct array.
% =========================================================================
function [pos_ecef, clk_bias] = run_const_wls(valid_sats, cfg)

n = length(valid_sats);
pr_vec  = zeros(n, 1);
sat_pos = zeros(n, 3);
weights = zeros(n, 1);

for i = 1:n
    pr_vec(i)    = valid_sats(i).pseudorange;
    sat_pos(i,:) = valid_sats(i).pos_ecef(:)';
    weights(i)   = valid_sats(i).weight;
end

% Correct signature: wls_solver(pseudoranges, sat_positions, weights, pos_init)
[pos_ecef, clk_bias] = wls_solver(pr_vec, sat_pos, weights, cfg.ref_pos);

end % run_const_wls


%% =========================================================================
%  LOCAL HELPER: build_degenerate_result
% =========================================================================
function result = build_degenerate_result(const_solutions, n_solved)

result.constellation_solutions = const_solutions;
result.pairwise_distances      = zeros(n_solved, n_solved);
result.outlier_flags           = false(n_solved, 1);
result.outlier_constellations  = {};
result.consensus_pos_ecef      = [NaN; NaN; NaN];
result.max_pairwise_dist       = NaN;
result.n_constellations_solved = n_solved;
result.spoofing_suspected      = false;
result.consistency_threshold   = NaN;

if n_solved == 1
    result.consensus_pos_ecef = const_solutions(1).pos_ecef;
end

end % build_degenerate_result

%% ARCHITECTURAL BOUNDARY CONDITIONS — inter_constellation reliability limits
%
% This detector uses a median-based majority vote across constellation solutions.
% Reliability depends on the ratio of spoofed to honest constellations:
%
%   1 spoofed,  3 honest  → RELIABLE
%     The spoofed constellation disagrees with all 3 honest ones.
%     Its median pairwise distance exceeds threshold → correctly flagged.
%     Covers Scenarios 1 (GPS), 2 (Galileo), 3 (BeiDou).
%
%   2 spoofed,  2 honest  → UNRELIABLE
%     Both groups agree internally, disagree with each other.
%     All 4 median distances exceed threshold → all constellations flagged.
%     Fallback to best pair fires — result is indeterminate.
%     Covers Scenario 4 (GPS+GLONASS), Scenario 5 (GPS+Galileo).
%
%   3 spoofed,  1 honest  → INVERTED — dangerous failure mode
%     The 3 spoofed agree with each other; the honest one disagrees with all 3.
%     The honest constellation has the largest median distance → flagged as outlier.
%     Spoofed constellations declared trusted. Detector inverts.
%
% CONSEQUENCE FOR ARCHITECTURE (Chapter 4):
%   inter_constellation is the PRIMARY identifier only for single-constellation
%   attacks. For multi-constellation attacks (Scenarios 4 and 5), RAIM-FDE
%   and Stage 1 detection carry the identification burden.
%   classify_spoofed_sats fuses all evidence sources precisely because no
%   single detector covers all attack scenarios.
%
% REFERENCE:
%   Majority-vote geometric median outlier detection — see Rousseeuw & Leroy
%   (1987), "Robust Regression and Outlier Detection", Wiley, Chapter 7.
%   RAIM reliability under multiple faults — ESA Navipedia, RAIM Fundamentals,
%   https://gssc.esa.int/navipedia/index.php/RAIM_Fundamentals
```

### `stage2_identification/classify_spoofed_sats.m`

Fuses RAIM and inter-constellation evidence into trusted, suspect, and spoofed satellite classifications.

```matlab
function result = classify_spoofed_sats(raim_result, inter_result, obs_epoch, cfg)
% CLASSIFY_SPOOFED_SATS  Fuse RAIM-FDE and inter-constellation evidence
%
% Stage 2 final step: combines evidence from raim_fde and inter_constellation
% to produce a definitive per-satellite classification:
%   - 'trusted'  → satellite is healthy; include in Stage 3/4
%   - 'spoofed'  → satellite is flagged; exclude in Stage 3/4
%   - 'suspect'  → inconclusive; use with elevated noise in Stage 3/4
%
% FUSION LOGIC:
%   Evidence is interpreted contextually, not by simple detector voting.
%
%   Mode A: no constellation outlier from inter_constellation
%     This is RAIM-FDE's intended single-fault regime.
%     RAIM exclusions are treated as real single-satellite faults.
%
%   Mode B-single: inter_constellation flags one constellation, but RAIM
%     cleanly isolates one satellite in that same constellation.
%     The constellation outlier is interpreted as contamination from a
%     single unresolved measurement fault. Only the RAIM-isolated satellite
%     is classified spoofed/faulty.
%
%   Mode B-constellation: inter_constellation flags one or more constellations
%     and the single-satellite override does not apply.
%     Inter-constellation is the primary arbiter. Satellites in flagged
%     constellations are excluded as spoofed; RAIM-only exclusions outside
%     flagged constellations are demoted to suspect because multi-fault WLS
%     distortion can create collateral FDE artefacts.
% INPUTS:
%   raim_result    struct  — output of raim_fde()
%   inter_result   struct  — output of inter_constellation()
%   obs_epoch      struct  — single-epoch observables
%                            fields: .prn, .constellation, .pseudorange
%   cfg            struct  — configuration
%
% OUTPUT:
%   result         struct with fields:
%     .classifications    struct array — per-satellite verdict
%       .prn              int
%       .constellation    char
%       .status           'trusted' | 'spoofed' | 'suspect'
%       .evidence         cell of strings — evidence sources
%       .weight_factor    double — multiplier applied to meas noise in EKF
%     .trusted_mask       logical [N x 1] — index into obs_epoch
%     .spoofed_mask       logical [N x 1]
%     .suspect_mask       logical [N x 1]
%     .n_trusted          int
%     .n_spoofed          int
%     .n_suspect          int
%     .attack_type        char — 'none'|'single_sat'|'constellation'|'multi_const'
%     .recommended_action char — action for Stage 3

% -------------------------------------------------------------------------
% 0. Constants
% -------------------------------------------------------------------------
SUSPECT_WEIGHT_FACTOR = 5.0;    % inflate noise for suspect satellites in EKF
SPOOFED_WEIGHT_FACTOR = 1e6;    % effectively exclude from WLS via near-zero weight

n_obs = length(obs_epoch.prn);

% -------------------------------------------------------------------------
% 1. Build per-satellite evidence maps from raim_fde
% -------------------------------------------------------------------------
% Map: prn+constellation → 'excluded_by_raim' flag
raim_excluded = containers.Map('KeyType','char','ValueType','logical');

for k = 1:length(raim_result.spoofed_sats)
    s = raim_result.spoofed_sats{k};
    key = make_key(s.constellation, s.prn);
    raim_excluded(key) = true;
end

% -------------------------------------------------------------------------
% 2. Build per-constellation outlier map from inter_constellation
% -------------------------------------------------------------------------
inter_flagged_consts = inter_result.outlier_constellations;  % cell of names

% -------------------------------------------------------------------------
% 2b. Pre-compute single_sat_override BEFORE the per-satellite loop
%
% This flag controls per-satellite labelling, not just attack_type.
% When true: the inter-const outlier is explained by a single RAIM-isolated
% fault contaminating the constellation WLS — per-satellite labels must
% reflect single_sat semantics (only the RAIM-excluded satellite is spoofed).
%
% All four conditions must hold (see attack_type block for rationale):
% -------------------------------------------------------------------------
single_sat_override = false;
if inter_result.spoofing_suspected && length(inter_flagged_consts) == 1
    flagged_const_pre = inter_flagged_consts{1};
    n_raim_in_flagged_pre = 0;
    for kk = 1:length(raim_result.spoofed_sats)
        if strcmp(raim_result.spoofed_sats{kk}.constellation, flagged_const_pre)
            n_raim_in_flagged_pre = n_raim_in_flagged_pre + 1;
        end
    end
    single_sat_override = raim_result.fault_detected      && ...
                          raim_result.n_excluded <= 2     && ...
                          n_raim_in_flagged_pre >= 1      && ...
                          n_raim_in_flagged_pre <= 1;
end

% -------------------------------------------------------------------------
% 3. Classify each satellite in obs_epoch
% -------------------------------------------------------------------------
classifications(n_obs) = struct( ...
    'prn', 0, 'constellation', '', 'status', '', ...
    'evidence', {{}}, 'weight_factor', 1.0);

trusted_mask = false(n_obs, 1);
spoofed_mask = false(n_obs, 1);
suspect_mask = false(n_obs, 1);

for i = 1:n_obs
    prn   = obs_epoch.prn(i);
    const = obs_epoch.constellation{i};
    key   = make_key(const, prn);

    evidence = {};
    status   = 'trusted';
    wfactor  = 1.0;

    % --- Evidence (a): RAIM-FDE explicit exclusion ---
    raim_flag = isKey(raim_excluded, key) && raim_excluded(key);
    if raim_flag
        evidence{end+1} = 'raim_fde:excluded';
    end

    % --- Evidence (b): inter-constellation outlier constellation ---
    inter_flag = any(strcmp(inter_flagged_consts, const));
    if inter_flag
        evidence{end+1} = sprintf('inter_constellation:outlier_const=%s', const);
    end

    % --- Determine status (context-aware two-mode fusion) ---
    %
    % MODE A — No constellation flagged by inter-constellation:
    %   RAIM is operating in its designed single-fault regime.
    %   RAIM exclusions are trusted as real single-satellite faults.
    %
    % MODE B — One or more constellations flagged by inter-constellation:
    %   A constellation-level attack is suspected.
    %   Inter-constellation is the primary arbiter.
    %   RAIM-only exclusions OUTSIDE the flagged constellation are
    %   collateral artefacts of the multi-fault WLS distortion — mark suspect.
    %   RAIM-only exclusions INSIDE the flagged constellation are consistent
    %   with the constellation attack — mark suspect (inter-const confirms const).
    %
    % Source: ESA Navipedia RAIM Fundamentals (2011) — classic RAIM assumes
    % single fault; multi-fault behaviour is undefined and unreliable.

    const_is_flagged = any(strcmp(inter_flagged_consts, const));

    if ~inter_result.spoofing_suspected
        % ── MODE A: no constellation attack detected ──────────────────────
        if raim_flag
            % RAIM caught a real single-satellite fault
            status  = 'spoofed';
            wfactor = SPOOFED_WEIGHT_FACTOR;
            evidence{end+1} = 'fusion:raim_only(single_fault_mode)';
        end
        % no evidence → trusted (default)

    elseif single_sat_override
        % ── MODE B-single: inter-const flagged one constellation, but
        %    single_sat_override is true — the inter-const outlier is explained
        %    by a single RAIM-isolated fault contaminating that constellation's
        %    WLS solution. Only the RAIM-excluded satellite is spoofed.
        %    All other satellites, including others in the flagged constellation,
        %    are trusted.
        if raim_flag
            % This is the specific satellite RAIM isolated
            status  = 'spoofed';
            wfactor = SPOOFED_WEIGHT_FACTOR;
            evidence{end+1} = 'fusion:raim_only(single_sat_override)';
        end
        % all others → trusted (default)

    else
        % ── MODE B-constellation: genuine constellation-level attack ──────
        if raim_flag && inter_flag
            % Both detectors agree on this satellite's constellation
            status  = 'spoofed';
            wfactor = SPOOFED_WEIGHT_FACTOR;
            evidence{end+1} = 'fusion:both_detectors';

        elseif ~raim_flag && inter_flag
            % Inter-const flagged this constellation; RAIM did not exclude
            % this specific satellite — coherent attack absorbed by WLS
            status  = 'spoofed';
            wfactor = SPOOFED_WEIGHT_FACTOR;
            evidence{end+1} = 'fusion:inter_const_only(chi2_absorbed)';

        elseif raim_flag && ~inter_flag
            % RAIM excluded this satellite but its constellation was NOT
            % flagged by inter-const — collateral RAIM artefact.
            status  = 'suspect';
            wfactor = SUSPECT_WEIGHT_FACTOR;
            evidence{end+1} = 'fusion:raim_only(constellation_attack_mode—demoted_to_suspect)';

        elseif ~raim_flag && const_is_flagged
            % Satellite in the flagged constellation, not specifically excluded
            % by RAIM — part of the coherent spoofed set
            status  = 'suspect';
            wfactor = SUSPECT_WEIGHT_FACTOR;
            evidence{end+1} = 'fusion:inter_const_suspect(in_flagged_const)';
        end
        % satellites in non-flagged constellations with no RAIM flag → trusted
    end

    classifications(i).prn           = prn;
    classifications(i).constellation  = const;
    classifications(i).status         = status;
    classifications(i).evidence       = evidence;
    classifications(i).weight_factor  = wfactor;

    trusted_mask(i) = strcmp(status, 'trusted');
    spoofed_mask(i) = strcmp(status, 'spoofed');
    suspect_mask(i) = strcmp(status, 'suspect');
end

% -------------------------------------------------------------------------
% 4. Determine attack type
%
% Disambiguation rule for single_sat vs constellation:
%   If inter-constellation flags exactly 1 constellation, but RAIM excluded
%   only a small number of satellites from that same constellation, the
%   inter-const outlier is likely caused by the unresolved single fault
%   contaminating the constellation WLS solution — classify as single_sat.
%
%   Threshold: if n_raim_excluded_in_flagged_const <= 1 AND
%              raim_result.n_excluded <= 2, treat as single_sat.
%   Otherwise: constellation-level attack.
% -------------------------------------------------------------------------
n_trusted = sum(trusted_mask);
n_spoofed = sum(spoofed_mask);
n_suspect = sum(suspect_mask);
n_flagged_consts = length(inter_flagged_consts);

if n_spoofed == 0 && n_suspect == 0
    attack_type = 'none';

elseif n_flagged_consts >= 2
    attack_type = 'multi_const';

elseif n_flagged_consts == 1
    % Check whether inter-const outlier is caused by a single RAIM-resolved fault
    flagged_const = inter_flagged_consts{1};
    n_raim_in_flagged = 0;
    for kk = 1:length(raim_result.spoofed_sats)
        if strcmp(raim_result.spoofed_sats{kk}.constellation, flagged_const)
            n_raim_in_flagged = n_raim_in_flagged + 1;
        end
    end
    % single_sat_override was pre-computed before the per-satellite loop
    % (Section 2b) and already applied to per-satellite labels.
    % Reuse it here consistently for attack_type.
    if single_sat_override
        attack_type = 'single_sat';
    else
        attack_type = 'constellation';
    end

else
    % No constellation flagged — single satellite fault
    attack_type = 'single_sat';
end

% -------------------------------------------------------------------------
% 5. Recommended action for Stage 3
% -------------------------------------------------------------------------
switch attack_type
    case 'none'
        recommended_action = 'use_all_satellites';
    case 'single_sat'
        recommended_action = 'exclude_flagged_satellites';
    case 'constellation'
        if n_trusted >= cfg.identify.min_sats
            recommended_action = 'exclude_flagged_constellation';
        else
            recommended_action = 'flag_suspect_degrade_gracefully';
        end
    case 'multi_const'
        if n_trusted >= cfg.identify.min_sats
            recommended_action = 'exclude_all_flagged';
        else
            recommended_action = 'insufficient_trusted_sats_alert';
        end
    otherwise
        recommended_action = 'flag_suspect_degrade_gracefully';
end

% -------------------------------------------------------------------------
% 6. Assemble output
% -------------------------------------------------------------------------
result.classifications    = classifications;
result.sat_list           = rmfield(classifications, {'evidence', 'weight_factor'});
result.trusted_mask       = trusted_mask;
result.spoofed_mask       = spoofed_mask;
result.suspect_mask       = suspect_mask;
result.n_trusted          = n_trusted;
result.n_spoofed          = n_spoofed;
result.n_suspect          = n_suspect;
result.attack_type        = attack_type;
result.recommended_action = recommended_action;

% -------------------------------------------------------------------------
% 7. Verbose output
% -------------------------------------------------------------------------
if cfg.verbose
    fprintf('[CLASSIFY] trusted=%d | spoofed=%d | suspect=%d | attack_type=%s\n', ...
        n_trusted, n_spoofed, n_suspect, attack_type);
    fprintf('[CLASSIFY] recommended_action: %s\n', recommended_action);

    for i = 1:n_obs
        if ~strcmp(classifications(i).status, 'trusted')
            fprintf('[CLASSIFY]   %s PRN %2d → %s | evidence: %s\n', ...
                classifications(i).constellation, ...
                classifications(i).prn, ...
                upper(classifications(i).status), ...
                strjoin(classifications(i).evidence, ', '));
        end
    end
end

end % main function


%% =========================================================================
%  LOCAL HELPER: make_key
%  Creates a unique string key for a constellation+PRN pair.
% =========================================================================
function key = make_key(constellation, prn)
key = sprintf('%s_%03d', constellation, prn);
end
```

## Stage 3 - Exclusion and Gating

### `stage3_exclusion/apply_exclusion_mask.m`

Converts Stage 2 classifications into measurement-weight inflation and reports post-mask geometry sufficiency.

```matlab
function obs_masked = apply_exclusion_mask(obs_epoch, classify_result, cfg)
% APPLY_EXCLUSION_MASK  Stage 3 — Differential weight inflation for spoofed/suspect satellites.
%
% DESIGN BASIS — Robust Kalman Filter / M-estimator literature:
%
%   The technique of inflating a measurement's noise variance (equivalently,
%   reducing its weight) as a function of its anomaly severity originates in
%   the Robust Kalman Filter framework formalised by Yang, He, and Xu (2001),
%   who derive adaptive robust factors that reduce a measurement's weight as a
%   function of its residual severity:
%
%   Yang, Y., He, H., & Xu, G. (2001). "Adaptively robust filtering for
%   kinematic geodetic positioning." Journal of Geodesy, 75(2-3), 109-116.
%   https://doi.org/10.1007/s001900000157
%   (hereafter Yang 2001)
%
%   Yang (2001) identifies two anomaly regimes: moderate residuals (moderate
%   de-weighting) and gross errors (heavy de-weighting).  This module implements
%   a discrete three-tier version of that policy, driven by Stage 2 classification.
%
%   For covariance inflation as an EKF numerical conditioning tool, see:
%   Groves, P.D. (2013). Principles of GNSS, Inertial, and Multisensor
%   Integrated Navigation Systems, 2nd ed. Artech House, Section 14.3.3.
%
% WHAT THIS IS NOT:
%   Blanch et al. (2015/RTCA 040-15) describes a snapshot WLS hard-exclusion
%   architecture for civil aviation ARAIM.  It does NOT describe variance
%   inflation, does NOT have a "suspect" tier, and citing it for this technique
%   would create Protection Level calculation conflicts under civil aviation
%   standards.  It is NOT cited here.
%
% WEIGHT FIELD CONTRACT:
%   This function reads and writes a per-satellite '.weight' field in each
%   constellation sub-struct.  If '.weight' is absent (e.g. obs_epoch came
%   directly from rinex_read_obs which does not add weights), the function
%   initialises weights from cfg.ekf.meas_noise_* as 1/sigma^2.
%   This makes the function callable both from main.m (where main.m may or
%   may not have pre-computed weights) and from unit tests with synthetic structs.
%
% Weight factors applied (Yang 2001, adaptive robust factor framework):
%   spoofed  -> weight / cfg.stage3.spoof_weight_inflation   (default 1e6)
%   suspect  -> weight / cfg.stage3.suspect_weight_inflation  (default 5)
%   trusted  -> weight unchanged
%
% A factor of 1e6 on the variance renders a satellite's contribution to the
% H^T W H normal equations negligible relative to trusted satellites, while
% preserving the ability to monitor it in Stage 4.
%
% Graceful degradation: if n_trusted < cfg.identify.min_sats after masking,
% the function sets obs_masked.insufficient_geometry = true and falls through
% with the degraded set rather than crashing.  This is required by the
% DO-178C robustness objective (thesis objective O7).
%
% INPUTS
%   obs_epoch       struct from rinex_read_obs epoch assembly, with fields:
%                     .GPS/.Galileo/.BeiDou/.GLONASS  -- sub-structs each having:
%                       .prn          [nx1] double
%                       .pseudorange  [nx1] double  (or pseudorange_L1)
%                       .cn0          [nx1] double
%                       .elevation    [nx1] double   (radians)
%                       .weight       [nx1] double   OPTIONAL: 1/sigma^2 per sat.
%                                                    Initialised from cfg if absent.
%   classify_result struct from classify_spoofed_sats, with fields:
%                     .sat_list       [Nx1] struct array, each element:
%                       .constellation  char  ('GPS','Galileo','BeiDou','GLONASS')
%                       .prn            double
%                       .status         char  ('trusted','suspect','spoofed')
%                     .n_trusted      double
%                     .n_suspect      double
%                     .n_spoofed      double
%   cfg             config struct (from config.m)
%
% OUTPUTS
%   obs_masked      copy of obs_epoch with .weight fields updated and new fields:
%                     .insufficient_geometry  logical
%                     .n_trusted_post_mask    double
%                     .n_suspect_post_mask    double
%                     .n_spoofed_post_mask    double
%                     .mask_log               struct -- per-satellite action record
%
% STAGE:    3 -- Measurement Exclusion

    %% --- Parameter defaults ---------------------------------------------------
    if ~isfield(cfg, 'stage3')
        cfg.stage3 = struct();
    end

    if ~isfield(cfg.stage3, 'spoof_weight_inflation')
        % ANALYTICAL BASIS FOR 1e6:
        % In the WLS normal equations, satellite i contributes w_i * h_i * h_i^T
        % to H^T W H.  For its contribution to be negligible, w_spoofed must be
        % small relative to the minimum trusted weight.
        %
        % With GPS sigma^2 = 333 m^2, w_trusted = 1/333 ~ 3e-3.
        % After inflation by 1e6:  w_spoofed = 3e-3 / 1e6 = 3e-9.
        % Ratio: w_trusted / w_spoofed = 1e6  -->  spoofed satellite contributes
        % < 1e-6 of any trusted satellite's weight to the normal equations.
        % This is numerically indistinguishable from hard exclusion for any
        % double-precision solver (machine epsilon ~2.2e-16 << 1e-6).
        %
        % The factor is therefore a *numerical exclusion threshold*, not a
        % physical noise model.  It is justified by solver precision, not by
        % an assumed spoofing error magnitude.
        cfg.stage3.spoof_weight_inflation = 1e6;
    end

    if ~isfield(cfg.stage3, 'suspect_weight_inflation')
        % ENGINEERING SIMPLIFICATION — FACTOR OF 5:
        % Yang et al. (2001, J. Geodesy 75(2-3)) derive a *continuous* robust
        % factor function of the standardised residual v_i / sigma_i.  In that
        % framework the weight reduction is smooth, not discrete.
        %
        % Using a fixed factor of 5 for the "suspect" tier is an engineering
        % simplification: it is the discrete three-tier version of Yang's
        % moderate-residual regime.  The value 5 is NOT derived from the BUCU
        % residual distribution.  It is a conservative starting point that
        % reduces a suspect satellite's influence to ~20% of its nominal value
        % while preserving its geometric contribution.
        %
        % CALIBRATION PENDING: this factor should eventually be set by running
        % apply_exclusion_mask over the 2880-epoch authentic dataset and choosing
        % the factor that minimises position error on the authentic set (no
        % spoofing), to confirm that down-weighting honest suspects does not
        % degrade geometry.  Until that calibration is done, treat 5 as a
        % placeholder, not a validated parameter.
        cfg.stage3.suspect_weight_inflation = 5;
    end

    % Default noise variances used only when .weight is absent from obs_epoch.
    % Values match the calibrated parameters in config.m (HANDOVERSTAGE3.md).
    default_noise = struct( ...
        'GPS',     333.0, ...
        'Galileo', 301.0, ...
        'BeiDou',  4972.0, ...
        'GLONASS', 3476.0);

    %% --- Copy obs_epoch -------------------------------------------------------
    obs_masked = obs_epoch;

    %% --- Build lookup: (constellation, prn) -> status -------------------------
    status_map = containers.Map('KeyType','char','ValueType','char');
    for k = 1:numel(classify_result.sat_list)
        s   = classify_result.sat_list(k);
        key = make_key(s.constellation, s.prn);
        status_map(key) = s.status;
    end

    %% --- Apply weight inflation per constellation ----------------------------
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    mask_log       = struct('constellation',{},'prn',{},'status',{},'weight_before',{},'weight_after',{});
    n_trusted      = 0;
    n_suspect      = 0;
    n_spoofed      = 0;

    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs_masked, cname)
            continue
        end
        sub = obs_masked.(cname);
        if isempty(sub.prn)
            continue
        end
        n_sats = numel(sub.prn);

        % --- Initialise weight field if absent --------------------------------
        % rinex_read_obs does not add a .weight field; main.m computes weights
        % from cfg.ekf.meas_noise_* before calling the pipeline.  If .weight is
        % absent here (e.g. direct call from test or Stage 3 invoked before
        % main.m has added weights), initialise from the default noise map.
        if ~isfield(sub, 'weight')
            sigma2 = default_noise.(cname);
            obs_masked.(cname).weight = repmat(1/sigma2, n_sats, 1);
            sub = obs_masked.(cname);  % re-fetch with weight field present
        end

        for si = 1:n_sats
            prn    = sub.prn(si);
            key    = make_key(cname, prn);
            w_orig = sub.weight(si);

            % Determine trust status.  Default 'trusted' if not in classifier
            % output (satellite absent at this epoch -- treated as honest by
            % omission, consistent with conservative spoofing policy).
            if isKey(status_map, key)
                status = status_map(key);
            else
                status = 'trusted';
            end

            switch status
                case 'spoofed'
                    % Gross-error de-weighting: Yang (2001) adaptive robust factor.
                    w_new = w_orig / cfg.stage3.spoof_weight_inflation;
                    n_spoofed = n_spoofed + 1;

                case 'suspect'
                    % Moderate de-weighting: Yang (2001) "moderate residual" regime.
                    w_new = w_orig / cfg.stage3.suspect_weight_inflation;
                    n_suspect = n_suspect + 1;

                otherwise  % 'trusted'
                    w_new  = w_orig;
                    n_trusted = n_trusted + 1;
            end

            obs_masked.(cname).weight(si) = w_new;

            % Log entry for auditability (DO-178C traceability requirement).
            entry.constellation = cname;
            entry.prn           = prn;
            entry.status        = status;
            entry.weight_before = w_orig;
            entry.weight_after  = w_new;
            mask_log(end+1)     = entry; %#ok<AGROW>
        end
    end

    %% --- Geometry check -------------------------------------------------------
    % Two conditions must both hold for geometry to be declared sufficient:
    %
    % (1) Satellite count: n_trusted >= cfg.identify.min_sats (5).
    %     Five satellites are required for a 4-state (x,y,z,clk) WLS solution
    %     with at least one redundant measurement for chi-squared monitoring.
    %     Source: IS-GPS-200, Section 20.3.3.1; Groves (2013), Section 9.2.
    %
    % (2) Condition number of the geometry matrix H^T H <= cfg.stage3.max_cond_HtH.
    %     Satellite count alone is insufficient — five satellites in nearly the
    %     same elevation band can still produce a poorly conditioned geometry
    %     (e.g. all low-elevation satellites with poor vertical spread).
    %     A condition number threshold of 1e6 is used, above which the WLS
    %     normal equations are considered numerically unstable.
    %     Source: Strang & Borre (1997), Linear Algebra, Geodesy, and GPS,
    %     Wellesley-Cambridge Press, Chapter 9 (condition number and GPS geometry).
    %
    % The H matrix is approximated from elevation angles using a standard
    % unit line-of-sight model with azimuth spread assumed across 360 degrees.
    % This is an approximation; Stage 4 will compute exact H from sat positions.

    if ~isfield(cfg.stage3, 'max_cond_HtH')
        cfg.stage3.max_cond_HtH = 1e6;
    end

    % Collect elevation angles of trusted satellites only.
    el_trusted = [];
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs_masked, cname), continue; end
        sub = obs_masked.(cname);
        if isempty(sub.prn), continue; end
        if ~isfield(sub, 'elevation'), continue; end
        for si = 1:numel(sub.prn)
            key = make_key(cname, sub.prn(si));
            % Determine final status of this satellite post-masking.
            if isKey(status_map, key)
                st = status_map(key);
            else
                st = 'trusted';
            end
            if strcmp(st, 'trusted')
                el_trusted(end+1) = sub.elevation(si); %#ok<AGROW>
            end
        end
    end

    % Condition number check only when enough satellites are present.
    count_ok = (n_trusted >= cfg.identify.min_sats);
    cond_ok  = false;
    cond_HtH = Inf;

    if count_ok && ~isempty(el_trusted)
        % Build approximate H rows: [cos(el)*cos(az), cos(el)*sin(az), sin(el), 1]
        % Distribute azimuths uniformly over 360 deg (worst-case approximation).
        % This gives a conservative geometry estimate.
        n_t  = numel(el_trusted);
        az   = linspace(0, 2*pi*(1 - 1/n_t), n_t)';
        el   = el_trusted(:);
        H_approx = [cos(el).*cos(az), cos(el).*sin(az), sin(el), ones(n_t,1)];
        HtH      = H_approx' * H_approx;
        cond_HtH = cond(HtH);
        cond_ok  = (cond_HtH <= cfg.stage3.max_cond_HtH);
    end

    insufficient_geometry = ~count_ok || ~cond_ok;

    %% --- Annotate output struct -----------------------------------------------
    obs_masked.insufficient_geometry = insufficient_geometry;
    obs_masked.n_trusted_post_mask   = n_trusted;
    obs_masked.n_suspect_post_mask   = n_suspect;
    obs_masked.n_spoofed_post_mask   = n_spoofed;
    obs_masked.cond_HtH              = cond_HtH;
    obs_masked.mask_log              = mask_log;

end

%% ============================================================================
%  LOCAL HELPER
%% ============================================================================

function key = make_key(constellation, prn)
% MAKE_KEY  Builds a string key 'GPS_14' for containers.Map lookup.
    key = sprintf('%s_%d', constellation, prn);
end
```

### `stage3_exclusion/innovation_gate.m`

Applies scalar or joint Mahalanobis innovation tests to reject measurements inconsistent with the predicted state.

```matlab
function gate_result = innovation_gate(innovation, H, P, R, cfg)
% INNOVATION_GATE  Stage 3 — Chi-squared innovation gating for the EKF.
%
% Gates each measurement innovation using the Mahalanobis distance test:
%
%   d^2 = v' * S^{-1} * v
%
% where:
%   v = innovation vector  [mx1]  (measurement - predicted measurement)
%   S = innovation covariance [mxm] = H * P * H' + R
%   m = number of measurements being tested
%
% A measurement is ACCEPTED if d^2 <= chi2_threshold(m, false_alarm_prob).
% A measurement is REJECTED if d^2 > chi2_threshold(m, false_alarm_prob).
%
% This is the standard formulation from:
%   Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). Estimation with
%   Applications to Tracking and Navigation. Wiley, Section 1.4.3.
%   (hereafter Bar-Shalom 2001)
%
% The false alarm probability cfg.identify.false_alarm_prob = 0.001 is the
% Parkinson (1988) standard used throughout this implementation.
%
% SCALAR vs VECTOR gating:
%   The function supports both:
%     (a) SCALAR gate: test each innovation element independently (dof=1).
%         Used when measurements arrive sequentially and should be gated
%         individually before updating the EKF state.
%         Returns gate_result.accepted [mx1] logical mask.
%     (b) JOINT gate:  test the full innovation vector jointly (dof=m).
%         Used as an epoch-level consistency check.
%         Returns gate_result.epoch_accepted logical scalar.
%
%   Mode is selected by cfg.stage3.gate_mode ('scalar' | 'joint').
%
%   SCALAR mode (default) — use during the EKF measurement update loop:
%     Each pseudorange is tested individually before being incorporated.
%     dof = 1, threshold = chi2inv(1 - p_fa, 1) = 10.83 at p_fa=0.001.
%     A single bad measurement is rejected without discarding the epoch.
%     This is the correct mode when the EKF processes measurements one at
%     a time (sequential update), as described in:
%     Groves (2013), Section 3.2.3 — sequential measurement processing.
%
%   JOINT mode — use as an epoch-level consistency check (pre-update):
%     All m measurements are tested together against chi2(m).
%     At m=4, threshold = chi2inv(0.999, 4) = 18.47.
%     If the full epoch fails, all measurements are rejected for this update
%     cycle and the EKF coasts on prediction only.
%     This is the correct mode as a last-resort safety check when apply_
%     exclusion_mask has already masked spoofed satellites but a fully
%     coherent spoofed epoch might still pass per-measurement scalar gates.
%     Reference: Bar-Shalom, Li & Kirubarajan (2001), Section 1.4.3.
%
%   In the Stage 4 EKF pipeline:
%     innovation_gate is used in SCALAR mode on the update path.  JOINT mode
%     remains available for offline epoch-level diagnostics, but is not used
%     inside ekf_runner because repeated per-epoch joint tests can produce
%     false-rejection cascades over long EKF runs.
%
% INPUTS
%   innovation  [mx1] double — innovation vector (z_meas - z_pred)
%   H           [mx4] or [mx8] double — observation matrix.
%               4 columns for Stage 3 WLS-style gating (4-state).
%               8 columns for Stage 4 EKF gating (8-state: pos+vel+clk+drift).
%   P           [4x4] or [8x8] double — prior state covariance.
%               Must match the column count of H.
%   R           [mxm] double — measurement noise covariance (diagonal from cfg)
%   cfg         config struct with fields:
%                 cfg.identify.false_alarm_prob  (default 0.001)
%                 cfg.stage3.gate_mode           ('scalar'|'joint', default 'scalar')
%
% OUTPUTS
%   gate_result struct with fields:
%     .S                  [mxm] innovation covariance
%     .mahal_distances    [mx1] per-element (scalar) or [1x1] (joint) d^2 values
%     .threshold          chi-squared threshold used
%     .dof                degrees of freedom used
%     .accepted           [mx1] logical — per-measurement acceptance (scalar mode)
%     .epoch_accepted     logical      — joint test result (joint mode)
%     .n_accepted         double
%     .n_rejected         double
%     .gate_mode          char
%
% STAGE:    3 — Measurement Exclusion

    %% --- Validate inputs ------------------------------------------------------
    m        = numel(innovation);
    n_states = size(H, 2);   % 4 (Stage 3 standalone) or 8 (Stage 4 runner)

    assert(size(H, 1) == m, ...
        'innovation_gate: H rows (%d) must equal innovation length (%d)', size(H,1), m);
    assert(n_states >= 4, ...
        'innovation_gate: H must have at least 4 columns, got %d', n_states);
    % n_states is 4 (Stage 3 WLS-style), 8 (Stage 4 base EKF), or 8+n_isb
    % (Stage 4 with ISB states). S = H*P*H'+R is valid for any dimension;
    % only H/P column consistency matters, which the next assert checks.
    assert(size(P, 1) == n_states && size(P, 2) == n_states, ...
        'innovation_gate: P must be %dx%d to match H columns, got %dx%d', ...
        n_states, n_states, size(P,1), size(P,2));
    assert(size(R, 1) == m && size(R, 2) == m, ...
        'innovation_gate: R must be %dx%d to match innovation length, got %dx%d', ...
        m, m, size(R,1), size(R,2));

    %% --- Default parameters ---------------------------------------------------
    if ~isfield(cfg, 'stage3')
        cfg.stage3 = struct();
    end
    if ~isfield(cfg.stage3, 'gate_mode')
        cfg.stage3.gate_mode = 'scalar';
    end
    if ~isfield(cfg, 'identify') || ~isfield(cfg.identify, 'false_alarm_prob')
        cfg.identify.false_alarm_prob = 0.001;  % Parkinson 1988
    end

    gate_mode = cfg.stage3.gate_mode;
    p_fa      = cfg.identify.false_alarm_prob;

    %% --- Innovation covariance S = H * P * H' + R ----------------------------
    % Bar-Shalom 2001, eq. 1.4.3-3.
    S = H * P * H' + R;

    % Symmetrise to guard against floating-point asymmetry in P accumulation.
    S = 0.5 * (S + S');

    %% --- Gate ----------------------------------------------------------------
    switch gate_mode

        case 'scalar'
            % Test each innovation element against chi2(dof=1, p_fa).
            % Equivalent to testing |v_i| / sqrt(S_ii) against normal quantile,
            % but phrased as Mahalanobis for generality.
            % Reference: Bar-Shalom 2001, Section 1.4.3, scalar approximation.
            dof       = 1;
            threshold = chi2inv_approx(1 - p_fa, dof);
            S_diag    = max(diag(S), eps);  % avoid divide-by-zero
            mahal     = (innovation .^ 2) ./ S_diag;  % [mx1]
            accepted  = mahal <= threshold;

            gate_result.mahal_distances = mahal;
            gate_result.accepted        = accepted;
            gate_result.epoch_accepted  = all(accepted);
            gate_result.n_accepted      = sum(accepted);
            gate_result.n_rejected      = m - sum(accepted);

        case 'joint'
            % Joint Mahalanobis test: d^2 = v' * S^{-1} * v ~ chi2(m).
            % Reference: Bar-Shalom 2001, Section 1.4.3, eq. 1.4.3-1.
            dof       = m;
            threshold = chi2inv_approx(1 - p_fa, dof);

            % Use backslash for numerical stability over explicit inverse.
            d2 = innovation' * (S \ innovation);  % scalar

            gate_result.mahal_distances = d2;
            epoch_ok                    = d2 <= threshold;
            gate_result.accepted        = repmat(epoch_ok, m, 1);  % all or nothing
            gate_result.epoch_accepted  = epoch_ok;
            gate_result.n_accepted      = m * double(epoch_ok);
            gate_result.n_rejected      = m * double(~epoch_ok);

        otherwise
            error('innovation_gate: unknown gate_mode ''%s''. Use ''scalar'' or ''joint''.', gate_mode);
    end

    %% --- Populate common fields ----------------------------------------------
    gate_result.S          = S;
    gate_result.threshold  = threshold;
    gate_result.dof        = dof;
    gate_result.gate_mode  = gate_mode;

end

%% ============================================================================
%  LOCAL HELPER: Chi-squared inverse (chi2inv_approx)
%% ============================================================================

function x = chi2inv_approx(p, dof)
% CHI2INV_APPROX  Chi-squared inverse CDF.
%
% Uses MATLAB's built-in chi2inv if available (Statistics Toolbox).
% Falls back to Wilson-Hilferty cube-root normal approximation (1931)
% if the toolbox is absent.  The approximation is accurate to <0.1% for
% dof >= 1 at p = 0.999.
%
% Reference for approximation:
%   Wilson, E.B. & Hilferty, M.M. (1931). "The distribution of chi-square."
%   Proceedings of the National Academy of Sciences, 17(12), 684-688.

    if license('test','statistics_toolbox')
        x = chi2inv(p, dof);
    else
        % Wilson-Hilferty approximation
        % z = norminv(p) approximated via rational Beasley-Springer-Moro method
        z = bsm_norminv(p);
        h = 1 - (2 / (9 * dof));
        k = sqrt(2 / (9 * dof));
        x = dof * ((h + k * z) ^ 3);
        x = max(x, 0);  % numerical guard
    end
end

function z = bsm_norminv(p)
% BSM_NORMINV  Rational approximation to the normal inverse CDF.
%
% Beasley-Springer-Moro approximation, accurate to 1e-7 for p in [0.5, 0.9999].
% Reference:
%   Moro, B. (1995). "The Full Monte." Risk, 8(2), 57-58.
%
% Only used if the Statistics Toolbox is absent.

    if p >= 0.5
        sign_flag = 1;
        q = p;
    else
        sign_flag = -1;
        q = 1 - p;
    end

    r = sqrt(-2 * log(1 - q));

    % Rational coefficients
    a = [2.515517, 0.802853, 0.010328];
    b = [1.432788, 0.189269, 0.001308];

    z = r - (a(1) + a(2)*r + a(3)*r^2) / ...
            (1 + b(1)*r + b(2)*r^2 + b(3)*r^3);

    z = sign_flag * z;
end
```

## Stage 4 - Extended Kalman Filter

### `stage4_recovery/build_meas_model_isb.m`

Builds the multi-constellation pseudorange measurement model with GPS-master clock and non-GPS inter-system bias states.

```matlab
function [H, pr_pred, isb_col_per_row] = build_meas_model_isb(sat_positions, constellations, x_pred, cfg)
% BUILD_MEAS_MODEL_ISB  Stage 4 — measurement model with inter-system bias (ISB).
%
% Single source of truth for the EKF observation matrix H and the predicted
% pseudorange pr_pred.  Both ekf_runner.m (full innovation vector) and
% ekf_update.m (accepted-measurement update) call this, so the two cannot
% drift apart.
%
% MEASUREMENT MODEL (Method 1: GPS master clock + N-1 ISB states):
%   For a GPS satellite:
%     z_i = ||sat_i - pos|| + clk_GPS + noise
%     H_i = [ -e_i^T , 0_3^T , 1 , 0 , 0 ... 0 ]
%   For a non-GPS satellite of constellation c:
%     z_i = ||sat_i - pos|| + clk_GPS + ISB_c + noise
%     H_i = [ -e_i^T , 0_3^T , 1 , 0 , ... 1 (in the ISB_c column) ... ]
%
% STATE LAYOUT (n_states = 8 + n_isb):
%   1:3 = position, 4:6 = velocity, 7 = clk_GPS, 8 = clk_drift,
%   9..(8+n_isb) = ISB states, ordered by cfg.ekf.isb_order.
%
% BACKWARD COMPATIBILITY:
%   When numel(x_pred) == 8 (GPS-only configuration), no ISB columns are
%   added and pr_pred = rng + clk_GPS, i.e. exactly the original single-clock
%   model.  cfg.ekf.isb_order is only read when the state is extended, so a
%   minimal cfg (as in test_ekf.m) works unchanged.
%
% INPUTS
%   sat_positions  [m x 3] satellite ECEF positions [m]
%   constellations {m x 1} cell of labels ('GPS'|'Galileo'|'BeiDou'|'GLONASS')
%                  one per row of sat_positions.  May be omitted/empty when
%                  the state is 8 (all rows treated as GPS).
%   x_pred         [n_states x 1] predicted state
%   cfg            config struct (reads cfg.ekf.isb_order only if n_states>8)
%
% OUTPUTS
%   H               [m x n_states] observation matrix
%   pr_pred         [m x 1] predicted pseudorange (incl. clk and ISB)
%   isb_col_per_row [m x 1] ISB state-column index used by each row (0 = none)
%
% STAGE:    4 — EKF Position Recovery (ISB measurement model)

    n_meas   = size(sat_positions, 1);
    n_states = numel(x_pred);
    pos_pred = x_pred(1:3);
    clk_pred = x_pred(7);

    use_isb = (n_states > 8);
    if use_isb
        isb_order = cfg.ekf.isb_order;     % e.g. {'Galileo','BeiDou','GLONASS'}
    else
        isb_order = {};
    end

    if nargin < 2 || isempty(constellations)
        constellations = repmat({'GPS'}, n_meas, 1);
    end

    H               = zeros(n_meas, n_states);
    pr_pred         = zeros(n_meas, 1);
    isb_col_per_row = zeros(n_meas, 1);

    for i = 1:n_meas
        r_vec    = sat_positions(i,:)' - pos_pred;
        rng      = norm(r_vec);
        e_i      = r_vec / rng;

        H(i,1:3) = -e_i';      % position partials
        H(i,7)   = 1;          % GPS master clock column

        isb_val = 0;
        if use_isb
            cn = constellations{i};
            if ~strcmp(cn, 'GPS')
                idx = find(strcmp(isb_order, cn), 1);   % position within isb_order
                if ~isempty(idx)
                    col                = 8 + idx;        % ISB state column
                    H(i, col)          = 1;
                    isb_col_per_row(i) = col;
                    isb_val            = x_pred(col);
                end
            end
        end

        pr_pred(i) = rng + clk_pred + isb_val;
    end
end
```

### `stage4_recovery/ekf_predict.m`

Propagates the position, velocity, receiver-clock, clock-drift, and inter-system-bias states and covariance.

```matlab
function [x_pred, P_pred] = ekf_predict(x, P, cfg)
% EKF_PREDICT  Stage 4 — EKF predict step (time update).
%
% State dimension is derived from numel(x), so the same function serves the
% base 8-state model and the ISB-extended model (8 + n_isb states).
%
% STATE VECTOR (n_states = 8 + n_isb):
%   x(1:3) — receiver position ECEF [m]
%   x(4:6) — receiver velocity ECEF [m/s]
%   x(7)   — receiver GPS clock bias  [m]   (= c x t_bias)
%   x(8)   — receiver clock drift     [m/s] (= c x f_offset / f_nominal)
%   x(9..) — inter-system bias (ISB) states [m], one per non-GPS constellation,
%            ordered by cfg.ekf.isb_order
%
% STATE TRANSITION:
%   pos(k+1)   = pos(k) + dt x vel(k)
%   vel(k+1)   = vel(k)                  (random walk)
%   clk(k+1)   = clk(k) + dt x drift(k)
%   drift(k+1) = drift(k)                (random walk)
%   ISB(k+1)   = ISB(k)                  (random walk — slow hardware/time drift)
%
% Process noise: Q_pos, Q_vel, Q_clk, Q_clk_drift on the base states, and
% Q_isb on each ISB state (only present when n_states > 8).
%
% Justification for constant-velocity model and discrete Q approximation:
%   Groves (2013), Sections 9.4.2 and 3.4.  Joseph-form symmetrisation: 3.2.1.
%
% STAGE:    4 — EKF Position Recovery

    dt       = cfg.ekf.dt;     % 30.0 s
    n_states = numel(x);

    %% --- State transition matrix F [n_states x n_states] --------------------
    F          = eye(n_states);
    F(1:3,4:6) = dt * eye(3);   % pos += dt x vel
    F(7,8)     = dt;            % clk += dt x drift
    % ISB block (9:end) is identity (random walk) — already set by eye().

    %% --- Process noise matrix Q [n_states x n_states] -----------------------
    Q      = zeros(n_states);
    Q(1,1) = cfg.ekf.Q_pos;
    Q(2,2) = cfg.ekf.Q_pos;
    Q(3,3) = cfg.ekf.Q_pos;
    Q(4,4) = cfg.ekf.Q_vel;
    Q(5,5) = cfg.ekf.Q_vel;
    Q(6,6) = cfg.ekf.Q_vel;
    Q(7,7) = cfg.ekf.Q_clk;
    Q(8,8) = cfg.ekf.Q_clk_drift;
    if n_states > 8
        for s = 9:n_states
            Q(s,s) = cfg.ekf.Q_isb;    % ISB random-walk process noise
        end
    end

    %% --- Predict ------------------------------------------------------------
    x_pred = F * x;
    P_pred = F * P * F' + Q;

    % Symmetrise to prevent floating-point asymmetry accumulating over epochs.
    % Source: Groves (2013), Section 3.2.1.
    P_pred = 0.5 * (P_pred + P_pred');

end
```

### `stage4_recovery/ekf_update.m`

Performs the gated EKF measurement update using the shared inter-system-bias measurement model.

```matlab
function [x_upd, P_upd, update_result] = ekf_update(x_pred, P_pred, ...
                                                      pseudoranges, sat_positions, ...
                                                      weights, gate_result, cfg, ...
                                                      constellations)
% EKF_UPDATE  Stage 4 — EKF update step (measurement update).
%
% State dimension is derived from numel(x_pred), so this serves both the base
% 8-state model and the ISB-extended model (8 + n_isb states).  The
% observation matrix and predicted pseudorange are built by the shared helper
% build_meas_model_isb, the single source of truth shared with ekf_runner.
%
% Only measurements accepted by the scalar innovation gate are incorporated.
% Source: Bar-Shalom, Li & Kirubarajan (2001), Sections 5.4 and 1.4.3.
%
% MEASUREMENT MODEL (per build_meas_model_isb):
%   GPS row:      z = ||sat - pos|| + clk_GPS + noise
%   non-GPS row:  z = ||sat - pos|| + clk_GPS + ISB_c + noise
%   H row places -e_i^T in the position columns, 1 in the clock column (7),
%   and 1 in the relevant ISB column for non-GPS rows.
%
% KALMAN UPDATE (Joseph form):
%   S = H P H' + R ;  K = P H' S^-1 ;  x = x + K v
%   P = (I-KH) P (I-KH)' + K R K'        Source: Groves (2013), 3.2.2.
%
% BACKWARD COMPATIBILITY:
%   Called with 7 arguments (no constellations) and an 8-state x_pred, this
%   behaves exactly as the original single-clock 8-state update: all rows are
%   treated as GPS, no ISB columns exist, and pr_pred = rng + clk.  This keeps
%   test_ekf.m (8-state, minimal cfg) byte-identical.
%
% INPUTS
%   x_pred         [n x 1]  predicted state
%   P_pred         [n x n]  predicted covariance
%   pseudoranges   [m x 1]  corrected pseudoranges [m]
%   sat_positions  [m x 3]  satellite ECEF positions [m]
%   weights        [m x 1]  per-satellite weights (1/sigma^2), after masking
%   gate_result    struct from innovation_gate (scalar): .accepted, .n_accepted, .n_rejected
%   cfg            config struct
%   constellations {m x 1} OPTIONAL cell of labels. If omitted/empty, all GPS.
%
% OUTPUTS
%   x_upd, P_upd, update_result   (fields unchanged from the original API)
%
% STAGE:    4 — EKF Position Recovery

    n_states = numel(x_pred);
    n_meas   = numel(pseudoranges);

    if nargin < 8 || isempty(constellations)
        constellations = repmat({'GPS'}, n_meas, 1);
    end

    %% --- Graceful degradation: no accepted measurements --------------------
    if gate_result.n_accepted == 0
        x_upd   = x_pred;
        P_upd   = P_pred;
        update_result.H          = zeros(0, n_states);
        update_result.innovation = zeros(0, 1);
        update_result.S          = zeros(0);
        update_result.K          = zeros(n_states, 0);
        update_result.n_accepted = 0;
        update_result.n_rejected = n_meas;
        update_result.coasted    = true;
        update_result.pos_update = zeros(3,1);
        update_result.vel_update = zeros(3,1);
        update_result.clk_update = 0;
        return
    end

    %% --- Restrict to accepted measurements ---------------------------------
    acc_idx = find(gate_result.accepted);
    m       = numel(acc_idx);

    sp_acc  = sat_positions(acc_idx, :);
    cn_acc  = constellations(acc_idx);
    pr_acc  = pseudoranges(acc_idx);
    w_acc   = weights(acc_idx);

    %% --- Build H, predicted pseudorange, innovation, R ---------------------
    [H, pr_pred] = build_meas_model_isb(sp_acc, cn_acc, x_pred, cfg);
    v = pr_acc(:) - pr_pred;
    R = diag(1 ./ w_acc(:));

    %% --- Kalman gain --------------------------------------------------------
    S = H * P_pred * H' + R;
    S = 0.5 * (S + S');                       % symmetrise before inversion
    K = P_pred * H' * (S \ eye(size(S)));     % [n_states x m]

    %% --- State update -------------------------------------------------------
    x_upd = x_pred + K * v;

    %% --- Covariance update — Joseph form ------------------------------------
    IKH   = eye(n_states) - K * H;
    P_upd = IKH * P_pred * IKH' + K * R * K';
    P_upd = 0.5 * (P_upd + P_upd');

    %% --- Populate result struct ---------------------------------------------
    update_result.H          = H;
    update_result.innovation = v;
    update_result.S          = S;
    update_result.K          = K;
    update_result.n_accepted = m;
    update_result.n_rejected = n_meas - m;
    update_result.coasted    = false;
    update_result.pos_update = x_upd(1:3) - x_pred(1:3);
    update_result.vel_update = x_upd(4:6) - x_pred(4:6);
    update_result.clk_update = x_upd(7)   - x_pred(7);

end
```

### `stage4_recovery/ekf_runner.m`

Runs the multi-epoch EKF recovery chain, including WLS bootstrap, exclusion fallback, innovation gating, and output logging.

```matlab
function ekf_out = ekf_runner(obs, nav, classify_results, cfg)
% EKF_RUNNER  Stage 4 — Full epoch loop: predict → mask → gate → update.
%
% Runs the EKF over all epochs in obs, using Stage 2 classification results
% to mask spoofed measurements before each update. State dimension is
% 8 + n_isb (one inter-system bias state per enabled non-GPS constellation).
%
% EPOCH ALIGNMENT:
%   Epoch 1: WLS bootstrap solution stored directly as output.
%             No predict step — there is no prior state to propagate from.
%   Epoch 2+: standard predict → mask → gate → update cycle.
%
%   This avoids the off-by-one error of predicting from the epoch-1 WLS
%   state into epoch 1 measurements (which would apply 30s of process noise
%   before the first measurement update).
%
% INITIALISATION:
%   Position and clock bias initialised from WLS at epoch 1.
%   cfg.ref_pos is used as the WLS linearisation point (numerical starting
%   point for the iterative solver only — not asserted as the true position).
%   For a fielded receiver without a known survey position, replace with
%   [0;0;0] and allow more WLS iterations to converge.
%   Velocity initialised to zero. Clock drift from cfg.ekf.clk_drift_init
%   (calibrated from 100 authentic BUCU epochs). ISB states initialised to
%   zero with cfg.ekf.P_init_isb uncertainty.
%
% PIPELINE PER EPOCH (epochs 2+):
%   1. ekf_predict          — propagate state and covariance by dt
%   2. Assemble obs_epoch   — collect corrected pseudoranges + sat positions
%   3. apply_exclusion_mask — apply Stage 2 trust weights
%   4. innovation_gate (scalar) — isolate individual bad measurements
%   5. ekf_update           — Kalman update on accepted measurements only
%
% CONSTELLATIONS:
%   All four constellations (GPS, Galileo, BeiDou, GLONASS) used when
%   available. Measurement noise per constellation from cfg.ekf.meas_noise_*.
%   Inter-system bias modelled per non-GPS constellation (cfg.ekf.isb_order).
%
% INPUTS
%   obs               struct from rinex_read_obs
%   nav               struct from rinex_read_nav
%   classify_results  cell array {n_epochs x 1} of classify_spoofed_sats
%                     output structs, one per epoch.  Pass {} or [] to run
%                     in authentic mode (all satellites trusted).
%   cfg               config struct
%
% OUTPUTS
%   ekf_out   struct with fields:
%               .epochs      [Ex1] datetime — epoch timestamps
%               .pos         [Ex3] ECEF position estimates [m]
%               .clk_bias    [Ex1] clock bias estimates [m]
%               .clk_drift   [Ex1] clock drift estimates [m/s]
%               .pos_error   [Ex1] norm distance from cfg.ref_pos [m]
%               .P_trace     [Ex1] trace of position covariance [m²]
%               .hpl         [Ex1] horizontal protection level [m]
%               .n_accepted  [Ex1] measurements accepted per epoch
%               .n_rejected  [Ex1] measurements rejected per epoch
%               .coasted     [Ex1] logical — epoch coasted on prediction
%               .epoch_log   [Ex1] struct array — full per-epoch detail
%
% STAGE:    4 — EKF Position Recovery

    %% --- Setup --------------------------------------------------------------
    epochs       = unique(obs.GPS.time);
    n_epochs     = numel(epochs);
    use_classify = ~isempty(classify_results);

    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    noise_map = struct( ...
        'GPS',     cfg.ekf.meas_noise_GPS, ...
        'Galileo', cfg.ekf.meas_noise_Galileo, ...
        'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
        'GLONASS', cfg.ekf.meas_noise_GLONASS);

    %% --- Pre-allocate outputs -----------------------------------------------
    out_pos       = nan(n_epochs, 3);
    out_clk_bias  = nan(n_epochs, 1);
    out_clk_drift = nan(n_epochs, 1);
    out_pos_err   = nan(n_epochs, 1);
    out_P_trace   = nan(n_epochs, 1);
    out_hpl       = nan(n_epochs, 1);   % horizontal protection level [m]
    out_n_acc     = zeros(n_epochs, 1);
    out_n_rej     = zeros(n_epochs, 1);
    out_coasted   = false(n_epochs, 1);
    out_exclusion_fallback = false(n_epochs, 1);
    epoch_log     = cell(n_epochs, 1);

    %% --- Epoch 1: WLS bootstrap — no predict step --------------------------
    % The EKF has no prior state to propagate from at epoch 1.
    % Store the WLS solution directly as the epoch-1 output.
    % Predict is NOT called — calling predict before epoch-1 measurements
    % would add 30s of process noise to a state that has never been updated,
    % shifting the state forward in time before the first observation.
    %
    % Linearisation point: cfg.ref_pos is used as the numerical starting
    % point for the iterative WLS solver only.  For a receiver without a
    % known survey position, [0;0;0] or a coarse almanac position would be
    % used instead.  This is a known limitation of the current offline
    % validation implementation and is noted in the thesis.
    % Source: Groves (2013), Section 9.4.1.

    [pos_init, clk_init] = bootstrap_wls(obs, nav, epochs(1), constellations, noise_map, cfg);

    if isnan(pos_init(1))
        warning('ekf_runner: WLS bootstrap failed at epoch 1 — falling back to cfg.ref_pos');
        pos_init = cfg.ref_pos(:);
        clk_init = 0.0;
    end

    % State: [x, y, z, vx, vy, vz, clk_GPS, clk_drift, ISB_1..ISB_nisb]
    n_isb    = cfg.ekf.n_isb;
    x = [pos_init(:); ...
         zeros(3,1); ...                  % velocity: zero cold-start
         clk_init; ...                    % GPS clock bias: from WLS epoch 1
         cfg.ekf.clk_drift_init; ...      % clock drift: calibrated from BUCU
         zeros(n_isb,1)];                 % ISB states: zero cold-start

    P = diag([cfg.ekf.P_init_pos  * ones(3,1); ...
              cfg.ekf.P_init_vel  * ones(3,1); ...
              cfg.ekf.P_init_clk; ...
              cfg.ekf.P_init_drift; ...
              cfg.ekf.P_init_isb  * ones(n_isb,1)]);

    % Store epoch 1 output directly from WLS.
    out_pos(1,:)    = x(1:3)';
    out_clk_bias(1) = x(7);
    out_clk_drift(1)= x(8);
    out_pos_err(1)  = norm(x(1:3) - cfg.ref_pos);
    out_P_trace(1)  = trace(P(1:3,1:3));
    out_hpl(1)      = compute_hpl(x(1:3), P(1:3,1:3), cfg.integrity.K_H);
    out_n_acc(1)    = 0;   % WLS bootstrap: no EKF update counted
    epoch_log{1}    = struct('bootstrap', true, 'pos_init', pos_init, 'clk_init', clk_init);

    %% --- Epoch loop: epochs 2 onward ----------------------------------------
    for ei = 2:n_epochs
        t_e = epochs(ei);

        %% Step 1: Predict ---------------------------------------------------
        [x_pred, P_pred] = ekf_predict(x, P, cfg);

        %% Step 2: Assemble obs_epoch ----------------------------------------
        obs_epoch = struct();
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs, cname), continue; end
            mask_t = (obs.(cname).time == t_e);
            if ~any(mask_t), continue; end

            prns_e = obs.(cname).prn(mask_t);
            prs_e  = obs.(cname).pseudorange_L1(mask_t);
            cn0_e  = obs.(cname).cn0(mask_t);

            sub.prn        = [];
            sub.pseudorange = [];
            sub.cn0        = [];
            sub.elevation  = [];
            sub.weight     = [];
            sat_pos_sub    = [];

            for k = 1:numel(prns_e)
                try
                    % Transmit-time corrected measurement. Returns sat_pos at
                    % TRANSMIT time, used below for the EKF geometry so range
                    % and correction are consistent. rec_approx = x_pred(1:3).
                    [pr_corr, sp] = corrected_pseudorange(prs_e(k), prns_e(k), ...
                                        cname, t_e, x_pred(1:3), nav, cfg);
                    if isnan(pr_corr), continue; end

                    % Elevation mask already applied inside corrected_pseudorange.
                    sub.prn(end+1)         = prns_e(k);
                    sub.pseudorange(end+1) = pr_corr;
                    sub.cn0(end+1)         = cn0_e(k);
                    sub.weight(end+1)      = 1 / noise_map.(cname);
                    sat_pos_sub(end+1,:)   = sp';
                catch
                    continue
                end
            end

            % Store column vectors.
            obs_epoch.(cname).prn         = sub.prn(:);
            obs_epoch.(cname).pseudorange = sub.pseudorange(:);
            obs_epoch.(cname).cn0         = sub.cn0(:);
            obs_epoch.(cname).weight      = sub.weight(:);
            obs_epoch.(cname).elevation   = zeros(numel(sub.prn),1); % placeholder
            obs_epoch.(cname).sat_pos     = sat_pos_sub;
        end

        %% Step 3: Apply exclusion mask --------------------------------------
        if use_classify && ei <= numel(classify_results) && ...
                ~isempty(classify_results{ei})
            cr = classify_results{ei};
        else
            % Authentic mode: build trivial all-trusted classify result.
            cr = build_trusted_classify(obs_epoch, constellations);
        end
        obs_masked = apply_exclusion_mask(obs_epoch, cr, cfg);

        %% Step 4: Assemble flat measurement vectors --------------------------
        % all_w      : post-mask weights (Stage 2/3 exclusion applied)
        % all_w_orig : original nominal weights from obs_epoch (pre-mask).
        %              Used by the insufficient-geometry fallback below so
        %              the scalar gate sees nominal R, not a 1e6-inflated R
        %              that would blind it (all-spoofed collapse case).
        all_pr    = [];
        all_sp    = [];
        all_w     = [];
        all_w_orig = [];
        all_const = {};   % constellation label per measurement row (for ISB)
        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs_masked, cname), continue; end
            sub = obs_masked.(cname);
            if isempty(sub.prn), continue; end
            n_sub     = numel(sub.pseudorange(:));
            all_pr    = [all_pr;    sub.pseudorange(:)];          %#ok<AGROW>
            all_w     = [all_w;     sub.weight(:)];               %#ok<AGROW>
            all_sp    = [all_sp;    sub.sat_pos];                 %#ok<AGROW>
            all_const = [all_const; repmat({cname}, n_sub, 1)];   %#ok<AGROW>
            % Original nominal weight for the same row, from obs_epoch.
            all_w_orig = [all_w_orig; obs_epoch.(cname).weight(:)]; %#ok<AGROW>
        end

        %% Step 4b: Insufficient-geometry fallback --------------------------
        % apply_exclusion_mask sets insufficient_geometry=true when fewer
        % than min_sats trusted satellites remain (e.g. the all-spoofed
        % classification collapse under 2-vs-2 inter-constellation
        % ambiguity in dual-constellation attacks). In that state the
        % masked weights are degenerate (every weight deflated by 1e6),
        % which both lets the contaminated set leak into the EKF and
        % blinds the scalar gate (R inflated, Mahalanobis distances shrink).
        %
        % 'all satellites spoofed' here is an INDETERMINATE classifier
        % state (no trusted consensus), not a literal physical claim.
        % Operational response (policy cfg.stage3.insufficient_geometry_policy):
        %   'gate_only' (default): discard the degenerate mask, restore
        %                nominal weights, let the scalar gate protect
        %                position per-measurement against the filter's
        %                own prediction (proven to recover at collapse epochs).
        %   'coast'   : skip the measurement update (stricter integrity mode).
        exclusion_fallback = false;
        fallback_reason    = '';
        ig_policy = 'gate_only';
        if isfield(cfg,'stage3') && isfield(cfg.stage3,'insufficient_geometry_policy')
            ig_policy = cfg.stage3.insufficient_geometry_policy;
        end
        % TRIGGER: use the trusted-satellite COUNT, not obs_masked.
        % insufficient_geometry. The latter also folds in a condition-number
        % check that, inside ekf_runner, is computed from PLACEHOLDER-zero
        % elevations (line ~186) and would therefore fire on every epoch.
        % The count test (n_trusted_post_mask < min_sats) is the clean,
        % elevation-independent signal of the all-spoofed classification
        % collapse we are guarding against.
        n_trusted_pm = numel(all_pr);   % default: all rows present
        if isfield(obs_masked,'n_trusted_post_mask')
            n_trusted_pm = obs_masked.n_trusted_post_mask;
        end
        if n_trusted_pm < cfg.identify.min_sats
            exclusion_fallback = true;
            fallback_reason    = 'insufficient_trusted_geometry_after_classification';
            if strcmp(ig_policy,'gate_only')
                % Restore nominal weights so the gate works on nominal R.
                all_w = all_w_orig;
            end
        end

        n_meas = numel(all_pr);
        if n_meas < cfg.identify.min_sats
            % Insufficient measurements — coast.
            x      = x_pred;
            P      = P_pred;
            out_coasted(ei)  = true;
            out_n_rej(ei)    = n_meas;
            out_pos(ei,:)    = x(1:3)';
            out_clk_bias(ei) = x(7);
            out_clk_drift(ei)= x(8);
            out_pos_err(ei)  = norm(x(1:3) - cfg.ref_pos);
            out_P_trace(ei)  = trace(P(1:3,1:3));
            out_hpl(ei)      = compute_hpl(x(1:3), P(1:3,1:3), cfg.integrity.K_H);
            continue
        end

        %% Step 5: Scalar innovation gate — per-measurement gating -----------
        % DESIGN DECISION: joint gate removed from the EKF update path.
        %
        % A joint chi²(m) gate at p_fa=0.001 over m=36 measurements has
        % probability 1-(1-0.001)^200 = 18% of falsely rejecting at least
        % one authentic epoch over a 200-epoch run.  A single false rejection
        % triggers a coasting cascade: P grows each predict step from Q
        % accumulation, innovations grow, and every subsequent epoch is
        % rejected.  This was confirmed empirically: epoch 106 was rejected
        % on authentic data, causing P to grow from 114 m² to 19,100 m²
        % by epoch 200 with zero measurements accepted.
        %
        % The scalar gate (dof=1 per measurement, chi²(1)=10.83 at p=0.001)
        % is the correct choice for sequential EKF processing.  Each
        % measurement is tested independently; a single bad measurement is
        % rejected without discarding the epoch.
        % Source: Groves (2013), Section 3.2.3.
        %
        % The joint gate remains available in innovation_gate.m for offline
        % diagnostic use (epoch-level consistency checking in post-processing)
        % but must not be on the EKF update critical path.

        % Build measurement model with inter-system bias (ISB) support.
        % Non-GPS rows get their ISB column set in H and their ISB state added
        % to the predicted pseudorange, so the innovation is ISB-consistent.
        % GPS-only runs (n_states==8) reduce to the original single-clock model.
        [H_flat, pr_pred] = build_meas_model_isb(all_sp, all_const, x_pred, cfg);
        v_flat = all_pr(:) - pr_pred;
        R_flat = diag(1 ./ all_w(:));

        % Scalar innovation gate (per-measurement). Can be bypassed for
        % ablation studies via cfg.stage3.use_innovation_gate = false, which
        % accepts ALL measurements (no residual rejection). Default true.
        use_gate = true;
        if isfield(cfg,'stage3') && isfield(cfg.stage3,'use_innovation_gate')
            use_gate = cfg.stage3.use_innovation_gate;
        end
        if exclusion_fallback && strcmp(ig_policy,'coast')
            % COAST policy: indeterminate classification -> skip update.
            % Build an accept-nothing gate; ekf_update will coast.
            m_meas = numel(v_flat);
            gr_scalar = struct( ...
                'S',               H_flat*P_pred*H_flat' + R_flat, ...
                'mahal_distances', zeros(m_meas,1), ...
                'threshold',       0, ...
                'dof',             1, ...
                'accepted',        false(m_meas,1), ...
                'epoch_accepted',  false, ...
                'n_accepted',      0, ...
                'n_rejected',      m_meas, ...
                'gate_mode',       'coast_fallback');
        elseif use_gate
            cfg_scalar = cfg;
            cfg_scalar.stage3.gate_mode = 'scalar';
            gr_scalar = innovation_gate(v_flat, H_flat, P_pred, R_flat, cfg_scalar);
        else
            % ABLATION: gate disabled - accept every measurement.
            m_meas = numel(v_flat);
            gr_scalar = struct( ...
                'S',               H_flat*P_pred*H_flat' + R_flat, ...
                'mahal_distances', zeros(m_meas,1), ...
                'threshold',       Inf, ...
                'dof',             1, ...
                'accepted',        true(m_meas,1), ...
                'epoch_accepted',  true, ...
                'n_accepted',      m_meas, ...
                'n_rejected',      0, ...
                'gate_mode',       'disabled');
        end

        %% Step 6: EKF update ------------------------------------------------
        [x_upd, P_upd, ur] = ekf_update(x_pred, P_pred, ...
                                         all_pr, all_sp, all_w, gr_scalar, cfg, all_const);

        x = x_upd;
        P = P_upd;

        % Record fallback status for auditability (thesis traceability).
        ur.exclusion_fallback = exclusion_fallback;
        ur.fallback_reason    = fallback_reason;
        out_exclusion_fallback(ei) = exclusion_fallback;

        out_pos(ei,:)    = x(1:3)';
        out_clk_bias(ei) = x(7);
        out_clk_drift(ei)= x(8);
        out_pos_err(ei)  = norm(x(1:3) - cfg.ref_pos);
        out_P_trace(ei)  = trace(P(1:3,1:3));
        out_hpl(ei)      = compute_hpl(x(1:3), P(1:3,1:3), cfg.integrity.K_H);
        out_n_acc(ei)    = ur.n_accepted;
        out_n_rej(ei)    = ur.n_rejected;
        out_coasted(ei)  = ur.coasted;
        epoch_log{ei}    = ur;
    end

    %% --- Package output ----------------------------------------------------
    ekf_out.epochs     = epochs;
    ekf_out.pos        = out_pos;
    ekf_out.clk_bias   = out_clk_bias;
    ekf_out.clk_drift  = out_clk_drift;
    ekf_out.pos_error  = out_pos_err;
    ekf_out.P_trace    = out_P_trace;
    ekf_out.hpl        = out_hpl;    % horizontal protection level per epoch [m]
    ekf_out.n_accepted = out_n_acc;
    ekf_out.n_rejected = out_n_rej;
    ekf_out.coasted    = out_coasted;
    ekf_out.exclusion_fallback = out_exclusion_fallback;
    ekf_out.epoch_log  = epoch_log;

end

%% ============================================================================
%  LOCAL HELPERS
%% ============================================================================

function cr = build_trusted_classify(obs_epoch, constellations)
% BUILD_TRUSTED_CLASSIFY  Returns an all-trusted classify result for authentic mode.
    sat_list = struct('constellation',{},'prn',{},'status',{});
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        if ~isfield(obs_epoch, cname), continue; end
        for k = 1:numel(obs_epoch.(cname).prn)
            sat_list(end+1).constellation = cname;  %#ok<AGROW>
            sat_list(end).prn    = obs_epoch.(cname).prn(k);
            sat_list(end).status = 'trusted';
        end
    end
    cr.sat_list  = sat_list;
    cr.n_trusted = numel(sat_list);
    cr.n_suspect = 0;
    cr.n_spoofed = 0;
end

function [pos_wls, clk_wls] = bootstrap_wls(obs, nav, t_e, constellations, noise_map, cfg)
% BOOTSTRAP_WLS  Compute a single-epoch WLS solution for EKF initialisation.
%
% Uses all available measurements at epoch t_e with all-trusted weights.
% Single-clock WLS (no ISB): this only provides the epoch-1 starting
% position. The ISB states start at zero and converge in the main epoch
% loop. Returns [NaN; NaN; NaN] and 0 if fewer than cfg.identify.min_sats
% measurements are available.
%
% This ensures ekf_runner initialises from an observable position estimate
% rather than the known reference position — a requirement for any receiver
% that does not have prior knowledge of its location.
%
% Source: Groves (2013), Section 9.4.1.

    all_pr = [];
    all_sp = [];
    all_w  = [];

    % Linearisation point for WLS solver.
    % cfg.ref_pos is used here as the iterative WLS starting point because
    % this is an offline validation implementation with a known survey position.
    % A fielded receiver would use a coarse approximate position from one of:
    %   - a previous fix stored in non-volatile memory
    %   - an almanac-based position estimate
    %   - a survey-provided approximate coordinate
    % Using Earth-centre [0;0;0] is NOT appropriate here because
    % pseudorange_correct requires a physically meaningful receiver position
    % for elevation mask and tropospheric correction computation.
    % This limitation is noted in the thesis (Section 4.6, implementation scope).
    pos_lin = cfg.ref_pos(:);

    for ci = 1:numel(constellations)
        cname  = constellations{ci};
        if ~isfield(obs, cname), continue; end
        mask_t = (obs.(cname).time == t_e);
        if ~any(mask_t), continue; end

        prns_e = obs.(cname).prn(mask_t);
        prs_e  = obs.(cname).pseudorange_L1(mask_t);

        for k = 1:numel(prns_e)
            try
                % Transmit-time corrected measurement; rec_approx = pos_lin
                % (cfg.ref_pos cold-start linearization point).
                [pr_corr, sp] = corrected_pseudorange(prs_e(k), prns_e(k), ...
                                    cname, t_e, pos_lin, nav, cfg);
                if isnan(pr_corr), continue; end
                all_pr(end+1)   = pr_corr;    %#ok<AGROW>
                all_sp(end+1,:) = sp';         %#ok<AGROW>
                all_w(end+1)    = 1 / noise_map.(cname);  %#ok<AGROW>
            catch
                continue
            end
        end
    end

    if numel(all_pr) < cfg.identify.min_sats
        pos_wls = [NaN; NaN; NaN];
        clk_wls = 0;
        return
    end

    [pos_wls, clk_wls] = wls_solver(all_pr(:), all_sp, all_w(:), pos_lin);
end
```

## Horizontal Protection Level

### `stage4_recovery/compute_hpl.m`

Transforms EKF position covariance into the local horizontal frame and computes the experimental horizontal protection level.

```matlab
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
```

## Key Utilities

### `utils/wls_solver.m`

Solves the weighted pseudorange navigation equations and returns position, receiver clock bias, and post-fit residuals.

```matlab
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
```

### `utils/ecef2lla_simple.m`

Converts ECEF coordinates to geodetic latitude, longitude, and altitude for local-frame and atmospheric calculations.

```matlab
function [lat, lon, alt] = ecef2lla_simple(pos)
% ECEF2LLA_SIMPLE  Convert ECEF position to WGS84 geodetic latitude/longitude/altitude.
%
% Converts a position from Earth-Centred Earth-Fixed (ECEF) coordinates
% (X, Y, Z in metres) to WGS84 geodetic coordinates using the iterative
% Bowring method. Returns geodetic latitude and longitude in RADIANS and
% altitude above the WGS84 ellipsoid in metres.
%
% This is an authoritative coordinate-conversion utility used across the
% pipeline (e.g. by compute_hpl.m to build the local ENU frame). Geodetic
% (not geocentric) latitude is required so that the local vertical (Up)
% is normal to the WGS84 ellipsoid, as assumed by the ENU transformation.
%
% INPUT
%   pos   [3x1] or [1x3] ECEF position [x; y; z] in metres
%
% OUTPUT
%   lat   geodetic latitude  [rad]
%   lon   geodetic longitude [rad]
%   alt   altitude above WGS84 ellipsoid [m]
%
% REFERENCE
%   WGS84 datum; Bowring (1976) iterative geodetic latitude solution.
%

    % --- WGS84 ellipsoid constants ---
    a  = 6378137.0;             % semi-major axis [m]
    f  = 1/298.257223563;       % flattening
    e2 = 2*f - f^2;             % first eccentricity squared

    x = pos(1); y = pos(2); z = pos(3);

    lon = atan2(y, x);
    p   = sqrt(x^2 + y^2);      % distance from the Z (rotation) axis

    % Initial geodetic latitude estimate.
    lat = atan2(z, p * (1 - e2));

    % Iterative refinement (Bowring). Update lat at the END of each
    % iteration so the variable always holds the latest estimate, then
    % break once the change falls below tolerance. This avoids relying on
    % a post-loop fix-up and is well defined even if the loop runs once.
    for i = 1:10
        N       = a / sqrt(1 - e2 * sin(lat)^2);   % prime vertical radius
        lat_new = atan2(z + e2 * N * sin(lat), p);
        converged = abs(lat_new - lat) < 1e-12;
        lat = lat_new;
        if converged, break; end
    end

    % Altitude above the ellipsoid.
    N   = a / sqrt(1 - e2 * sin(lat)^2);
    alt = p / cos(lat) - N;
end
```

### `utils/rinex_read_obs.m`

Reads multi-constellation RINEX observation data into the pipeline observation structure.

```matlab
% to run fcn; obs = rinex_read_obs('data/rinex/observation/authentic.obs', cfg);
function obs = rinex_read_obs(obs_file, cfg)
% rinex_read_obs  Load and parse a RINEX observation file.
%
%   obs = rinex_read_obs(obs_file, cfg)
%
%   INPUT:
%     obs_file  - full path to the .obs RINEX file (string)
%     cfg       - configuration struct from config.m
%
%   OUTPUT:
%     obs       - struct with fields:
%                   .raw        raw output from rinexread()
%                   .GPS        GPS measurements struct
%                   .Galileo    Galileo measurements struct
%                   .BeiDou     BeiDou measurements struct
%                   .GLONASS    GLONASS measurements struct
%                   .epochs     datetime array of observation times
%
%   Each constellation struct contains:
%                   .pseudorange   [n_epochs x n_sats] metres
%                   .carrier_phase [n_epochs x n_sats] cycles
%                   .doppler       [n_epochs x n_sats] Hz
%                   .cn0           [n_epochs x n_sats] dB-Hz
%                   .prn           [1 x n_sats] satellite numbers
%                   .n_epochs      scalar
%                   .n_sats        scalar
%
%   Field names verified against BUCU00ROU RINEX file.

if cfg.verbose
    fprintf('      Reading observation file: %s\n', obs_file);
end

%% ── VALIDATE FILE ────────────────────────────────────────────────────────
if ~isfile(obs_file)
    error('rinex_read_obs: file not found:\n  %s', obs_file);
end

%% ── READ RAW RINEX ───────────────────────────────────────────────────────
raw    = rinexread(obs_file);
obs.raw = raw;

%% ── FIELD MAP (verified against your BUCU00ROU file) ─────────────────────
% Each row: {pseudorange_L1, pseudorange_L2, carrier_L1, carrier_L2, doppler, cn0}
field_map.GPS     = {'C1C', 'C2W', 'L1C', 'L2W', 'D1C', 'S1C'};
field_map.Galileo = {'C1C', 'C7Q', 'L1C', 'L7Q', 'D1C', 'S1C'};
field_map.BeiDou  = {'C1P', 'C2I', 'L1P', 'L2I', 'D1P', 'S1P'};
field_map.GLONASS = {'C1C', 'C2P', 'L1C', 'L2P', 'D1C', 'S1C'};

%% ── EXTRACT EACH CONSTELLATION ───────────────────────────────────────────
constellations = {'GPS', 'Galileo', 'BeiDou', 'GLONASS'};

for k = 1:length(constellations)
    name = constellations{k};

    if cfg.const.(name) && isfield(raw, name)
        obs.(name) = extract_constellation(raw.(name), name, field_map.(name));
        if cfg.verbose
            fprintf('      %-8s %d satellites, %d epochs\n', ...
                [name ':'], obs.(name).n_sats, obs.(name).n_epochs);
        end
    else
        obs.(name) = empty_constellation(name);
        if cfg.verbose
            fprintf('      %-8s not present or disabled\n', [name ':']);
        end
    end
end

%% ── EXTRACT EPOCH TIMES ──────────────────────────────────────────────────
for k = 1:length(constellations)
    name = constellations{k};
    if isfield(raw, name) && height(raw.(name)) > 0
        % Store UNIQUE epoch times only
        obs.epochs   = unique(raw.(name).Time);
        obs.n_epochs = length(obs.epochs);
        break;
    end
end

if cfg.verbose
    fprintf('      Total epochs: %d\n', obs.n_epochs);
    fprintf('      Time start:   %s\n', datestr(obs.epochs(1)));
    fprintf('      Time end:     %s\n', datestr(obs.epochs(end)));
end

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: extract_constellation
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function c = extract_constellation(tt, name, fields)
% Extract measurements from a rinexread timetable into a clean struct.
%
%   fields = {pr_L1, pr_L2, cp_L1, cp_L2, doppler, cn0}

    c.name = name;

    % Number of epochs
    c.n_epochs = height(tt);
    c.time = tt.Time;   % full time column — needed for epoch+PRN indexing

    % Extract satellite PRN numbers from SatelliteID column
    c.prn    = tt.SatelliteID;
    c.n_sats = length(unique(c.prn));

    % ── Pseudorange L1 ────────────────────────────────────────────────────
    c.pseudorange_L1  = safe_extract(tt, fields{1});

    % ── Pseudorange L2 ────────────────────────────────────────────────────
    c.pseudorange_L2  = safe_extract(tt, fields{2});

    % ── Carrier phase L1 ──────────────────────────────────────────────────
    c.carrier_phase_L1 = safe_extract(tt, fields{3});

    % ── Carrier phase L2 ──────────────────────────────────────────────────
    c.carrier_phase_L2 = safe_extract(tt, fields{4});

    % ── Doppler L1 ────────────────────────────────────────────────────────
    c.doppler         = safe_extract(tt, fields{5});

    % ── C/N0 L1 ───────────────────────────────────────────────────────────
    c.cn0             = safe_extract(tt, fields{6});

    % ── Elevation (not always present) ────────────────────────────────────
    if ismember('Elevation', tt.Properties.VariableNames)
        c.elevation = tt.Elevation;
    else
        c.elevation = nan(c.n_epochs, 1);
    end

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: safe_extract
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function data = safe_extract(tt, field_name)
% Extract a column from a timetable by field name.
% Returns NaN column if field does not exist.

    if ismember(field_name, tt.Properties.VariableNames)
        data = tt.(field_name);
    else
        warning('rinex_read_obs: field "%s" not found — filling with NaN', ...
            field_name);
        data = nan(height(tt), 1);
    end

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: empty_constellation
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function c = empty_constellation(name)
% Return empty struct when constellation is disabled or not in file.

    c.name            = name;
    c.pseudorange_L1  = [];
    c.pseudorange_L2  = [];
    c.carrier_phase_L1= [];
    c.carrier_phase_L2= [];
    c.doppler         = [];
    c.cn0             = [];
    c.elevation       = [];
    c.prn             = [];
    c.n_epochs        = 0;
    c.n_sats          = 0;

end
```

### `utils/rinex_read_nav.m`

Reads multi-constellation RINEX navigation records into the broadcast-ephemeris structure.

```matlab
%to test fucn run nav = rinex_read_nav('data/rinex/navigation/authentic.nav', cfg);
function nav = rinex_read_nav(nav_file, cfg)
% rinex_read_nav  Load and parse a RINEX navigation file.
%
%   nav = rinex_read_nav(nav_file, cfg)
%
%   INPUT:
%     nav_file  - full path to the .nav RINEX file (string)
%     cfg       - configuration struct from config.m
%
%   OUTPUT:
%     nav       - struct with fields:
%                   .raw        raw output from rinexread()
%                   .GPS        GPS ephemeris struct
%                   .Galileo    Galileo ephemeris struct
%                   .BeiDou     BeiDou ephemeris struct
%                   .GLONASS    GLONASS ephemeris struct
%
%   Each constellation struct contains:
%                   .prn        satellite PRN numbers
%                   .toe        time of ephemeris (datetime)
%                   .data       full ephemeris timetable

if cfg.verbose
    fprintf('      Reading navigation file: %s\n', nav_file);
end

%% ── VALIDATE FILE ────────────────────────────────────────────────────────
if ~isfile(nav_file)
    error('rinex_read_nav: file not found:\n  %s', nav_file);
end

%% ── READ RAW RINEX ───────────────────────────────────────────────────────
raw     = rinexread(nav_file);
nav.raw = raw;

%% ── EXTRACT EACH CONSTELLATION ───────────────────────────────────────────
constellations = {'GPS', 'Galileo', 'BeiDou', 'GLONASS'};

for k = 1:length(constellations)
    name = constellations{k};

    if cfg.const.(name) && isfield(raw, name)
        nav.(name) = extract_nav(raw.(name), name);
        if cfg.verbose
            fprintf('      %-8s %d ephemeris records\n', ...
                [name ':'], height(raw.(name)));
        end
    else
        nav.(name) = empty_nav(name);
        if cfg.verbose
            fprintf('      %-8s not present or disabled\n', [name ':']);
        end
    end
end

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: extract_nav
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function n = extract_nav(tt, name)
% Extract ephemeris data from a rinexread navigation timetable.

    n.name = name;
    n.data = tt;

    % Satellite PRN numbers
    if ismember('SatelliteID', tt.Properties.VariableNames)
        n.prn = tt.SatelliteID;
    else
        n.prn = [];
    end

    % Time of ephemeris
    n.toe = tt.Time;

    % Total records
    n.n_records = height(tt);

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: empty_nav
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function n = empty_nav(name)
% Return empty struct when constellation is disabled or not in file.

    n.name      = name;
    n.data      = [];
    n.prn       = [];
    n.toe       = [];
    n.n_records = 0;

end
```

### `utils/inject_spoofing.m`

Generates the evaluated gradual position drag-off by applying exact transmit-time geometric range differences to configured signals.

```matlab
function obs_spoofed = inject_spoofing(obs, nav, cfg)
% inject_spoofing  Mathematically inject a POSITION spoof into authentic obs.
%
%   Spoofing model: gradual drag-off attack (Humphreys et al. 2008),
%   implemented as a coherent POSITION spoof consistent with thesis Eq. 4.14.
%
%   Each spoofed satellite receives its own pseudorange bias equal to the
%   exact geometric range difference between the intended FALSE receiver
%   position and the TRUE receiver position:
%
%       bias_i = || sat_i - rec_false || - || sat_i - rec_true ||
%
%   where rec_true = cfg.ref_pos and rec_false = rec_true + dp(e), with dp(e)
%   ramping from zero to the full cfg.spoof.target_offset VECTOR along its own
%   direction at drift_rate (capped at the target magnitude).
%
%   WHY NOT A UNIFORM SCALAR DRAG:
%     Adding the same scalar to every satellite is a clock/time shift, which
%     the WLS/EKF absorbs almost entirely into the receiver clock state,
%     producing ~0 m of position displacement. A per-satellite, geometry-
%     dependent bias is what actually drags the position solution. This was
%     verified numerically (uniform 591 m -> clock, position ~0 m; per-sat
%     geometric bias -> position converges to rec_false).
%
%   SIGN: the exact two-norm difference is used (not the linearised -e.dp),
%   so there is no line-of-sight sign convention to get wrong. The verification
%   test test_injection_geometry.m confirms the solved displacement equals
%   +target_offset in both direction and magnitude.
%
%   INPUT:
%     obs - authentic observation struct from rinex_read_obs()
%     nav - navigation struct from rinex_read_nav()  (needed for sat positions)
%     cfg - configuration struct from config.m
%
%   OUTPUT:
%     obs_spoofed - modified observation struct,
%                   saved to results/simulated_scenarios/<scenario_name>/
%

%% ── COPY AUTHENTIC OBS ───────────────────────────────────────────────────
obs_spoofed = obs;

%% ── PARAMETERS ───────────────────────────────────────────────────────────
start_epoch            = cfg.spoof.start_epoch;
drift_rate             = cfg.spoof.drift_rate;
spoofed_constellations = cfg.spoof.spoofed_constellations;
cn0_boost              = cfg.spoof.cn0_boost;

% Intended displacement as a VECTOR (ECEF), with its magnitude and direction.
target_vec = cfg.spoof.target_offset(:);     % [3x1] m
target_mag = norm(target_vec);               % scalar drag cap
if target_mag > 0
    target_unit = target_vec / target_mag;
else
    target_unit = [0;0;0];
end

rec_true = cfg.ref_pos(:);                   % [3x1] true receiver position

% L1 wavelengths per constellation (metres) — for carrier-phase cycle bias.
lambda_map.GPS     = 0.190293672;  % c / 1575.42 MHz
lambda_map.Galileo = 0.190293672;  % E1 same frequency as GPS L1
lambda_map.BeiDou  = 0.192039486;  % c / 1561.098 MHz
lambda_map.GLONASS = 0.187523460;  % c / 1598.0625 MHz centre

if cfg.verbose
    fprintf('      Scenario: %s\n', cfg.spoof.scenario_name);
    fprintf('      Spoofed constellations: ');
    fprintf('%s ', spoofed_constellations{:});
    fprintf('\n');
    fprintf('      Attack start: epoch %d\n', start_epoch);
    fprintf('      Drift rate:   %.1f m/epoch\n', drift_rate);
    fprintf('      Target drag:  %.1f m  (vector [%.0f %.0f %.0f])\n', ...
            target_mag, target_vec(1), target_vec(2), target_vec(3));
    fields = fieldnames(cfg.spoof.spoofed_PRNs);
    for fi = 1:length(fields)
        fname = fields{fi};
        fprintf('      %s PRNs: ', fname);
        fprintf('%d ', cfg.spoof.spoofed_PRNs.(fname));
        fprintf('\n');
    end
end

%% ── GET UNIQUE EPOCHS ────────────────────────────────────────────────────
epochs   = obs.epochs;
n_epochs = length(epochs);

%% ── INJECT PER CONSTELLATION ─────────────────────────────────────────────
total_injected = 0;

for c = 1:length(spoofed_constellations)
    const_name = spoofed_constellations{c};

    if ~isfield(cfg.spoof.spoofed_PRNs, const_name)
        warning('inject_spoofing: no PRNs defined for %s — skipping', const_name);
        continue;
    end

    spoofed_PRNs = cfg.spoof.spoofed_PRNs.(const_name);
    lambda       = lambda_map.(const_name);
    n_injected   = 0;

    for e = 1:n_epochs
        if e < start_epoch, continue; end

        t_since = e - start_epoch;
        drag_m  = min(t_since * drift_rate, target_mag);   % magnitude along target_unit
        dp      = drag_m * target_unit;                     % [3x1] current displacement
        rec_false = rec_true + dp;                          % [3x1] intended false position
        t_e     = epochs(e);

        for k = 1:length(spoofed_PRNs)
            prn_k = spoofed_PRNs(k);

            mask = (obs.(const_name).time == t_e) & ...
                   (obs.(const_name).prn  == prn_k);
            idx  = find(mask, 1, 'first');

            if isempty(idx), continue; end
            if isnan(obs.(const_name).pseudorange_L1(idx)), continue; end

            % Satellite ECEF position at SIGNAL TRANSMIT TIME - same
            % convention used by the receiver measurement model (Stage 2/4
            % via corrected_pseudorange). Iterate travel time against the TRUE
            % receiver position, seeded by the raw L1 pseudorange. Using one
            % transmit-time position for both range terms below is exact to
            % about 8 mm for the configured spoof offset, far below the
            % project's measurement/noise scale.
            C_LIGHT = 299792458.0;
            pr_raw  = obs.(const_name).pseudorange_L1(idx);
            tau     = pr_raw / C_LIGHT;
            sp      = [NaN; NaN; NaN];
            try
                for it = 1:3
                    t_tx = t_e - seconds(tau);
                    [sp, ~] = sat_position(nav, prn_k, const_name, t_tx);
                    if any(isnan(sp)), break; end
                    tau = norm(sp(:) - rec_true) / C_LIGHT;
                end
            catch
                continue
            end
            if isempty(sp) || any(isnan(sp)), continue; end
            sp = sp(:);

            % Exact per-satellite geometric range-difference bias (metres),
            % using the transmit-time satellite position for both terms.
            bias_i = norm(sp - rec_false) - norm(sp - rec_true);

            % Corrupt pseudorange L1
            obs_spoofed.(const_name).pseudorange_L1(idx) = ...
                obs.(const_name).pseudorange_L1(idx) + bias_i;

            % Corrupt pseudorange L2 (range bias is frequency-independent)
            if ~isempty(obs.(const_name).pseudorange_L2) && ...
               idx <= length(obs.(const_name).pseudorange_L2) && ...
               ~isnan(obs.(const_name).pseudorange_L2(idx))
                obs_spoofed.(const_name).pseudorange_L2(idx) = ...
                    obs.(const_name).pseudorange_L2(idx) + bias_i;
            end

            % Corrupt carrier phase L1 (bias in cycles = bias_i / lambda)
            if ~isempty(obs.(const_name).carrier_phase_L1) && ...
               idx <= length(obs.(const_name).carrier_phase_L1) && ...
               ~isnan(obs.(const_name).carrier_phase_L1(idx))
                obs_spoofed.(const_name).carrier_phase_L1(idx) = ...
                    obs.(const_name).carrier_phase_L1(idx) + bias_i / lambda;
            end

            % Boost C/N0 (spoofer overpowers authentic signal)
            if ~isempty(obs.(const_name).cn0) && ...
               idx <= length(obs.(const_name).cn0) && ...
               ~isnan(obs.(const_name).cn0(idx))
                obs_spoofed.(const_name).cn0(idx) = ...
                    min(obs.(const_name).cn0(idx) + cn0_boost, 60.0);
            end

            n_injected = n_injected + 1;
        end
    end

    total_injected = total_injected + n_injected;

    if cfg.verbose
        fprintf('      %s: injected %d measurements\n', const_name, n_injected);
    end
end

if cfg.verbose
    fprintf('      Total injected: %d measurements\n', total_injected);
end

%% ── SAVE SCENARIO ────────────────────────────────────────────────────────
scenario_dir = fullfile(cfg.paths.scenarios, cfg.spoof.scenario_name);
if ~exist(scenario_dir, 'dir'), mkdir(scenario_dir); end

save(fullfile(scenario_dir, 'spoofed_obs.mat'), 'obs_spoofed');

params.scenario_name          = cfg.spoof.scenario_name;
params.spoofed_constellations = spoofed_constellations;
params.spoofed_PRNs           = cfg.spoof.spoofed_PRNs;
params.start_epoch            = start_epoch;
params.drift_rate             = drift_rate;
params.target_offset          = cfg.spoof.target_offset;   % stored as vector
params.cn0_boost              = cn0_boost;
params.injection_model        = 'geometric_position_spoof_exact_range_diff';
params.created                = datetime('now', 'Format', 'dd-MMM-yyyy HH:mm:ss');

save(fullfile(scenario_dir, 'params.mat'), 'params');

if cfg.verbose
    fprintf('      Saved to: %s\n', scenario_dir);
end

end

%% to test on cmd (NOTE: nav is now required)
% clear functions
% config
% obs = rinex_read_obs(fullfile(cfg.paths.obs, 'authentic.obs'), cfg);
% nav = rinex_read_nav(fullfile(cfg.paths.nav, 'authentic.nav'), cfg);
%
% fprintf('Generating all 5 fixed scenarios...\n\n');
% for s = 1:5
%     cfg.spoof.active_scenario        = s;
%     cfg.spoof.scenario_name          = cfg.scenarios{s}.name;
%     cfg.spoof.spoofed_constellations = cfg.scenarios{s}.spoofed_constellations;
%     cfg.spoof.spoofed_PRNs           = cfg.scenarios{s}.spoofed_PRNs;
%     cfg.spoof.start_epoch            = cfg.scenarios{s}.start_epoch;
%     cfg.spoof.drift_rate             = cfg.scenarios{s}.drift_rate;
%     cfg.spoof.target_offset          = cfg.scenarios{s}.target_offset;
%     cfg.spoof.cn0_boost              = cfg.scenarios{s}.cn0_boost;
%
%     fprintf('--- Scenario %d ---\n', s);
%     inject_spoofing(obs, nav, cfg);
%     fprintf('\n');
% end
% fprintf('All 5 scenarios saved.\n');
```

## Available on Request - Plotting, Figure, Audit, and Diagnostic Files

The following files were intentionally excluded from the printed annex because they generate figures, inspect saved evidence, or perform one-off audit and diagnostic tasks:

- `audit_fallback_epochs.m`
- `check_sessionB.m`
- `compute_horizontal_error.m`
- `confirm_hardware_scatter.m`
- `confirm_hpl_compliance.m`
- `diagnose_epoch.m`
- `generate_all_figures.m`
- `horizontal_error.m`
- `inspect_mat_fields.m`
- `integrity_table.m`
- `plot_ablation_zoom.m`
- `plot_chi2_timeline.m`
- `plot_classification_counts.m`
- `plot_exclusion_heatmap.m`
- `plot_hardware_wls_scatter.m`
- `plot_measurement_counts.m`
- `plot_stanford.m`
- `verify_provenance.m`

## All Other MATLAB Files Not Included

These MATLAB files exist in the project but are outside the selected core-contribution scope of this annex:

- `main.m`
- `run_ablation.m`
- `run_all_scenarios.m`
- `run_baseline_authentic.m`
- `run_pipeline.m`
- `run_real_authentic_validation.m`
- `setup_project.m`
- `stage0_osnma/mac_verify.m`
- `stage0_osnma/osnma_status.m`
- `stage0_osnma/osnma_verify.m`
- `stage0_osnma/parse_inav_page.m`
- `stage0_osnma/parse_sfrbx_gate.m`
- `stage0_osnma/tesla_key_chain.m`
- `stage0_osnma/test_osnma.m`
- `stage0_osnma/test_osnma_crypto.m`
- `stage1_detection/combine_detectors.m`
- `stage1_detection/detect_agc.m`
- `stage1_detection/detect_clock.m`
- `stage1_detection/detect_cn0.m`
- `stage1_detection/detect_pseudorange.m`
- `stage2_identification/calibrate_inter_const_threshold.m`
- `stage2_identification/test_classify_spoofed_sats.m`
- `stage2_identification/test_inter_constellation.m`
- `stage2_identification/test_raim_fde.m`
- `stage3_exclusion/test_apply_exclusion_mask.m`
- `stage3_exclusion/test_innovation_gate.m`
- `stage3_exclusion/test_stage3_integration.m`
- `stage4_recovery/calibrate_clock_drift.m`
- `stage4_recovery/calibration/test_sat_bias_calibration.m`
- `stage4_recovery/ionofree/test_ionofree.m`
- `stage4_recovery/test_ekf.m`
- `stage4_recovery/test_hpl.m`
- `stage4_recovery/test_isb_reproduction.m`
- `stage4_recovery/test_isb_resolved.m`
- `stage4_recovery/test_stage4_integration.m`
- `test_1hz_ekf_feasibility.m`
- `test_insufficient_geometry_fallback.m`
- `utils/calibration/apply_sat_bias.m`
- `utils/calibration/calibrate_sat_bias.m`
- `utils/coord_convert.m`
- `utils/ionofree_combination.m`
- `utils/test_corrected_pseudorange.m`
- `utils/test_group_delay_bgd.m`
- `utils/test_injection_geometry.m`
- `utils/test_sagnac_fix.m`
- `validate_1hz_profile.m`

