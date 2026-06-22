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
