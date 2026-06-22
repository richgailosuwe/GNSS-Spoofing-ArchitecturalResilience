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
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

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
