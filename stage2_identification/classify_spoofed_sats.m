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
