function diagnose_epoch(scenario_id, target_epoch)
% DIAGNOSE_EPOCH  Dissect Stage 2 classification at one epoch of one scenario.
%
%   diagnose_epoch(scenario_id, target_epoch)
%
% Re-derives the spoofed observations and Stage 2 classification for the given
% scenario, then reports — at target_epoch — exactly which satellites were
% marked spoofed/suspect/trusted, how many remained, which constellations
% survived, and the attack type. Used to root-cause the A>B anomaly where the
% full pipeline (exclusion ON) does worse than gate-only.
%

    run('config.m');
    cfg.verbose = false;

    s = cfg.scenarios{scenario_id};
    cfg.spoof.scenario_name          = s.name;
    cfg.spoof.spoofed_constellations = s.spoofed_constellations;
    cfg.spoof.spoofed_PRNs           = s.spoofed_PRNs;
    cfg.spoof.start_epoch            = s.start_epoch;
    cfg.spoof.drift_rate             = s.drift_rate;
    cfg.spoof.target_offset          = s.target_offset;
    cfg.spoof.cn0_boost              = s.cn0_boost;

    obs = rinex_read_obs(fullfile(cfg.paths.obs,'authentic.obs'), cfg);
    nav = rinex_read_nav(fullfile(cfg.paths.nav,'authentic.nav'), cfg);
    obs_sp = inject_spoofing(obs, nav, cfg);

    epochs_all = unique(obs_sp.GPS.time);
    t_e = epochs_all(target_epoch);

    fprintf('\n=======================================================\n');
    fprintf('  EPOCH DIAGNOSTIC — %s, epoch %d\n', s.name, target_epoch);
    fprintf('  t = %s\n', string(t_e));
    fprintf('  spoofed constellations: %s, PRNs: %s\n', ...
        strjoin(s.spoofed_constellations,'+'), mat2str(getfield_safe(s.spoofed_PRNs)));
    fprintf('=======================================================\n');

    % --- Rebuild Stage 2 at this epoch -----------------------------------
    oe = extract_epoch_flat(obs_sp, t_e);
    rr = raim_fde(oe, nav, cfg, t_e);
    ir = inter_constellation(oe, nav, cfg, t_e);
    cr = classify_spoofed_sats(rr, ir, oe, cfg);

    % --- Report classification -------------------------------------------
    fprintf('\n[CLASSIFY] trusted=%d suspect=%d spoofed=%d attack_type=%s\n', ...
        cr.n_trusted, cr.n_suspect, cr.n_spoofed, cr.attack_type);
    if isfield(cr,'recommended_action')
        fprintf('[CLASSIFY] recommended_action: %s\n', cr.recommended_action);
    end

    % --- Per-satellite status --------------------------------------------
    fprintf('\nPer-satellite status this epoch:\n');
    n_by_const_trusted = containers.Map('KeyType','char','ValueType','double');
    if isfield(cr,'sat_list') && ~isempty(cr.sat_list)
        for k = 1:numel(cr.sat_list)
            sl = cr.sat_list(k);
            st = sl.status;
            mark = '';
            if ~strcmp(st,'trusted'), mark = '   <-- excluded/down-weighted'; end
            fprintf('  %-8s PRN %3d : %-8s%s\n', sl.constellation, sl.prn, st, mark);
            if strcmp(st,'trusted')
                if isKey(n_by_const_trusted, sl.constellation)
                    n_by_const_trusted(sl.constellation) = n_by_const_trusted(sl.constellation)+1;
                else
                    n_by_const_trusted(sl.constellation) = 1;
                end
            end
        end
    end

    % --- Trusted satellites per constellation (geometry after exclusion) --
    fprintf('\nTrusted satellites remaining per constellation:\n');
    ks = keys(n_by_const_trusted);
    total_trusted = 0;
    for i=1:numel(ks)
        fprintf('  %-8s : %d\n', ks{i}, n_by_const_trusted(ks{i}));
        total_trusted = total_trusted + n_by_const_trusted(ks{i});
    end
    fprintf('  TOTAL trusted: %d\n', total_trusted);
    n_const = numel(ks);
    fprintf('  constellations with >=1 trusted sat: %d\n', n_const);
    if total_trusted < 5
        fprintf('  *** WARNING: fewer than 5 trusted sats -> weak geometry\n');
    end
    if n_const < 2
        fprintf('  *** WARNING: only 1 constellation trusted -> no ISB observability\n');
    end

    % --- Inter-constellation detail (the 2v2 ambiguity check) ------------
    fprintf('\nInter-constellation result:\n');
    if isfield(ir,'spoofing_suspected')
        fprintf('  spoofing_suspected: %d\n', ir.spoofing_suspected);
    end
    if isfield(ir,'outlier_constellations') && ~isempty(ir.outlier_constellations)
        fprintf('  outlier constellations: %s\n', strjoin(cellstr(ir.outlier_constellations),', '));
    end
    if isfield(ir,'max_pairwise_dist')
        fprintf('  max pairwise dist: %.2f m (threshold %.1f m)\n', ...
            ir.max_pairwise_dist, cfg.identify.inter_const_threshold);
    end

    % --- RAIM detail ------------------------------------------------------
    fprintf('\nRAIM-FDE result:\n');
    if isfield(rr,'fault_detected')
        fprintf('  fault_detected: %d  n_excluded: %d\n', rr.fault_detected, rr.n_excluded);
    end
    if isfield(rr,'spoofed_sats') && ~isempty(rr.spoofed_sats)
        fprintf('  RAIM-excluded:');
        for k=1:numel(rr.spoofed_sats)
            fprintf(' %s%d', rr.spoofed_sats{k}.constellation, rr.spoofed_sats{k}.prn);
        end
        fprintf('\n');
    end
end

% ------------------------------------------------------------------------
function v = getfield_safe(prn_struct)
    v = [];
    if isstruct(prn_struct)
        f = fieldnames(prn_struct);
        for i=1:numel(f), v = [v, prn_struct.(f{i})(:)']; end %#ok<AGROW>
    end
end

function oe = extract_epoch_flat(obs, t_e)
    C = {'GPS','Galileo','BeiDou','GLONASS'};
    oe.time=t_e; oe.prn=[]; oe.constellation={}; oe.pseudorange=[]; oe.cn0=[];
    for i=1:numel(C)
        c=C{i}; if ~isfield(obs,c), continue; end
        m=(obs.(c).time==t_e);
        p=obs.(c).prn(m); pr=obs.(c).pseudorange_L1(m);
        cn=nan(size(p)); if isfield(obs.(c),'cn0'), cn=obs.(c).cn0(m); end
        v=~isnan(pr)&pr>0; p=p(v); pr=pr(v); cn=cn(v);
        oe.prn=[oe.prn,p(:)'];
        oe.constellation=[oe.constellation,repmat({c},1,numel(p))];
        oe.pseudorange=[oe.pseudorange,pr(:)'];
        oe.cn0=[oe.cn0,cn(:)'];
    end
end
