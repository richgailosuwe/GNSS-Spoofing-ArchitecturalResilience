function results = run_ablation(scenario_id)
% RUN_ABLATION  Three-run mitigation ablation for one spoofing scenario.
%
%   results = run_ablation(scenario_id)
%
% Runs the SAME spoofed observations through Stage 4 EKF recovery under three
% mitigation configurations, to isolate what each protection layer contributes:
%
%   A. Full mitigation  : Stage 2 classification + exclusion + scalar gate
%   B. Gate only        : exclusion OFF (all trusted), scalar gate ON
%   C. Unmitigated      : exclusion OFF, scalar gate OFF (true baseline)
%
% Interpretation:
%   A == B  >> C   ->  scalar gating alone defeats the spoof; exclusion adds
%                      interpretability/classification, not raw protection.
%   A  > B  >  C   ->  layered mitigation adds incremental protection.
%   A  >> B,C      ->  classification/exclusion is essential.
%
% The attack model is identical across all three runs — only mitigation
% layers are toggled — so the comparison is fair.
%
% Requires: cfg.stage3.use_innovation_gate flag support in ekf_runner.m.
%

    if nargin < 1, scenario_id = 1; end

    cfg = struct();
    run('config.m');
    REF_SCALE = 185.2;
    REF_SCALE_2X = 370.4;   % descriptive only; not a certified aviation limit

    % --- Resolve scenario and set spoof params ---------------------------
    s = cfg.scenarios{scenario_id};
    cfg.spoof.scenario_name          = s.name;
    cfg.spoof.spoofed_constellations = s.spoofed_constellations;
    cfg.spoof.spoofed_PRNs           = s.spoofed_PRNs;
    cfg.spoof.start_epoch            = s.start_epoch;
    cfg.spoof.drift_rate             = s.drift_rate;
    cfg.spoof.target_offset          = s.target_offset;
    cfg.spoof.cn0_boost              = s.cn0_boost;

    fprintf('\n=======================================================\n');
    fprintf('  MITIGATION ABLATION — %s (%s)\n', s.name, ...
        strjoin(s.spoofed_constellations,'+'));
    fprintf('  Attack: start ep %d, target %.1f m\n', ...
        s.start_epoch, norm(s.target_offset));
    fprintf('=======================================================\n');

    % --- Load + inject (once; shared by all three runs) -------------------
    obs = rinex_read_obs(fullfile(cfg.paths.obs,'authentic.obs'), cfg);
    nav = rinex_read_nav(fullfile(cfg.paths.nav,'authentic.nav'), cfg);
    obs_sp = inject_spoofing(obs, nav, cfg);

    % --- Build Stage 2 classify_results (needed for run A) ----------------
    epochs_all = unique(obs_sp.GPS.time);
    n_ep = numel(epochs_all);
    classify_results = cell(n_ep,1);
    for ei = 1:n_ep
        t_e = epochs_all(ei);
        oe  = extract_epoch_flat(obs_sp, t_e);
        rr  = raim_fde(oe, nav, cfg, t_e);
        ir  = inter_constellation(oe, nav, cfg, t_e);
        classify_results{ei} = classify_spoofed_sats(rr, ir, oe, cfg);
    end

    % --- Run A: full mitigation ------------------------------------------
    cfgA = cfg; cfgA.stage3.use_innovation_gate = true;
    ekfA = ekf_runner(obs_sp, nav, classify_results, cfgA);

    % --- Run B: exclusion OFF, gate ON -----------------------------------
    cfgB = cfg; cfgB.stage3.use_innovation_gate = true;
    ekfB = ekf_runner(obs_sp, nav, {}, cfgB);

    % --- Run C: exclusion OFF, gate OFF (true unmitigated) ---------------
    cfgC = cfg; cfgC.stage3.use_innovation_gate = false;
    ekfC = ekf_runner(obs_sp, nav, {}, cfgC);

    % --- Metrics helper ---------------------------------------------------
    pe = @(e) vecnorm(e.pos - cfg.ref_pos', 2, 2);
    peA = pe(ekfA); peB = pe(ekfB); peC = pe(ekfC);

    a = summ(peA, ekfA, s.start_epoch, REF_SCALE_2X, REF_SCALE);
    b = summ(peB, ekfB, s.start_epoch, REF_SCALE_2X, REF_SCALE);
    c = summ(peC, ekfC, s.start_epoch, REF_SCALE_2X, REF_SCALE);

    % --- Report -----------------------------------------------------------
    fprintf('\n%-22s %10s %10s %10s\n', '', 'A:full', 'B:gate', 'C:none');
    fprintf('%s\n', repmat('-',1,56));
    fprintf('%-22s %10.2f %10.2f %10.2f\n','final err [m]',     a.final,  b.final,  c.final);
    fprintf('%-22s %10.2f %10.2f %10.2f\n','median err [m]',    a.med,    b.med,    c.med);
    fprintf('%-22s %10.2f %10.2f %10.2f\n','p95 err [m]',       a.p95,    b.p95,    c.p95);
    fprintf('%-22s %10.2f %10.2f %10.2f\n','max err [m]',       a.max,    b.max,    c.max);
    fprintf('%-22s %10.2f %10.2f %10.2f\n','err @ ep200 [m]',   a.e200,   b.e200,   c.e200);
    fprintf('%-22s %10d %10d %10d\n','rejected post-attack',    a.rej,    b.rej,    c.rej);
    fprintf('%-22s %10d %10d %10d\n','coasted epochs',          a.coast,  b.coast,  c.coast);
    fprintf('%s\n', repmat('-',1,56));
    fprintf('RNP-0.1 ref scale=%.1f m  2x ref scale=%.1f m (descriptive, not certified)\n', REF_SCALE, REF_SCALE_2X);

    % --- Interpretation ---------------------------------------------------
    fprintf('\nInterpretation:\n');
    if c.p95 > 5*max(a.p95,1)
        fprintf('  C drifts far beyond A -> mitigation provides clear recovery.\n');
        if abs(a.p95-b.p95) < 0.1*max(a.p95,1)
            fprintf('  A ~= B -> scalar gating alone defeats this spoof;\n');
            fprintf('           Stage 2 exclusion adds classification, not raw protection.\n');
        else
            fprintf('  A < B  -> exclusion adds protection beyond gating.\n');
        end
    else
        fprintf('  C does NOT drift far -> EKF geometry/redundancy absorbs this\n');
        fprintf('  attack even unmitigated. This scenario does not demonstrate\n');
        fprintf('  recovery value at the position level (report honestly).\n');
    end

    results = struct('scenario',s.name,'A',a,'B',b,'C',c, ...
                     'peA',peA,'peB',peB,'peC',peC, ...
                     'start_epoch',s.start_epoch);

    % --- Persist to disk for thesis provenance ---------------------------
    results.config_snapshot = cfg;
    results.created         = datetime('now');
    results.code_notes      = 'post BGD-fix #11 + insufficient-geometry-fallback #12';
    out_dir = fullfile(cfg.root, 'results', 'ablation');
    if ~isfolder(out_dir), mkdir(out_dir); end
    out_path = fullfile(out_dir, sprintf('%s_ablation.mat', s.name));
    save(out_path, 'results');
    fprintf('\nAblation saved: %s\n', out_path);
end

% ------------------------------------------------------------------------
function r = summ(pe, ekf, start_ep, REF_SCALE_2X, REF_SCALE) %#ok<INUSD>
    r.final = pe(end);
    r.med   = median(pe);
    r.p95   = prctile(pe,95);
    r.max   = max(pe);
    r.e200  = pe(min(200,numel(pe)));
    r.rej   = sum(ekf.n_rejected(start_ep:end));
    r.coast = sum(ekf.coasted(start_ep:end));
    r.hpl_p95 = prctile(ekf.hpl(~isnan(ekf.hpl)),95);
    r.pct_below_ref2x = mean(ekf.hpl(~isnan(ekf.hpl)) < REF_SCALE_2X)*100;
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
