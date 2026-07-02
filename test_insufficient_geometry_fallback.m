function test_insufficient_geometry_fallback()
% TEST_INSUFFICIENT_GEOMETRY_FALLBACK
%
% Verifies the ekf_runner fallback that guards against the all-spoofed
% classification collapse under 2-vs-2 inter-constellation ambiguity
% (dual-constellation attacks, Scenarios 4 and 5).
%
% Failure mode (pre-fix): classifier marks every satellite spoofed ->
% apply_exclusion_mask deflates all weights by 1e6 -> EKF update runs on a
% degenerate, gate-blinded measurement set -> position leaks toward the
% spoofed solution (A max 75.34 m at S5 ep2734 vs gate-only B 2.19 m).
%
% Fix: when n_trusted_post_mask < min_sats, the runner treats the
% classification as INDETERMINATE (not literal all-spoofed), restores nominal
% weights, and lets the scalar innovation gate protect position (gate_only
% policy, default).
%
% TWO-TIER ASSERTIONS:
%   T1 SPECIFIC : at S5 collapse epoch 2734, full-pipeline error must drop
%                 from the ~75 m spike to near the gate-only value (~2-3 m).
%   T2 AGGREGATE: over Scenario 5, full-pipeline max must no longer be far
%                 above gate-only max; fallback count > 0; unmitigated C still
%                 bad (sanity that the attack is still dangerous).
%   T3 REGRESSION: Scenario 1 (single-constellation, no collapse) unchanged
%                 -> fallback never fires, final within tolerance of known 6.95.
%
% NOTE: helpers are LOCAL functions (after the main end), not nested, so
% config is called as a script statement (config;) NOT run('config.m'): run()
% uses evalin which trips the static-workspace rule when local functions exist.
%

    PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
    cd(PROJECT_ROOT); config; cfg.verbose = false;

    fprintf('=======================================================\n');
    fprintf('  INSUFFICIENT-GEOMETRY FALLBACK TEST\n');
    fprintf('=======================================================\n\n');

    nav = rinex_read_nav(fullfile(cfg.paths.nav,'authentic.nav'), cfg);

    % ===================================================================
    % Scenario 5: A (full, gate_only fallback), B (gate-only), C (none)
    % ===================================================================
    fprintf('Running Scenario 5 full pipeline (with fallback)...\n');
    [peA5, ekfA5] = run_mode(cfg, nav, 5, true,  true,  'gate_only');
    fprintf('Running Scenario 5 gate-only...\n');
    [peB5, ~    ] = run_mode(cfg, nav, 5, false, true,  'gate_only');
    fprintf('Running Scenario 5 unmitigated...\n');
    [peC5, ~    ] = run_mode(cfg, nav, 5, false, false, 'gate_only');

    EP = 2734;
    n_fb = sum(ekfA5.exclusion_fallback);

    fprintf('\n--- T1: S5 collapse epoch %d ---\n', EP);
    fprintf('  A(full+fallback) err = %.2f m\n', peA5(EP));
    fprintf('  B(gate-only)     err = %.2f m\n', peB5(EP));
    t1 = peA5(EP) < 10;
    fprintf('  T1 (A at collapse < 10 m): %s\n', pf(t1));

    fprintf('\n--- T2: S5 aggregate ---\n');
    fprintf('  A max=%.2f  B max=%.2f  C p95=%.2f  | fallback epochs=%d\n', ...
        max(peA5), max(peB5), prctile(peC5,95), n_fb);
    t2a = max(peA5) < 2.0 * max(peB5) + 5;
    t2b = n_fb > 0;
    t2c = prctile(peC5,95) > 100;
    fprintf('  T2a (A max ~<= B max):     %s\n', pf(t2a));
    fprintf('  T2b (fallback fired >0):   %s\n', pf(t2b));
    fprintf('  T2c (C still bad p95>100): %s\n', pf(t2c));

    % ===================================================================
    % Scenario 1: regression — no collapse, fallback must NOT fire
    % ===================================================================
    fprintf('\nRunning Scenario 1 full pipeline (regression)...\n');
    [peA1, ekfA1] = run_mode(cfg, nav, 1, true, true, 'gate_only');
    n_fb1 = sum(ekfA1.exclusion_fallback);
    fprintf('\n--- T3: S1 regression ---\n');
    fprintf('  final=%.2f median=%.2f p95=%.2f | fallback epochs=%d\n', ...
        peA1(end), median(peA1), prctile(peA1,95), n_fb1);
    t3a = n_fb1 == 0;
    t3b = abs(peA1(end) - 6.95) < 1.0;
    fprintf('  T3a (no fallback in S1):   %s\n', pf(t3a));
    fprintf('  T3b (S1 final ~6.95 m):    %s\n', pf(t3b));

    all_pass = t1 && t2a && t2b && t2c && t3a && t3b;
    fprintf('\n=======================================================\n');
    fprintf('  OVERALL: %s\n', pf(all_pass));
    fprintf('=======================================================\n');
end

%% ===================== LOCAL FUNCTIONS (not nested) =====================

function [pe, ekf] = run_mode(cfg, nav, scn_idx, use_excl, use_gate, ig_policy)
    s = cfg.scenarios{scn_idx};
    c = cfg;
    c.spoof.scenario_name=s.name; c.spoof.spoofed_constellations=s.spoofed_constellations;
    c.spoof.spoofed_PRNs=s.spoofed_PRNs; c.spoof.start_epoch=s.start_epoch;
    c.spoof.drift_rate=s.drift_rate; c.spoof.target_offset=s.target_offset;
    c.spoof.cn0_boost=s.cn0_boost;
    c.stage3.use_innovation_gate = use_gate;
    c.stage3.insufficient_geometry_policy = ig_policy;
    obs = rinex_read_obs(fullfile(c.paths.obs,'authentic.obs'), c);
    obs_sp = inject_spoofing(obs, nav, c);
    if use_excl
        ep = unique(obs_sp.GPS.time); ncl = numel(ep); cr = cell(ncl,1);
        for ii=1:ncl
            oe = extract_epoch_flat(obs_sp, ep(ii));
            rr = raim_fde(oe, nav, c, ep(ii));
            ir = inter_constellation(oe, nav, c, ep(ii));
            cr{ii} = classify_spoofed_sats(rr, ir, oe, c);
        end
        ekf = ekf_runner(obs_sp, nav, cr, c);
    else
        ekf = ekf_runner(obs_sp, nav, {}, c);
    end
    pe = vecnorm(ekf.pos - c.ref_pos', 2, 2);
end

function s = pf(c)
    if c, s='PASS'; else, s='FAIL'; end
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
        oe.prn=[oe.prn,p(:)']; oe.constellation=[oe.constellation,repmat({c},1,numel(p))];
        oe.pseudorange=[oe.pseudorange,pr(:)']; oe.cn0=[oe.cn0,cn(:)'];
    end
end
