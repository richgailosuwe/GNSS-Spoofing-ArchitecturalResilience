function test_corrected_pseudorange(obs, nav, cfg)
% TEST_CORRECTED_PSEUDORANGE  Staged validation of the transmit-time + group
% delay measurement model, with per-effect attribution AND a seed-robustness
% check against the "you only got low error because you started from truth"
% objection.
%
% PART 1 — ATTRIBUTION (truth-seeded), mean GPS-only WLS error over 50 epochs:
%   A. BASELINE : reception-time sat_pos, no group delay   (current code)
%   B. +TX-TIME : transmit-time sat_pos, no group delay
%   C. +TX+TGD  : transmit-time sat_pos + GPS TGD          (full fix)
%
% PART 2 — SEED ROBUSTNESS:
%   Re-run the full model (C) but seed BOTH the WLS start AND the correction
%   linearization point at an OFFSET position (truth + [1000;1000;1000] m),
%   using a two-pass outer loop (solve, re-linearize corrections at the
%   solution, solve again) to mirror real coarse-start behaviour. If the
%   offset-seeded error stays close to the truth-seeded error, the 2.77 m
%   result is not an artefact of starting from the surveyed coordinate.
%
% PASS CONDITIONS:
%   (1) truth-seeded full model mean error  < 5 m
%   (2) offset-seeded full model mean error < 10 m  AND within 3 m of (1)
%
% RTKLIB single-point (Q=5) on the same files: ~14.5 m mean (user-computed).
%
% USAGE:
%   config; obs = rinex_read_obs(...); nav = rinex_read_nav(...);
%   test_corrected_pseudorange(obs, nav, cfg)
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

    fprintf('\n=== test_corrected_pseudorange.m (attribution + seed robustness) ===\n');

    C_LIGHT  = 299792458.0;
    rec_true = cfg.ref_pos(:);
    epochs   = unique(obs.GPS.time);
    N        = 50;

    %% ---- PART 1: attribution (truth-seeded) -------------------------------
    errA=[]; errB=[]; errC=[];
    for ei = 1:N
        t_e=epochs(ei); m=(obs.GPS.time==t_e);
        prns=obs.GPS.prn(m); prs=obs.GPS.pseudorange_L1(m);
        spA=[];prA=[];spB=[];prB=[];spC=[];prC=[];w=[];
        for k=1:numel(prns)
            pr_raw=prs(k); if isnan(pr_raw)||pr_raw<=0, continue; end
            prn=prns(k);
            [spa,sca]=sat_position(nav,prn,'GPS',t_e);
            if any(isnan(spa)), continue; end
            pa=pseudorange_correct(pr_raw,spa,sca,rec_true,t_e,nav,'GPS',cfg);
            if isnan(pa), continue; end
            tau=pr_raw/C_LIGHT;
            for it=1:3
                t_tx=t_e-seconds(tau);
                [spb,scb]=sat_position(nav,prn,'GPS',t_tx);
                if any(isnan(spb)), break; end
                tau=norm(spb-rec_true)/C_LIGHT;
            end
            if any(isnan(spb)), continue; end
            pb=pseudorange_correct(pr_raw,spb,scb,rec_true,t_e,nav,'GPS',cfg);
            if isnan(pb), continue; end
            gd=get_group_delay(nav,prn,'GPS',t_tx);
            pc=pb-gd;
            spA(end+1,:)=spa';prA(end+1,1)=pa; %#ok<AGROW>
            spB(end+1,:)=spb';prB(end+1,1)=pb; %#ok<AGROW>
            spC(end+1,:)=spb';prC(end+1,1)=pc; %#ok<AGROW>
            w(end+1,1)=1/cfg.ekf.meas_noise_GPS; %#ok<AGROW>
        end
        if numel(prA)<cfg.identify.min_sats, continue; end
        pA=wls_solver(prA,spA,w,rec_true);
        pB=wls_solver(prB,spB,w,rec_true);
        pC=wls_solver(prC,spC,w,rec_true);
        errA(end+1)=norm(pA-rec_true); %#ok<AGROW>
        errB(end+1)=norm(pB-rec_true); %#ok<AGROW>
        errC(end+1)=norm(pC-rec_true); %#ok<AGROW>
    end
    mA=mean(errA); mB=mean(errB); mC=mean(errC);

    fprintf('\nPART 1 — attribution (truth-seeded), epochs used: %d\n', numel(errA));
    fprintf('%-28s | mean err | p95 err\n','Model variant');
    fprintf('%s\n',repmat('-',1,52));
    fprintf('%-28s | %7.2f  | %6.2f\n','A baseline (recv-time)',mA,prctile(errA,95));
    fprintf('%-28s | %7.2f  | %6.2f\n','B +transmit-time',mB,prctile(errB,95));
    fprintf('%-28s | %7.2f  | %6.2f\n','C +transmit-time +TGD',mC,prctile(errC,95));
    fprintf('%s\n',repmat('-',1,52));
    fprintf('Transmit-time: %+6.2f m | TGD: %+6.2f m | Total: %+6.2f m\n', mB-mA, mC-mB, mC-mA);

    %% ---- PART 2: seed robustness (offset-seeded, two outer passes) --------
    rec_seed = rec_true + [1000; 1000; 1000];   % deliberately wrong start
    errC_off = [];
    for ei = 1:N
        t_e=epochs(ei); m=(obs.GPS.time==t_e);
        prns=obs.GPS.prn(m); prs=obs.GPS.pseudorange_L1(m);

        rec_lin = rec_seed;          % correction linearization point
        pos_sol = rec_seed;
        for outer = 1:2              % solve, re-linearize at solution, solve
            spC=[]; prC=[]; w=[];
            for k=1:numel(prns)
                pr_raw=prs(k); if isnan(pr_raw)||pr_raw<=0, continue; end
                prn=prns(k);
                tau=pr_raw/C_LIGHT;
                for it=1:3
                    t_tx=t_e-seconds(tau);
                    [spb,scb]=sat_position(nav,prn,'GPS',t_tx);
                    if any(isnan(spb)), break; end
                    tau=norm(spb-rec_lin)/C_LIGHT;
                end
                if any(isnan(spb)), continue; end
                pb=pseudorange_correct(pr_raw,spb,scb,rec_lin,t_e,nav,'GPS',cfg);
                if isnan(pb), continue; end
                pc=pb-get_group_delay(nav,prn,'GPS',t_tx);
                spC(end+1,:)=spb'; prC(end+1,1)=pc; %#ok<AGROW>
                w(end+1,1)=1/cfg.ekf.meas_noise_GPS; %#ok<AGROW>
            end
            if numel(prC)<cfg.identify.min_sats, pos_sol=[NaN;NaN;NaN]; break; end
            pos_sol = wls_solver(prC, spC, w, rec_lin);
            rec_lin = pos_sol;       % re-linearize corrections at the solution
        end
        if any(isnan(pos_sol)), continue; end
        errC_off(end+1)=norm(pos_sol-rec_true); %#ok<AGROW>
    end
    mC_off = mean(errC_off);

    fprintf('\nPART 2 — seed robustness (start offset by %.0f m, 2 outer passes)\n', norm([1000 1000 1000]));
    fprintf('  Full model, TRUTH-seeded  mean error: %6.2f m\n', mC);
    fprintf('  Full model, OFFSET-seeded mean error: %6.2f m  (epochs %d)\n', mC_off, numel(errC_off));
    fprintf('  Difference (offset - truth):          %+6.2f m\n', mC_off - mC);

    %% ---- Verdict ----------------------------------------------------------
    cond1 = mC < 5.0;
    cond2 = (mC_off < 10.0) && (abs(mC_off - mC) < 3.0);
    fprintf('\n%s\n',repmat('=',1,52));
    fprintf('  Cond 1 (truth-seeded < 5 m):              %s (%.2f m)\n', tf(cond1), mC);
    fprintf('  Cond 2 (offset-seeded < 10 m & ~truth):   %s (%.2f m)\n', tf(cond2), mC_off);
    if cond1 && cond2
        fprintf('  RESULT: PASS — low error is NOT an artefact of the truth seed.\n');
        fprintf('  The transmit-time fix is robust to starting position.\n');
    else
        fprintf('  RESULT: REVIEW — see conditions above.\n');
        if ~cond2
            fprintf('  Offset seed inflated error: report coarse-start behaviour honestly.\n');
        end
    end
    fprintf('  RTKLIB single-point (Q=5) reference: ~14.5 m mean\n');
    fprintf('%s\n',repmat('=',1,52));
end

function s = tf(b)
    if b, s='PASS'; else, s='FAIL'; end
end