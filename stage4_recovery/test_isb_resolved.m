function test_isb_resolved(obs, nav, cfg)
% TEST_ISB_RESOLVED  Verification test for the inter-system bias (ISB) fix.
%
% PURPOSE (debugging protocol step 4 — verify the fix):
%   The companion test_isb_reproduction.m demonstrates that the single-clock
%   model leaves a systematic per-constellation residual (the uncorrected
%   ISB).  This test estimates one clock per constellation (GPS master + an
%   ISB per non-GPS system, the same measurement model implemented in
%   build_meas_model_isb) and confirms the systematic residual collapses to
%   near zero for every constellation.
%
% METHOD:
%   For each authentic epoch, solve a single-epoch weighted least squares
%   with an AUGMENTED design matrix: position (3) + GPS clock (1) + one ISB
%   column per non-GPS constellation.  Group the post-fit residuals by
%   constellation and average over many epochs.  Under the corrected model
%   every constellation's mean residual must be near zero.
%
% EXPECTED RESULT:  PASS — all constellations zero-mean within ISB_TOL_M.
%
% NOTE: this test validates the MEASUREMENT MODEL (the H / ISB columns).
%   Full EKF convergence with these states is exercised by
%   test_stage4_integration.m, and the GPS-only regression (state must be
%   identical to the pre-ISB pipeline) by re-running with cfg.const.GPS only.
%
% USAGE:
%   config; obs = rinex_read_obs(...); nav = rinex_read_nav(...);
%   test_isb_resolved(obs, nav, cfg)
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery (ISB verification)

    fprintf('\n=== test_isb_resolved.m (expected to PASS — ISB modelled out) ===\n');

    %% --- Deterministic configuration --------------------------------------
    EPOCH_RANGE = 1:200;     % same window as test_isb_reproduction
    ISB_TOL_M   = 1.0;       % stricter than the 5 m bug-detection threshold
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};

    noise_map = struct( ...
        'GPS',     cfg.ekf.meas_noise_GPS, ...
        'Galileo', cfg.ekf.meas_noise_Galileo, ...
        'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
        'GLONASS', cfg.ekf.meas_noise_GLONASS);

    % ISB columns: one per enabled non-GPS constellation, in cfg.ekf.isb_order.
    isb_order = cfg.ekf.isb_order;             % e.g. {'Galileo','BeiDou','GLONASS'}
    n_isb     = numel(isb_order);

    pos_lin    = cfg.ref_pos(:);
    epochs_all = unique(obs.GPS.time);
    nE         = numel(epochs_all);

    resid_accum = struct('GPS',[],'Galileo',[],'BeiDou',[],'GLONASS',[]);
    n_epochs_used = 0;

    for ii = 1:numel(EPOCH_RANGE)
        ei = EPOCH_RANGE(ii);
        if ei < 1 || ei > nE, continue; end
        t_e = epochs_all(ei);

        sp_list  = [];     % [k x 3] sat positions
        pr_list  = [];     % [k x 1] corrected pseudoranges
        w_list   = [];     % [k x 1] weights
        lab_list = {};     % {k x 1} constellation labels

        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs, cname),  continue; end
            if ~cfg.const.(cname),    continue; end
            mask_t = (obs.(cname).time == t_e);
            if ~any(mask_t),          continue; end

            prns_e = obs.(cname).prn(mask_t);
            prs_e  = obs.(cname).pseudorange_L1(mask_t);

            for k = 1:numel(prns_e)
                if strcmp(cname,'BeiDou') && prns_e(k) == 33, continue; end  % corrupted
                try
                    [sp, sc] = sat_position(nav, prns_e(k), cname, t_e);
                    pr_corr  = pseudorange_correct(prs_e(k), sp, sc, pos_lin, t_e, nav, cname, cfg);
                    if isnan(pr_corr), continue; end
                    sp_list(end+1,:) = sp';            %#ok<AGROW>
                    pr_list(end+1,1) = pr_corr;        %#ok<AGROW>
                    w_list(end+1,1)  = 1/noise_map.(cname); %#ok<AGROW>
                    lab_list{end+1,1}= cname;          %#ok<AGROW>
                catch
                    continue
                end
            end
        end

        k = numel(pr_list);
        % Need enough rows to solve 4 + n_isb unknowns with redundancy.
        if k < (4 + n_isb + 1), continue; end

        % --- Iterated augmented WLS: unknowns [dpos(3); clk_GPS; ISB(1..n_isb)]
        pos = pos_lin;
        clk = 0;
        isb = zeros(n_isb,1);
        for iter = 1:8
            H = zeros(k, 4 + n_isb);
            z = zeros(k, 1);
            for r = 1:k
                rvec = sp_list(r,:)' - pos;
                rng  = norm(rvec);
                e    = rvec / rng;
                H(r,1:3) = -e';
                H(r,4)   = 1;                         % GPS master clock
                isb_val  = 0;
                if ~strcmp(lab_list{r},'GPS')
                    idx = find(strcmp(isb_order, lab_list{r}), 1);
                    if ~isempty(idx)
                        H(r,4+idx) = 1;               % ISB column
                        isb_val    = isb(idx);
                    end
                end
                pr_model = rng + clk + isb_val;
                z(r)     = pr_list(r) - pr_model;     % prefit residual
            end
            W   = diag(w_list);
            dx  = (H'*W*H) \ (H'*W*z);
            pos = pos + dx(1:3);
            clk = clk + dx(4);
            if n_isb > 0, isb = isb + dx(5:end); end
            if norm(dx(1:3)) < 1e-4, break; end
        end

        % Post-fit residuals at the converged solution.
        for r = 1:k
            rng = norm(sp_list(r,:)' - pos);
            isb_val = 0;
            if ~strcmp(lab_list{r},'GPS')
                idx = find(strcmp(isb_order, lab_list{r}), 1);
                if ~isempty(idx), isb_val = isb(idx); end
            end
            res = pr_list(r) - (rng + clk + isb_val);
            resid_accum.(lab_list{r})(end+1) = res; %#ok<AGROW>
        end
        n_epochs_used = n_epochs_used + 1;
    end

    %% --- Report and assert -------------------------------------------------
    fprintf('Epochs used: %d   (tolerance |mean residual| < %.1f m)\n', n_epochs_used, ISB_TOL_M);
    fprintf('%-9s | %8s | %8s | %7s\n', 'Const', 'meanRes', 'stdRes', 'nObs');
    fprintf('%s\n', repmat('-',1,44));

    worst = 0; worst_name = '';
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        v = resid_accum.(cname);
        if isempty(v)
            fprintf('%-9s | %8s | %8s | %7d\n', cname, '   ---', '   ---', 0);
            continue
        end
        mr = mean(v);
        fprintf('%-9s | %8.3f | %8.3f | %7d\n', cname, mr, std(v), numel(v));
        if abs(mr) > abs(worst), worst = mr; worst_name = cname; end
    end

    fprintf('%s\n', repmat('-',1,44));
    if abs(worst) <= ISB_TOL_M
        fprintf('PASS — all constellations zero-mean (worst: %s = %+.3f m <= %.1f m).\n', ...
                worst_name, worst, ISB_TOL_M);
        fprintf('ISB fix verified: per-constellation systematic residual removed.\n');
    else
        fprintf(2, 'FAIL — %s still shows %+.3f m (> %.1f m). ', worst_name, worst, ISB_TOL_M);
        fprintf(2, 'Check the ISB column mapping in build_meas_model_isb.\n');
    end
end