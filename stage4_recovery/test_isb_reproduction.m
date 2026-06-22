function test_isb_reproduction(obs, nav, cfg)
% TEST_ISB_REPRODUCTION  Reproduce-first test for the inter-system bias (ISB) bug.
%
% PURPOSE (debugging protocol step 2 — reproduce before fixing):
%   Demonstrate, deterministically and on authentic BUCU data, that the
%   current single-clock measurement model leaves an uncorrected
%   inter-system bias on every non-GPS constellation.
%
% ROOT CAUSE UNDER TEST (traced, not assumed):
%   ekf_runner.m stacks GPS+Galileo+BeiDou+GLONASS into one measurement
%   vector and sets H(i,7)=1 for ALL rows (a single shared GPS clock).
%   pseudorange_correct.m applies sat-clock + Sagnac + iono + tropo only,
%   with NO inter-system time-offset term.  Therefore each non-GPS
%   pseudorange enters the solution carrying an uncorrected inter-system
%   bias (GGTO, BDT offset, GLONASS offset, plus receiver hardware delay).
%
% WHAT THIS TEST MEASURES:
%   For each authentic epoch it solves the SAME single-clock WLS the
%   pipeline uses (all constellations, one clock), then groups the
%   post-fit residuals by constellation and averages them over many epochs.
%   Under a correct model every constellation's residuals are zero-mean.
%   The estimated ISB for a constellation is its mean residual minus the
%   GPS mean residual.
%
% EXPECTED RESULT:  *** THIS TEST IS EXPECTED TO FAIL ***
%   At least one non-GPS constellation will show |ISB| > ISB_TOL_M,
%   proving the bug.  After the ISB states are added to the EKF (the fix),
%   the companion test test_isb_resolved.m must PASS.
%
% USAGE:
%   Run main.m or config first to set paths and load obs/nav/cfg, then:
%     >> test_isb_reproduction(obs, nav, cfg)
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG
% STAGE:    4 — EKF Position Recovery (ISB diagnostic)

    fprintf('\n=== test_isb_reproduction.m (expected to FAIL — demonstrates the bug) ===\n');

    %% --- Deterministic configuration --------------------------------------
    EPOCH_RANGE = 1:200;     % fixed, deterministic — no randomness
    ISB_TOL_M   = 5.0;       % metres; an ISB above this is a real modelling error
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};

    noise_map = struct( ...
        'GPS',     cfg.ekf.meas_noise_GPS, ...
        'Galileo', cfg.ekf.meas_noise_Galileo, ...
        'BeiDou',  cfg.ekf.meas_noise_BeiDou, ...
        'GLONASS', cfg.ekf.meas_noise_GLONASS);

    pos_lin    = cfg.ref_pos(:);            % WLS linearisation point (as in bootstrap_wls)
    epochs_all = unique(obs.GPS.time);
    nE         = numel(epochs_all);

    % Accumulate residuals per constellation across epochs.
    resid_accum = struct('GPS',[],'Galileo',[],'BeiDou',[],'GLONASS',[]);

    n_epochs_used = 0;

    for ii = 1:numel(EPOCH_RANGE)
        ei = EPOCH_RANGE(ii);
        if ei < 1 || ei > nE, continue; end
        t_e = epochs_all(ei);

        all_pr   = [];
        all_sp   = [];
        all_w    = [];
        all_lab  = {};   % constellation label per measurement row

        for ci = 1:numel(constellations)
            cname = constellations{ci};
            if ~isfield(obs, cname),          continue; end
            if ~cfg.const.(cname),            continue; end   % respect enable flags
            mask_t = (obs.(cname).time == t_e);
            if ~any(mask_t),                  continue; end

            prns_e = obs.(cname).prn(mask_t);
            prs_e  = obs.(cname).pseudorange_L1(mask_t);

            for k = 1:numel(prns_e)
                % Known data anomaly: BeiDou PRN 33 corrupted in authentic.obs.
                if strcmp(cname,'BeiDou') && prns_e(k) == 33, continue; end
                try
                    [sp, sc] = sat_position(nav, prns_e(k), cname, t_e);
                    pr_corr  = pseudorange_correct(prs_e(k), sp, sc, pos_lin, t_e, nav, cname, cfg);
                    if isnan(pr_corr), continue; end
                    all_pr(end+1)   = pr_corr;          %#ok<AGROW>
                    all_sp(end+1,:) = sp';              %#ok<AGROW>
                    all_w(end+1)    = 1 / noise_map.(cname); %#ok<AGROW>
                    all_lab{end+1}  = cname;            %#ok<AGROW>
                catch
                    continue
                end
            end
        end

        if numel(all_pr) < cfg.identify.min_sats, continue; end

        % Single-clock WLS — exactly the model the pipeline uses.
        % wls_solver signature: [pos, clk_bias, residuals, H, W] = wls_solver(pr, sp, w, pos_init)
        [~, ~, residuals] = wls_solver(all_pr(:), all_sp, all_w(:), pos_lin);
        if any(isnan(residuals)), continue; end

        % Group post-fit residuals by constellation.
        for r = 1:numel(residuals)
            lab = all_lab{r};
            resid_accum.(lab)(end+1) = residuals(r); %#ok<AGROW>
        end
        n_epochs_used = n_epochs_used + 1;
    end

    %% --- Report per-constellation mean residual and estimated ISB ----------
    fprintf('Epochs used: %d   (tolerance |ISB| < %.1f m)\n', n_epochs_used, ISB_TOL_M);
    fprintf('%-9s | %8s | %8s | %7s | %s\n', 'Const', 'meanRes', 'stdRes', 'nObs', 'ISB vs GPS');
    fprintf('%s\n', repmat('-',1,60));

    means = struct();
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        v = resid_accum.(cname);
        if isempty(v)
            means.(cname) = NaN;
            fprintf('%-9s | %8s | %8s | %7d | %s\n', cname, '   ---', '   ---', 0, '(absent)');
        else
            means.(cname) = mean(v);
        end
    end

    gps_mean = means.GPS;
    worst_isb = 0;  worst_name = '';
    for ci = 1:numel(constellations)
        cname = constellations{ci};
        v = resid_accum.(cname);
        if isempty(v), continue; end
        isb = means.(cname) - gps_mean;     % estimated inter-system bias vs GPS
        if ~strcmp(cname,'GPS') && abs(isb) > abs(worst_isb)
            worst_isb = isb;  worst_name = cname;
        end
        fprintf('%-9s | %8.2f | %8.2f | %7d | %+8.2f m\n', ...
                cname, means.(cname), std(v), numel(v), isb);
    end

    %% --- Assertion: non-GPS constellations must be ~zero-mean (they are NOT)
    fprintf('%s\n', repmat('-',1,60));
    if abs(worst_isb) > ISB_TOL_M
        fprintf(2, 'FAIL (as expected): %s shows an uncorrected ISB of %+.2f m ', ...
                worst_name, worst_isb);
        fprintf(2, '(> %.1f m).\n', ISB_TOL_M);
        fprintf(2, 'Root cause reproduced: single shared clock + no ISB term.\n');
        fprintf('\nThis failing test is the precondition for implementing the fix.\n');
    else
        fprintf('PASS: no constellation exceeds the ISB tolerance.\n');
        fprintf('If this passes on authentic multi-constellation data, re-examine\n');
        fprintf('whether an ISB correction is already being applied somewhere.\n');
    end
end