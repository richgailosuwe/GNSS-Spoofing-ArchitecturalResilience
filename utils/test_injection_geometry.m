function test_injection_geometry(obs, nav, cfg)
% TEST_INJECTION_GEOMETRY  Verify the corrected position-spoof injection.
%
% PURPOSE (debugging protocol step 4 — verify the fix):
%   Confirm inject_spoofing produces a coherent POSITION displacement equal
%   to +target_offset (direction AND magnitude), not a clock shift.
%
% KEY INSIGHT — why we difference two solutions:
%   The spoofed pseudorange is  rho_spoof = rho_auth + bias_i, so the WLS
%   solution on spoofed data is  rec_false + (authentic SPP error), because
%   the same measurement noise/residual rides along.  Authentic GPS-only SPP
%   at BUCU has a ~50-70 m error floor, so comparing the spoofed solution
%   directly to rec_true conflates the injection with that floor.
%
%   Solving authentic AND spoofed at the SAME epoch with the SAME satellites
%   and DIFFERENCING the two solutions cancels the shared noise/residual
%   exactly, leaving only the injected displacement:
%
%       solved_spoof - solved_auth = rec_false - rec_true = target_offset
%
%   This isolates the injection geometry and is correct to sub-metre.
%   A sign error would give -target_offset; a uniform (clock) drag would give
%   ~0.  Either makes this test FAIL.
%
% EXPECTED RESULT:  PASS (difference within DIFF_TOL_M of target_offset).
%
% USAGE:
%   config; obs = rinex_read_obs(...); nav = rinex_read_nav(...);
%   test_injection_geometry(obs, nav, cfg)
%
% PROJECT:  GNSS Thesis MATLAB Implementation, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

    fprintf('\n=== test_injection_geometry.m (expected to PASS) ===\n');

    DIFF_TOL_M = 1.0;   % spoof-minus-authentic must match target to sub-metre

    rec_true   = cfg.ref_pos(:);
    target_vec = cfg.scenarios{1}.target_offset(:);   % stay in sync with config
    target_mag = norm(target_vec);

    % --- Control scenario: ALL visible GPS PRNs spoofed --------------------
    gps_prns = unique(obs.GPS.prn);

    cfg_t = cfg;
    cfg_t.verbose                       = false;
    cfg_t.spoof.scenario_name           = 'verify_injection_geometry';
    cfg_t.spoof.spoofed_constellations  = {'GPS'};
    cfg_t.spoof.spoofed_PRNs            = struct('GPS', gps_prns(:)');
    cfg_t.spoof.start_epoch             = 120;
    cfg_t.spoof.drift_rate              = 5.0;
    cfg_t.spoof.target_offset           = target_vec;
    cfg_t.spoof.cn0_boost               = 8.0;

    obs_sp = inject_spoofing(obs, nav, cfg_t);

    % --- Saturated epoch (drag at full magnitude) --------------------------
    epochs_all = unique(obs.GPS.time);
    ei  = 250;
    t_e = epochs_all(ei);

    % --- Assemble authentic and spoofed measurements over the SAME sats ----
    mask = (obs.GPS.time == t_e);
    prns = obs.GPS.prn(mask);
    ra_all = obs.GPS.pseudorange_L1(mask);
    rs_all = obs_sp.GPS.pseudorange_L1(mask);

    sp_list = [];  pr_auth = [];  pr_spoof = [];  w_list = [];
    for k = 1:numel(prns)
        try
            [sp, sc] = sat_position(nav, prns(k), 'GPS', t_e);

            pa = pseudorange_correct(ra_all(k), sp, sc, rec_true, t_e, nav, 'GPS', cfg);
            ps = pseudorange_correct(rs_all(k), sp, sc, rec_true, t_e, nav, 'GPS', cfg);
            if isnan(pa) || isnan(ps), continue; end

            sp_list(end+1,:)  = sp(:)';                  %#ok<AGROW>
            pr_auth(end+1,1)  = pa;                      %#ok<AGROW>
            pr_spoof(end+1,1) = ps;                      %#ok<AGROW>
            w_list(end+1,1)   = 1/cfg.ekf.meas_noise_GPS; %#ok<AGROW>
        catch
            continue
        end
    end

    n_used = numel(pr_auth);
    if n_used < 5
        fprintf(2, 'FAIL — only %d GPS sats at epoch %d, need >= 5.\n', n_used, ei);
        return
    end

    % --- Solve both, difference out the shared SPP error -------------------
    [pos_auth,  ~] = wls_solver(pr_auth,  sp_list, w_list, rec_true);
    [pos_spoof, ~] = wls_solver(pr_spoof, sp_list, w_list, rec_true);

    spp_floor = norm(pos_auth(:)  - rec_true);          % authentic SPP error
    inj_disp  = pos_spoof(:) - pos_auth(:);             % isolated injection
    inj_err   = norm(inj_disp - target_vec);            % vs intended

    %% --- Report -----------------------------------------------------------
    fprintf('Epoch %d (saturated), GPS sats used: %d\n', ei, n_used);
    fprintf('Authentic SPP error (floor):  %.2f m   <-- this rode under the old test\n', spp_floor);
    fprintf('Intended displacement: [%+7.1f %+7.1f %+7.1f]  |dp| = %.1f m\n', ...
            target_vec(1), target_vec(2), target_vec(3), target_mag);
    fprintf('Injected (spoof-auth): [%+7.1f %+7.1f %+7.1f]  |dp| = %.1f m\n', ...
            inj_disp(1), inj_disp(2), inj_disp(3), norm(inj_disp));
    fprintf('Injection error:       %.3f m  (tolerance %.1f m)\n', inj_err, DIFF_TOL_M);

    if inj_err <= DIFF_TOL_M
        fprintf('PASS — injection drags the SOLUTION by exactly +target_offset.\n');
        fprintf('Coherent position spoof confirmed; sign correct; SPP floor cancels.\n');
    else
        fprintf(2, 'FAIL — isolated injection does not match target_offset.\n');
        if norm(inj_disp) < 50
            fprintf(2, '  ~0 displacement: looks like a clock shift (uniform drag?).\n');
        elseif norm(inj_disp + target_vec) <= DIFF_TOL_M
            fprintf(2, '  = -target: bias SIGN inverted.\n');
        end
    end
end