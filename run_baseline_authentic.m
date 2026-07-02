function baseline = run_baseline_authentic(verbose)
% RUN_BASELINE_AUTHENTIC  Nominal (no-spoof) performance on authentic BUCU data.
% Thesis Section 6.2 baseline / reference. Mirrors run_pipeline.m EXACTLY but:
%   (1) does NOT call inject_spoofing  -> authentic observations straight through
%   (2) passes {} for classify_results -> all satellites trusted (nominal op)
% Reports nominal position error vs cfg.ref_pos (BUCU is a surveyed IGS coord,
% so error-vs-truth is legitimate) and HPL. This is the clean reference the five
% spoofing scenarios are compared against.
%
% Uses the SAME signatures as run_pipeline.m (verified): cfg.paths.obs/nav,
% rinex_read_obs/nav, ekf_runner(obs,nav,{},cfg), ekf_out.pos [n x 3], .coasted, .hpl
%
% Usage:  run_baseline_authentic        % quiet
%         run_baseline_authentic(true)  % verbose

    if nargin < 1, verbose = true; end
    run('config.m');
    cfg.verbose = logical(verbose);

    addpath(fullfile(cfg.root,'utils'));
    addpath(fullfile(cfg.root,'utils','calibration'));
    addpath(fullfile(cfg.root,'stage4_recovery'));

    % --- Load authentic RINEX (same paths as run_pipeline) ---
    obs_file = fullfile(cfg.paths.obs, 'authentic.obs');
    nav_file = fullfile(cfg.paths.nav, 'authentic.nav');
    if ~isfile(obs_file), error('Observation file not found: %s', obs_file); end
    if ~isfile(nav_file), error('Navigation file not found: %s', nav_file); end

    obs = rinex_read_obs(obs_file, cfg);
    nav = rinex_read_nav(nav_file, cfg);

    % --- NO inject_spoofing. Empty classify => all trusted (ablation B-arm pattern) ---
    ekf_out = ekf_runner(obs, nav, {}, cfg);

    % --- Metrics vs BUCU surveyed reference ---
    pe  = vecnorm(ekf_out.pos - cfg.ref_pos(:)', 2, 2);
    hpl = ekf_out.hpl(:);
    valid = ~isnan(pe);

    fprintf('\n=====================================================\n');
    fprintf('  BASELINE — authentic BUCU, NO spoofing (Section 6.2)\n');
    fprintf('=====================================================\n');
    fprintf('Epochs:                  %d\n', numel(pe));
    fprintf('Position error vs BUCU:   median %.2f m, p95 %.2f m, max %.2f m\n', ...
        median(pe,'omitnan'), prctile(pe(valid),95), max(pe));
    fprintf('Final position error:     %.2f m\n', pe(end));
    if any(~isnan(hpl))
        vh = ~isnan(hpl);
        fprintf('HPL:                      median %.2f m, p95 %.2f m\n', ...
            median(hpl,'omitnan'), prctile(hpl(vh),95));
        both = valid & vh;
        fprintf('HPL >= error every epoch: %s\n', tern(all(hpl(both)>=pe(both)),'YES','NO - CHECK'));
    else
        fprintf('HPL:                      (no valid HPL values returned)\n');
    end
    fprintf('Coasted epochs:           %d / %d\n', sum(ekf_out.coasted), numel(pe));
    fprintf('=====================================================\n');

    baseline = struct('pos_error',pe,'hpl',hpl,'ekf',ekf_out, ...
        'median_err',median(pe,'omitnan'),'p95_err',prctile(pe(valid),95), ...
        'final_err',pe(end),'created',datetime('now'), ...
        'code_notes','authentic no-spoof baseline for Sec 6.2');

    if ~isfolder(cfg.paths.pvt), mkdir(cfg.paths.pvt); end
    save(fullfile(cfg.paths.pvt,'baseline_authentic.mat'),'baseline');
    fprintf('Saved: %s\n', fullfile(cfg.paths.pvt,'baseline_authentic.mat'));
end
function s=tern(c,a,b); if c, s=a; else, s=b; end; end
