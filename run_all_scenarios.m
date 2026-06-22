% run_all_scenarios.m
% Batch runner: executes all five spoofing scenarios sequentially.
%
% Usage:
%   run_all_scenarios
%
% Results saved to results/pvt/<scenario_name>_pipeline.mat per scenario.
% Summary table printed at the end.
%
% Scenarios:
%   1 = scenario_1_gps           GPS only
%   2 = scenario_2_galileo       Galileo only
%   3 = scenario_3_beidou        BeiDou only
%   4 = scenario_4_gps_glonass   GPS + GLONASS
%   5 = scenario_5_gps_galileo   GPS + Galileo
%
% INTEGRITY NOTE:
%   Position errors and HPL are reported as indicative metrics.
%   The 185.2 m column is the 0.1 NM RNP lateral reference scale
%   used for reference only — this is not a certified aviation integrity
%   claim. HPL is computed with a conservative RAIM-style multiplier
%   (K_H = 6.18) and is not a certified protection level.
%
% PROJECT:  GNSS Thesis, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

clc;
fprintf('========================================\n');
fprintf('  Batch Pipeline — All 5 Scenarios\n');
fprintf('========================================\n\n');

run('config.m');
n_scenarios = numel(cfg.scenarios);
RNP01_ref   = 185.2;  % RNP-0.1 reference scale [m], descriptive only

results_summary = struct();
t_batch_start   = tic;

for k = 1:n_scenarios
    s = cfg.scenarios{k};
    fprintf('[%d/%d] %s (%s)...', k, n_scenarios, s.name, ...
        strjoin(s.spoofed_constellations, '+'));

    t_start = tic;
    try
        result = run_pipeline(k, false);  % quiet — no per-epoch verbose output

        pos_errors = vecnorm(result.ekf.pos - result.ref_pos', 2, 2);
        n_coasted  = sum(result.ekf.coasted);
        n_spoof_ep = sum(cellfun(@(r) ~isempty(r) && any(r.spoofed_mask), ...
                         result.classification));

        % HPL metrics (indicative, K_H = 6.18, not certified)
        if isfield(result.ekf, 'hpl') && ~isempty(result.ekf.hpl)
            hpl_vals         = result.ekf.hpl(~isnan(result.ekf.hpl));
            hpl_p95          = prctile(hpl_vals, 95);
            hpl_pct_within   = mean(hpl_vals < RNP01_ref) * 100;
        else
            hpl_p95        = NaN;
            hpl_pct_within = NaN;
        end

        results_summary(k).scenario      = s.name;
        results_summary(k).consts        = strjoin(s.spoofed_constellations, '+');
        results_summary(k).final_err     = pos_errors(end);
        results_summary(k).median_err    = median(pos_errors);
        results_summary(k).p95_err       = prctile(pos_errors, 95);
        results_summary(k).hpl_p95       = hpl_p95;
        results_summary(k).hpl_pct       = hpl_pct_within;
        results_summary(k).coasted       = n_coasted;
        results_summary(k).spoof_ep      = n_spoof_ep;
        results_summary(k).osnma         = result.osnma.auth_status;
        results_summary(k).status        = 'OK';
        results_summary(k).elapsed       = toc(t_start);

        fprintf(' done in %.1f s | pos_err=%.2f m | coasted=%d\n', ...
            results_summary(k).elapsed, results_summary(k).final_err, n_coasted);

    catch ME
        results_summary(k).scenario = s.name;
        results_summary(k).consts   = strjoin(s.spoofed_constellations, '+');
        results_summary(k).status   = 'FAILED';
        results_summary(k).error_msg = ME.message;
        results_summary(k).elapsed  = toc(t_start);
        fprintf(' FAILED: %s\n', ME.message);
    end
end

t_total = toc(t_batch_start);

%% ── SUMMARY TABLE ─────────────────────────────────────────────────────────
fprintf('\n');
fprintf('========================================\n');
fprintf('  Batch Complete (%.1f s total)\n', t_total);
fprintf('  RNP-0.1 reference scale: %.1f m (descriptive, not certified)\n', RNP01_ref);
fprintf('  HPL multiplier K_H=6.18 (indicative RAIM-style, not certified)\n');
fprintf('========================================\n\n');

% Header
fprintf('%-30s %-16s %9s %9s %9s %9s %9s %8s\n', ...
    'Scenario', 'Constellations', ...
    'Final(m)', 'Med(m)', 'p95(m)', ...
    'HPL_p95', 'HPL<185m%', 'Coasted');
fprintf('%s\n', repmat('-', 1, 100));

for k = 1:n_scenarios
    r = results_summary(k);
    if strcmp(r.status, 'OK')
        fprintf('%-30s %-16s %9.2f %9.2f %9.2f %9.2f %9.1f %8d\n', ...
            r.scenario, r.consts, ...
            r.final_err, r.median_err, r.p95_err, ...
            r.hpl_p95, r.hpl_pct, r.coasted);
    else
        fprintf('%-30s %-16s %9s %9s %9s %9s %9s %8s  FAILED\n', ...
            r.scenario, r.consts, '-','-','-','-','-','-');
    end
end

fprintf('%s\n', repmat('-', 1, 100));

% Save summary
summary_file = fullfile(cfg.paths.pvt, 'batch_summary.mat');
if ~isfolder(cfg.paths.pvt), mkdir(cfg.paths.pvt); end
save(summary_file, 'results_summary');
fprintf('\nBatch summary saved: %s\n', summary_file);