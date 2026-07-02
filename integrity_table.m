function integrity_table()
% INTEGRITY_TABLE  Read-only pull of the exact numbers for thesis 6.2 + 6.5.
% For baseline + all 5 scenarios: HPL median/p95/max, coasting count, and
% count (epochs where HORIZONTAL EN error > HPL — should be 0). No rerun.

    config; cfg.verbose = false;
    ref = cfg.ref_pos(:)';

    fprintf('\n=====================================================================\n');
    fprintf('  INTEGRITY TABLE (read-only)  — EN error vs HPL\n');
    fprintf('=====================================================================\n');
    fprintf('%-22s %8s %8s %8s %7s %8s %4s\n', ...
        'case','HPLmed','HPLp95','HPLmax','coast','HPLexc','n');
    fprintf('---------------------------------------------------------------------\n');

    % --- baseline ---
    L = load(fullfile(cfg.paths.pvt,'baseline_authentic.mat'));
    ekf = L.baseline.ekf;
    row('baseline', ekf, ref);

    % --- 5 scenarios ---
    for i = 1:numel(cfg.scenarios)
        name = cfg.scenarios{i}.name;
        fp = fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', name));
        if ~isfile(fp), continue; end
        S = load(fp); ekf = S.pipeline_result.ekf;
        row(name, ekf, ref);
    end
    fprintf('=====================================================================\n');
    fprintf(['HPLexc = empirical HPL exceedance epochs where horizontal EN ' ...
             'error > HPL (should be 0).\n']);
end

function row(label, ekf, ref)
    hpl = ekf.hpl(:);
    eH  = horizontal_error(ekf.pos, ref);
    v = ~isnan(hpl) & ~isnan(eH);
    hpl_exceed = sum(eH(v) > hpl(v));
    coast = sum(ekf.coasted);
    fprintf('%-22s %8.2f %8.2f %8.2f %7d %8d %4d\n', ...
        label, median(hpl(v)), prctile(hpl(v),95), max(hpl(v)), coast, ...
        hpl_exceed, sum(v));
end
