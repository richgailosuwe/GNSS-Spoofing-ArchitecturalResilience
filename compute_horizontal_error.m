function compute_horizontal_error(scenario_name)
% COMPUTE_HORIZONTAL_ERROR  Derive horizontal (EN) position error from saved
% ECEF positions, for thesis Ch6. Read-only: loads saved .mat, no rerun.
% Uses horizontal_error.m, the shared EN-error helper used by Chapter 6 plots.
%
%   compute_horizontal_error('baseline')
%   compute_horizontal_error('scenario_1_gps')
% AUTHOR: RG

    if nargin < 1, scenario_name = 'baseline'; end
    config; cfg.verbose = false;

    if strcmp(scenario_name,'baseline')
        L = load(fullfile(cfg.paths.pvt,'baseline_authentic.mat'));
        ekf = L.baseline.ekf;
        ref = cfg.ref_pos(:)';
    else
        L = load(fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', scenario_name)));
        ekf = L.pipeline_result.ekf;
        ref = L.pipeline_result.ref_pos(:)';
    end
    pos = ekf.pos;                 % [n x 3] ECEF
    hpl = ekf.hpl(:);

    % 3D reference-relative error (what pos_error currently is)
    e3d = vecnorm(pos - ref, 2, 2);

    [lat, lon, ~] = ecef2lla_simple(ref);
    eHoriz = horizontal_error(pos, ref);

    v = ~isnan(e3d) & ~isnan(hpl);
    fprintf('\n=====================================================\n');
    fprintf('  HORIZONTAL vs 3D ERROR — %s\n', scenario_name);
    fprintf('  (ref lat %.4f deg, lon %.4f deg)\n', rad2deg(lat), rad2deg(lon));
    fprintf('=====================================================\n');
    fprintf('3D reference-relative error: final %.2f, med %.2f, p95 %.2f, max %.2f m\n', ...
        e3d(end), median(e3d,'omitnan'), prctile(e3d(~isnan(e3d)),95), max(e3d));
    fprintf('Horizontal EN error:         final %.2f, med %.2f, p95 %.2f, max %.2f m\n', ...
        eHoriz(end), median(eHoriz,'omitnan'), prctile(eHoriz(~isnan(eHoriz)),95), max(eHoriz));
    fprintf('-----------------------------------------------------\n');
    fprintf('Sanity: horizontal <= 3D every epoch? %s\n', ...
        string(all(eHoriz(~isnan(eHoriz)) <= e3d(~isnan(e3d)) + 1e-6)));
    fprintf('HPL >= HORIZONTAL error: %d / %d epochs (%.1f%%)\n', ...
        sum(hpl(v) >= eHoriz(v)), sum(v), 100*mean(hpl(v) >= eHoriz(v)));
    fprintf('Horizontal MI epochs (horiz err > HPL): %d\n', sum(eHoriz(v) > hpl(v)));
    fprintf('=====================================================\n');
end
