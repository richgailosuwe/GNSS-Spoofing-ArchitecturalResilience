function confirm_hpl_compliance(scenario_name)
% CONFIRM_HPL_COMPLIANCE  Read-only HPL analysis for thesis Section 6.5.
% Computes, from saved pipeline evidence (no rerun, no modification):
%   1. Fraction of epochs with HPL < 185.2 m (RNP-0.1 accuracy scale)
%   2. Fraction of epochs with HPL < 370.4 m (HAL reference, 2x RNP)
%   3. Whether the epochs that EXCEED 370.4 m coincide with the gate_only
%      fallback window (the "integrity layer signals unavailable" claim).
%
%   confirm_hpl_compliance('scenario_4_gps_glonass')
%   confirm_hpl_compliance('scenario_5_gps_galileo')
%   (run for all five to fill the HPL table)

    if nargin < 1, scenario_name = 'scenario_4_gps_glonass'; end
    config; cfg.verbose = false;

    P = load(fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', scenario_name)));
    P = P.pipeline_result;
    hpl = P.ekf.hpl(:);
    fb  = logical(P.ekf.exclusion_fallback(:));
    valid = ~isnan(hpl);
    n = sum(valid);

    RNP = 185.2;   % RNP-0.1 accuracy scale
    HAL = 370.4;   % HAL reference (2x RNP)

    below_rnp = sum(hpl(valid) < RNP);
    below_hal = sum(hpl(valid) < HAL);
    exceeds_hal = valid & (hpl >= HAL);   % epochs where HPL >= HAL

    fprintf('\n=====================================================\n');
    fprintf('  HPL COMPLIANCE — %s\n', scenario_name);
    fprintf('=====================================================\n');
    fprintf('Valid HPL epochs:        %d\n', n);
    fprintf('HPL median / p95 / max:  %.2f / %.2f / %.2f m\n', ...
        median(hpl(valid)), prctile(hpl(valid),95), max(hpl(valid)));
    fprintf('HPL < %.1f m (RNP-0.1):  %d / %d = %.1f%%\n', RNP, below_rnp, n, 100*below_rnp/n);
    fprintf('HPL < %.1f m (HAL ref):  %d / %d = %.1f%%\n', HAL, below_hal, n, 100*below_hal/n);
    fprintf('-----------------------------------------------------\n');

    % Coincidence check: do HPL>=HAL epochs fall within the fallback window?
    n_exceed = sum(exceeds_hal);
    fprintf('Epochs with HPL >= %.1f m: %d\n', HAL, n_exceed);
    if n_exceed > 0
        in_fb     = sum(exceeds_hal & fb);
        out_fb    = sum(exceeds_hal & ~fb);
        fprintf('  of those, IN fallback window:  %d (%.1f%%)\n', in_fb, 100*in_fb/n_exceed);
        fprintf('  of those, OUTSIDE fallback:    %d (%.1f%%)\n', out_fb, 100*out_fb/n_exceed);
        if out_fb == 0
            fprintf('  -> ALL HPL-exceedances occur during fallback. The "integrity layer\n');
            fprintf('     signals unavailable during fallback" claim is SUPPORTED.\n');
        elseif in_fb >= out_fb
            fprintf('  -> MOST HPL-exceedances occur during fallback (but not all). Soften the\n');
            fprintf('     claim to "predominantly during the fallback windows".\n');
        else
            fprintf('  -> HPL-exceedances are NOT mainly in fallback. Do NOT claim the\n');
            fprintf('     coincidence; report the percentages plainly instead.\n');
        end
    else
        fprintf('  No epochs exceed HAL — HPL stays under %.1f m for 100%% of epochs.\n', HAL);
    end
    fprintf('=====================================================\n');
end
