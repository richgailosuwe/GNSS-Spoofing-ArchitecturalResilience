function confirm_hardware_scatter()
% CONFIRM_HARDWARE_SCATTER  Read-only: compute WLS scatter about session mean
% for all three hardware sessions, BOTH 3D and horizontal EN, so the thesis
% cites them on a consistent, correctly-labelled metric. No rerun.
% AUTHOR: RG

    config; cfg.verbose = false;
    hw = fullfile(cfg.root,'results','hardware');
    files = { 'june18footballpitchdomnesti2_validation.mat', 'Football pitch A (domnesti2)';
              'june18footballpitchdomnesti_validation.mat',  'Football pitch B (domnesti)';
              'june21rooftop_validation.mat',                'Rooftop (june21)' };

    fprintf('\n=====================================================================\n');
    fprintf('  HARDWARE WLS SCATTER about session mean — 3D vs horizontal EN\n');
    fprintf('=====================================================================\n');
    fprintf('%-26s %8s %8s | %8s %8s\n','session','3D med','3D p95','EN med','EN p95');
    fprintf('---------------------------------------------------------------------\n');

    for i = 1:size(files,1)
        fp = fullfile(hw, files{i,1});
        if ~isfile(fp)
            fprintf('%-26s   (file not found: %s)\n', files{i,2}, files{i,1});
            continue;
        end
        L = load(fp);
        f = fieldnames(L); X = L.(f{1});
        if ~isfield(X,'wls_pos')
            fprintf('%-26s   (no wls_pos field)\n', files{i,2});
            continue;
        end
        wls = X.wls_pos;
        v = all(~isnan(wls),2);
        wls = wls(v,:);
        mu  = mean(wls,1);

        % 3D scatter about mean
        s3d = vecnorm(wls - mu, 2, 2);

        % Horizontal EN scatter about mean (rotate to ENU at the mean)
        [lat, lon, ~] = ecef2lla_simple(mu);
        R = [ -sin(lon),            cos(lon),           0;
              -sin(lat)*cos(lon),  -sin(lat)*sin(lon),  cos(lat);
               cos(lat)*cos(lon),   cos(lat)*sin(lon),  sin(lat) ];
        enu = (R * (wls - mu)')';
        sEN = vecnorm(enu(:,1:2), 2, 2);

        fprintf('%-26s %8.2f %8.2f | %8.2f %8.2f\n', files{i,2}, ...
            median(s3d), prctile(s3d,95), median(sEN), prctile(sEN,95));
    end
    fprintf('=====================================================================\n');
    fprintf('Cite ONE metric consistently. EN columns = horizontal (label "horizontal").\n');
    fprintf('3D columns must be labelled "3D scatter", never "horizontal".\n');
end