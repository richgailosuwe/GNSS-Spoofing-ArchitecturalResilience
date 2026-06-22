function check_sessionB()
% Read-only: find session B's WLS scatter (3D + horizontal) from whatever
% saved file actually contains its wls_pos. Checks the crossval files.
    config; cfg.verbose = false;
    hw = fullfile(cfg.root,'results','hardware');
    cands = { 'june18footballpitchdomnesti_crossval_q10000.mat';
              'june18footballpitchdomnesti_crossval_q1000.mat' };
    for i = 1:numel(cands)
        fp = fullfile(hw, cands{i});
        if ~isfile(fp), continue; end
        L = load(fp); f = fieldnames(L); X = L.(f{1});
        fprintf('\n=== %s ===\n', cands{i});
        fprintf('top field: %s, subfields:\n', f{1}); disp(fieldnames(X));
        % Look for a wls position field
        wlsfield = '';
        for nm = {'wls_pos','wlsPos','wls'}
            if isfield(X, nm{1}), wlsfield = nm{1}; break; end
        end
        if isempty(wlsfield)
            fprintf('  no wls_pos-like field here.\n'); continue;
        end
        wls = X.(wlsfield);
        v = all(~isnan(wls),2); wls = wls(v,:); mu = mean(wls,1);
        s3d = vecnorm(wls - mu, 2, 2);
        [lat,lon,~] = ecef2lla_simple(mu);
        R = [ -sin(lon), cos(lon), 0;
              -sin(lat)*cos(lon), -sin(lat)*sin(lon), cos(lat);
               cos(lat)*cos(lon),  cos(lat)*sin(lon), sin(lat) ];
        enu = (R*(wls-mu)')'; sEN = vecnorm(enu(:,1:2),2,2);
        fprintf('  Session B WLS scatter: 3D med %.2f p95 %.2f | EN med %.2f p95 %.2f m\n', ...
            median(s3d), prctile(s3d,95), median(sEN), prctile(sEN,95));
        return;  % first one with wls_pos is enough
    end
end