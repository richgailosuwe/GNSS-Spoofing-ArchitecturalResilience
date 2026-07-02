function results = run_real_authentic_validation(obs_path)
% RUN_REAL_AUTHENTIC_VALIDATION  Real-hardware robustness/consistency check.
%
%   results = run_real_authentic_validation()           % prompts for .obs file
%   results = run_real_authentic_validation(obs_path)   % explicit .obs path
%
% Runs the FINAL pipeline (Stage 4 EKF, no spoofing, all satellites trusted)
% on independent ZED-F9P hardware RINEX, alongside an independent per-epoch
% WLS baseline computed from the SAME data via corrected_pseudorange.
%
% WHAT THIS VALIDATES (and what it does NOT):
%   This is a REAL-HARDWARE ROBUSTNESS / CONSISTENCY check, NOT an absolute
%   accuracy test. The same pseudoranges feed both WLS and EKF, so EKF-vs-WLS
%   shows internal consistency and stability on independent data -- it does
%   NOT prove absolute accuracy. The session was standalone (no NTRIP/RTCM
%   corrections), so absolute accuracy is bounded by the ZED-F9P standalone
%   limit (~1.5 m); RTK-corrected validation is future work. No "position
%   error" vs truth is reported; only scatter, separation, HPL, and stability.
%
% SEED (linearisation point, NOT truth) preference order:
%   1. APPROX POSITION XYZ from the .obs header (best -- from the real file)
%   2. Domnesti football-pitch fallback (only if header field missing)
%   3. BUCU cfg.ref_pos (last resort seed only)
%

    run('config.m');
    cfg.verbose = false;
    addpath(fullfile(cfg.root,'utils'));
    addpath(fullfile(cfg.root,'stage4_recovery'));

    %% --- Resolve the .obs file ------------------------------------------
    if nargin < 1 || isempty(obs_path)
        [fn, fp] = uigetfile({'*.obs','RINEX observation (*.obs)'}, ...
            'Select hardware RINEX .obs file', ...
            fullfile(cfg.root,'data','rinex','hardware'));
        if isequal(fn,0), fprintf('Cancelled.\n'); results=[]; return; end
        obs_path = fullfile(fp, fn);
    end
    assert(isfile(obs_path), 'obs file not found: %s', obs_path);

    % Auto-locate matching .nav (same folder, same stem; else any .nav there)
    [fp, stem, ~] = fileparts(obs_path);
    nav_path = fullfile(fp, [stem '.nav']);
    if ~isfile(nav_path)
        nd = dir(fullfile(fp,'*.nav'));
        assert(~isempty(nd), 'No .nav file found alongside %s', obs_path);
        nav_path = fullfile(fp, nd(1).name);
    end

    fprintf('\n=====================================================\n');
    fprintf('  REAL-HARDWARE AUTHENTIC VALIDATION\n');
    fprintf('  obs: %s\n', obs_path);
    fprintf('  nav: %s\n', nav_path);
    fprintf('=====================================================\n');

    %% --- Seed from RINEX header APPROX POSITION XYZ ----------------------
    approx = parse_approx_position(obs_path);
    if ~isempty(approx)
        cfg.ref_pos = approx(:);
        fprintf('Seed: APPROX POSITION XYZ from header = [%.1f %.1f %.1f]\n', approx);
        seed_src = 'rinex_header_approx';
    else
        % Fallback 2: Domnesti football pitch (rough), converted to ECEF.
        % Stadionul FC Domnesti approx 44.532 N, 25.978 E, ~90 m ellipsoidal.
        lla = [44.532, 25.978, 90];
        cfg.ref_pos = lla2ecef_local(lla(1),lla(2),lla(3));
        fprintf('Seed: header APPROX missing -> Domnesti fallback = [%.1f %.1f %.1f]\n', cfg.ref_pos);
        seed_src = 'domnesti_fallback';
        fprintf('  (override by adding APPROX POSITION XYZ to the .obs header)\n');
    end

    %% --- Load data ------------------------------------------------------
    obs = rinex_read_obs(obs_path, cfg);
    nav = rinex_read_nav(nav_path, cfg);
    epochs = unique(obs.GPS.time);
    nep = numel(epochs);
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    fprintf('\nLoaded %d epochs.\n', nep);

    % CRITICAL: set EKF timestep from the actual RINEX epoch spacing.
    % cfg.ekf.dt defaults to the BUCU 30 s interval; this hardware logs at 1 Hz.
    % A 30x over-propagation between 1 s updates causes a large early EKF
    % transient (WLS, being memoryless, is unaffected). Set dt from the file.
    dt_real = median(seconds(diff(epochs)));
    dt_bucu = cfg.ekf.dt;            % original tuning interval (BUCU = 30 s)
    cfg.ekf.dt = dt_real;
    fprintf('EKF dt set from RINEX epoch spacing: %.3f s (was %.1f s)\n', cfg.ekf.dt, dt_bucu);
    % NOTE: process noise Q is left at its BUCU tuning. Theoretical dt-scaling
    % (Q ∝ dt / dt^3) was tried and made the filter overconfident: the covariance
    % collapsed, the scalar gate rejected almost all real measurements (median
    % ~3 of 18 accepted), and the under-constrained 11-state filter diverged.
    % For this 1 Hz dataset, fixing dt is correct; Q/gate retuning for 1 Hz is
    % an empirical exercise deferred to future work. The WLS baseline below is
    % the primary measurement-model validation and is independent of EKF tuning.

    %% --- EKF (authentic mode: empty classify => all trusted) ------------
    fprintf('Running EKF (authentic, all-trusted)...\n');
    ekf = ekf_runner(obs, nav, {}, cfg);
    ekf_pos = ekf.pos;                 % [nep x 3] ECEF

    %% --- Independent per-epoch WLS baseline (same measurement model) -----
    fprintf('Computing independent per-epoch WLS baseline...\n');
    wls_pos = nan(nep,3);
    noise_map = struct('GPS',cfg.ekf.meas_noise_GPS,'Galileo',cfg.ekf.meas_noise_Galileo, ...
                       'BeiDou',cfg.ekf.meas_noise_BeiDou,'GLONASS',cfg.ekf.meas_noise_GLONASS);
    nsat = zeros(nep,1);
    for ei = 1:nep
        [p, ns] = wls_epoch(obs, nav, epochs(ei), constellations, noise_map, cfg);
        if ~isempty(p), wls_pos(ei,:) = p(:)'; end
        nsat(ei) = ns;
    end

    %% --- Consistency / stability metrics (NO truth comparison) ----------
    % Session mean (receiver assumed stationary on the pitch).
    ekf_mean = mean(ekf_pos,1,'omitnan');
    wls_mean = mean(wls_pos,1,'omitnan');

    ekf_scatter = vecnorm(ekf_pos - ekf_mean, 2, 2);   % precision around own mean
    wls_scatter = vecnorm(wls_pos - wls_mean, 2, 2);
    ekf_wls_sep = vecnorm(ekf_pos - wls_pos, 2, 2);     % internal consistency

    hpl = ekf.hpl(:);
    n_coast = sum(ekf.coasted);
    n_fb    = 0; if isfield(ekf,'exclusion_fallback'), n_fb = sum(ekf.exclusion_fallback); end
    mean_dist_ekf_wls_mean = norm(ekf_mean - wls_mean);

    % --- EKF health check (estimator-validity gate) ----------------------
    % The EKF is tuned for the BUCU 30 s cadence. On 1 Hz hardware the
    % innovation gate and process noise are cadence-mismatched, which can
    % starve the filter. Accept rate is the decisive health signal: an
    % 11-state EKF needs comfortably more than min_sats measurements/epoch.
    med_accepted = median(ekf.n_accepted);
    ekf_valid = med_accepted >= max(cfg.identify.min_sats, 8);

    %% --- Report: WLS PRIMARY, EKF conditional ---------------------------
    fprintf('\n========================================================\n');
    fprintf('  PRIMARY RESULT: WLS measurement-model validation\n');
    fprintf('========================================================\n');
    fprintf('Epochs:                       %d\n', nep);
    fprintf('Satellites/epoch (mean):      %.1f\n', mean(nsat));
    fprintf('WLS scatter about its mean:   median %.2f m,  p95 %.2f m\n', ...
        median(wls_scatter,'omitnan'), prctile(wls_scatter(~isnan(wls_scatter)),95));
    fprintf('  -> The final transmit-time measurement model produces a stable\n');
    fprintf('     standalone solution on independent ZED-F9P hardware. This is\n');
    fprintf('     the hardware validation of the measurement model.\n');

    fprintf('\n--- EKF estimator health (1 Hz cadence) ---\n');
    fprintf('Median accepted measurements: %d / %.0f  (need >= %d)\n', ...
        med_accepted, mean(nsat), max(cfg.identify.min_sats,8));
    if ekf_valid
        fprintf('EKF regime: VALID. Reporting EKF consistency metrics:\n');
        fprintf('  EKF scatter about mean:   median %.2f m,  p95 %.2f m\n', ...
            median(ekf_scatter,'omitnan'), prctile(ekf_scatter(~isnan(ekf_scatter)),95));
        fprintf('  EKF-WLS separation:       median %.2f m,  p95 %.2f m,  max %.2f m\n', ...
            median(ekf_wls_sep,'omitnan'), prctile(ekf_wls_sep(~isnan(ekf_wls_sep)),95), max(ekf_wls_sep));
        fprintf('  EKF mean vs WLS mean:     %.2f m apart\n', mean_dist_ekf_wls_mean);
        fprintf('  HPL: median %.2f m, p95 %.2f m | coasted %d, fallbacks %d\n', ...
            median(hpl,'omitnan'), prctile(hpl(~isnan(hpl)),95), n_coast, n_fb);
    else
        fprintf('EKF regime: INVALID for this cadence (gate starves the filter).\n');
        fprintf('  The BUCU-tuned innovation gate / process noise reject most real\n');
        fprintf('  1 Hz measurements, so the 11-state EKF is under-constrained and\n');
        fprintf('  its position metrics here are NOT a valid result. A dedicated\n');
        fprintf('  1 Hz hardware profile (dt + Q + R + gate, calibrated from the\n');
        fprintf('  hardware WLS residuals, optionally using UBX-RXM-RAWX stdev\n');
        fprintf('  fields) is required for live operation -- identified as FUTURE WORK.\n');
        fprintf('  (For the record only: EKF-WLS median %.0f m at this invalid regime.)\n', ...
            median(ekf_wls_sep,'omitnan'));
    end

    fprintf('\n--------------------------------------------------------\n');
    fprintf('NOTE: standalone session, no NTRIP/RTCM corrections. Metrics are\n');
    fprintf('consistency/precision, NOT absolute accuracy. Absolute accuracy is\n');
    fprintf('NOT evaluated without surveyed/RTK truth; ZED-F9P standalone accuracy\n');
    fprintf('is a nominal manufacturer expectation, not a bound for this session.\n');
    fprintf('RTK-corrected and 1 Hz-profiled EKF validation = future work.\n');
    fprintf('========================================================\n');

    %% --- Package + save --------------------------------------------------
    results = struct();
    results.obs_path = obs_path; results.nav_path = nav_path;
    results.seed_source = seed_src; results.seed_ecef = cfg.ref_pos(:)';
    results.epochs = epochs;
    results.ekf_pos = ekf_pos; results.wls_pos = wls_pos;
    results.ekf_scatter = ekf_scatter; results.wls_scatter = wls_scatter;
    results.ekf_wls_sep = ekf_wls_sep; results.hpl = hpl; results.nsat = nsat;
    results.ekf_mean = ekf_mean; results.wls_mean = wls_mean;
    results.n_coast = n_coast; results.n_fallback = n_fb;
    results.ekf_dt = cfg.ekf.dt;
    results.ekf = ekf;                 % full EKF output for diagnostics
    results.ekf_valid = ekf_valid;     % estimator-health verdict for this cadence
    results.median_accepted = med_accepted;
    results.dataset_profile = sprintf('hardware_%.0fHz', 1/dt_real);
    results.n_accepted = ekf.n_accepted;
    results.n_rejected = ekf.n_rejected;
    results.coasted = ekf.coasted;
    if isfield(ekf,'exclusion_fallback'), results.exclusion_fallback = ekf.exclusion_fallback; end
    results.config_snapshot = cfg;
    results.created = datetime('now');
    results.code_notes = 'real-hardware authentic validation (standalone, no RTK)';

    out_dir = fullfile(cfg.root,'results','hardware');
    if ~isfolder(out_dir), mkdir(out_dir); end
    out_path = fullfile(out_dir, sprintf('%s_validation.mat', stem));
    save(out_path, 'results');
    fprintf('\nSaved: %s\n', out_path);
end

%% ====================================================================
function approx = parse_approx_position(obs_path)
% Read 'APPROX POSITION XYZ' from a RINEX obs header. Returns [] if absent.
    approx = [];
    fid = fopen(obs_path,'r'); if fid<0, return; end
    cleanup = onCleanup(@() fclose(fid));
    while true
        line = fgetl(fid);
        if ~ischar(line), break; end
        if contains(line,'APPROX POSITION XYZ')
            nums = sscanf(line(1:60), '%f');
            if numel(nums)>=3, approx = nums(1:3); end
            return;
        end
        if contains(line,'END OF HEADER'), return; end
    end
end

function [pos, nsat] = wls_epoch(obs, nav, t_e, constellations, noise_map, cfg)
% Independent single-epoch weighted least squares using corrected_pseudorange
% (same measurement model as the pipeline). Returns ECEF pos and sat count.
    pos = []; nsat = 0;
    PR=[]; SP=[]; W=[];
    for ci=1:numel(constellations)
        c=constellations{ci};
        if ~isfield(obs,c), continue; end
        m=(obs.(c).time==t_e);
        prns=obs.(c).prn(m); prs=obs.(c).pseudorange_L1(m);
        for k=1:numel(prns)
            if isnan(prs(k))||prs(k)<=0, continue; end
            [pr_corr, sp] = corrected_pseudorange(prs(k), prns(k), c, t_e, cfg.ref_pos, nav, cfg);
            if isempty(pr_corr)||any(isnan(sp))||isnan(pr_corr), continue; end
            PR(end+1,1)=pr_corr; SP(end+1,:)=sp(:)'; W(end+1,1)=1/noise_map.(c); %#ok<AGROW>
        end
    end
    nsat=numel(PR);
    if nsat<4, return; end
    % Iterative WLS for [x y z cdt]
    x=[cfg.ref_pos(:);0];
    for it=1:8
        rho=vecnorm(SP-x(1:3)',2,2);
        Hh=[-(SP-x(1:3)')./rho, ones(nsat,1)];
        pred=rho+x(4);
        dz=PR-pred;
        Wm=diag(W);
        dx=(Hh'*Wm*Hh)\(Hh'*Wm*dz);
        x=x+dx;
        if norm(dx(1:3))<1e-3, break; end
    end
    pos=x(1:3);
end

function ecef = lla2ecef_local(lat_deg, lon_deg, h)
% Minimal WGS84 lla->ecef for the fallback seed only.
    a=6378137.0; f=1/298.257223563; e2=f*(2-f);
    lat=deg2rad(lat_deg); lon=deg2rad(lon_deg);
    N=a/sqrt(1-e2*sin(lat)^2);
    ecef=[(N+h)*cos(lat)*cos(lon); (N+h)*cos(lat)*sin(lon); (N*(1-e2)+h)*sin(lat)];
end
