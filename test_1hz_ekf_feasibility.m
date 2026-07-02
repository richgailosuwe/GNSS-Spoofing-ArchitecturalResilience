function test_1hz_ekf_feasibility(obs_path)
% TEST_1HZ_EKF_FEASIBILITY  One-shot experiment: is a healthy 1 Hz EKF reachable?
%
%   test_1hz_ekf_feasibility()           % prompts for .obs
%   test_1hz_ekf_feasibility(obs_path)
%
% EXPERIMENTAL — does NOT touch run_real_authentic_validation.m or config.m.
% Sweeps a small set of process-noise multipliers at dt=1 s and reports, for
% each, the median accepted measurements and EKF-WLS separation. Purpose: learn
% in ONE run whether ANY simple Q setting restores filter health (accept rate
% comfortably above min_sats), which tells us if a full 1 Hz profile is worth
% building now or should be future work.
%
% Decision rule:
%   If some multiplier gives healthy acceptance AND EKF-WLS tail metrics remain
%   bounded
%       -> 1 Hz EKF is REACHABLE; building the profile is worthwhile.
%   If no multiplier helps (accept stays starved or EKF still diverges)
%       -> needs full calibration (R-from-residuals, covariance floor, Doppler);
%          scope as future work, finalise on the WLS result.
%

    run('config.m'); cfg.verbose = false;
    addpath(fullfile(cfg.root,'utils'));
    addpath(fullfile(cfg.root,'stage4_recovery'));

    if nargin < 1 || isempty(obs_path)
        [fn,fp] = uigetfile({'*.obs'}, 'Select hardware .obs', ...
            fullfile(cfg.root,'data','rinex','hardware'));
        if isequal(fn,0), fprintf('Cancelled.\n'); return; end
        obs_path = fullfile(fp,fn);
    end
    [fp,stem,~] = fileparts(obs_path);
    nav_path = fullfile(fp,[stem '.nav']);
    if ~isfile(nav_path)
        nd = dir(fullfile(fp,'*.nav'));
        if isempty(nd)
            error('No matching .nav file and no *.nav file found in: %s', fp);
        end
        nav_path = fullfile(fp,nd(1).name);
    end

    % Seed from header
    approx = parse_approx_position(obs_path);
    if ~isempty(approx), cfg.ref_pos = approx(:); end

    obs = rinex_read_obs(obs_path, cfg);
    nav = rinex_read_nav(nav_path, cfg);
    constellations = {'GPS','Galileo','BeiDou','GLONASS'};
    epochs = collect_epochs(obs, constellations);
    nep = numel(epochs);
    if nep < 2
        error('Need at least two epochs for 1 Hz feasibility test.');
    end
    dt_real = median(seconds(diff(epochs)));

    % WLS baseline (reference for separation) — reuse same model
    noise_map = struct('GPS',cfg.ekf.meas_noise_GPS,'Galileo',cfg.ekf.meas_noise_Galileo, ...
                       'BeiDou',cfg.ekf.meas_noise_BeiDou,'GLONASS',cfg.ekf.meas_noise_GLONASS);
    wls_pos = nan(nep,3);
    for ei=1:nep
        p = wls_epoch(obs,nav,epochs(ei),constellations,noise_map,cfg);
        if ~isempty(p), wls_pos(ei,:)=p(:)'; end
    end

    % Baseline Q values
    Qpos0=cfg.ekf.Q_pos; Qvel0=cfg.ekf.Q_vel; Qclk0=cfg.ekf.Q_clk; Qdr0=cfg.ekf.Q_clk_drift;
    has_isb = isfield(cfg.ekf,'Q_isb'); if has_isb, Qisb0=cfg.ekf.Q_isb; end

    fprintf('\n=====================================================\n');
    fprintf('  1 Hz EKF FEASIBILITY SWEEP  (dt = %.3f s)\n', dt_real);
    fprintf('  Baseline Q: pos=%.3g vel=%.3g clk=%.3g drift=%.3g\n', Qpos0,Qvel0,Qclk0,Qdr0);
    fprintf('=====================================================\n');
    fprintf('  Epochs: %d | nav: %s\n', nep, nav_path);
    fprintf('  Decision rule requires acceptance health AND bounded EKF-WLS tail.\n');
    fprintf('-----------------------------------------------------\n');
    fprintf('%-8s %6s %6s %6s %9s %9s %9s %9s %9s %-18s\n', ...
        'Q_mult','medAcc','p05Acc','minAcc','sepMed','sepP95','sepMax','sepEnd','HPLp95','verdict');

    % Sweep: multiply ALL Q terms by k (uniform empirical bump — the gate needs
    % bigger predicted covariance so normal 1 Hz innovations pass). Uniform (not
    % r^3) avoids the overconfidence trap we already hit.
    mults = [1, 10, 100, 1000, 10000];
    best = struct('k',NaN,'medAcc',-1,'p05Acc',-1,'sepMed',Inf, ...
                  'sepP95',Inf,'sepMax',Inf,'sepEnd',Inf,'hplP95',Inf, ...
                  'reachable',false);
    for k = mults
        cfg.ekf.dt = dt_real;
        cfg.ekf.Q_pos=Qpos0*k; cfg.ekf.Q_vel=Qvel0*k;
        cfg.ekf.Q_clk=Qclk0*k; cfg.ekf.Q_clk_drift=Qdr0*k;
        if has_isb, cfg.ekf.Q_isb=Qisb0*k; end

        ekf = ekf_runner(obs, nav, {}, cfg);
        acc = ekf.n_accepted(:);
        if numel(acc) > 1
            acc_eval = acc(2:end);  % epoch 1 is WLS bootstrap, not an EKF update
        else
            acc_eval = acc;
        end
        macc = median(acc_eval,'omitnan');
        p05acc = pct_valid(acc_eval,5);
        minacc = min(acc_eval);

        sep  = vecnorm(ekf.pos - wls_pos, 2, 2);
        msep = median(sep,'omitnan');
        p95sep = pct_valid(sep,95);
        maxsep = max(sep,[],'omitnan');
        last_valid = find(~isnan(sep),1,'last');
        if isempty(last_valid), endsep = NaN; else, endsep = sep(last_valid); end

        if isfield(ekf,'hpl') && any(~isnan(ekf.hpl))
            hplp95 = pct_valid(ekf.hpl,95);
        else
            hplp95 = NaN;
        end

        accept_ok = (macc >= 8) && (p05acc >= 5);
        sep_ok    = (msep < 20) && (p95sep < 50) && (maxsep < 100) && (endsep < 50);
        hpl_ok    = isnan(hplp95) || hplp95 < 185.2;
        reachable = accept_ok && sep_ok && hpl_ok;

        if reachable
            v = 'HEALTHY';
        elseif ~accept_ok
            v = 'starved';
        elseif ~sep_ok
            v = 'accepts-but-drifts';
        else
            v = 'HPL-too-large';
        end

        fprintf('%-8g %6.1f %6.1f %6.0f %9.1f %9.1f %9.1f %9.1f %9.1f %-18s\n', ...
            k, macc, p05acc, minacc, msep, p95sep, maxsep, endsep, hplp95, v);
        if reachable && p95sep<best.sepP95
            best=struct('k',k,'medAcc',macc,'p05Acc',p05acc,'sepMed',msep, ...
                        'sepP95',p95sep,'sepMax',maxsep,'sepEnd',endsep, ...
                        'hplP95',hplp95,'reachable',true);
        end
    end

    fprintf('-----------------------------------------------------\n');
    if best.reachable
        fprintf('VERDICT: 1 Hz EKF REACHABLE under this uniform-Q sweep.\n');
        fprintf('  Q_mult=%g | medAcc=%.1f p05Acc=%.1f | sep med/p95/max/end=%.1f/%.1f/%.1f/%.1f m | HPLp95=%.1f m\n', ...
            best.k, best.medAcc, best.p05Acc, best.sepMed, best.sepP95, ...
            best.sepMax, best.sepEnd, best.hplP95);
        fprintf('  -> Worth building the zedf9p_1hz profile around this point.\n');
        fprintf('  -> Still calibrate R from hardware residuals before thesis claims.\n');
    else
        fprintf('VERDICT: no simple uniform-Q multiplier restores health.\n');
        fprintf('  -> 1 Hz EKF needs full calibration (R-from-residuals, covariance\n');
        fprintf('     floor/reset, Doppler updates). Scope as FUTURE WORK; finalise\n');
        fprintf('     the hardware validation on the WLS result (already passing).\n');
    end
    fprintf('=====================================================\n');
end

%% ---- helpers (copies, self-contained) ----
function epochs = collect_epochs(obs, constellations)
    epochs = datetime.empty(0,1);
    for ci = 1:numel(constellations)
        c = constellations{ci};
        if isfield(obs,c) && isfield(obs.(c),'time') && ~isempty(obs.(c).time)
            epochs = [epochs; obs.(c).time(:)]; %#ok<AGROW>
        end
    end
    epochs = unique(epochs);
end

function p = pct_valid(x, q)
    x = x(:);
    x = x(~isnan(x));
    if isempty(x)
        p = NaN;
    else
        p = prctile(x, q);
    end
end

function approx = parse_approx_position(obs_path)
    approx=[]; fid=fopen(obs_path,'r'); if fid<0, return; end
    c=onCleanup(@() fclose(fid));
    while true
        line=fgetl(fid); if ~ischar(line), break; end
        if contains(line,'APPROX POSITION XYZ')
            n=sscanf(line(1:60),'%f'); if numel(n)>=3, approx=n(1:3); end; return;
        end
        if contains(line,'END OF HEADER'), return; end
    end
end

function pos = wls_epoch(obs,nav,t_e,constellations,noise_map,cfg)
    pos=[]; PR=[]; SP=[]; W=[];
    for ci=1:numel(constellations)
        c=constellations{ci}; if ~isfield(obs,c), continue; end
        m=(obs.(c).time==t_e); prns=obs.(c).prn(m); prs=obs.(c).pseudorange_L1(m);
        for k=1:numel(prns)
            if isnan(prs(k))||prs(k)<=0, continue; end
            [pr_corr, sp]=corrected_pseudorange(prs(k),prns(k),c,t_e,cfg.ref_pos,nav,cfg);
            if isempty(pr_corr)||any(isnan(sp))||isnan(pr_corr), continue; end
            PR(end+1,1)=pr_corr; SP(end+1,:)=sp(:)'; W(end+1,1)=1/noise_map.(c); %#ok<AGROW>
        end
    end
    if numel(PR)<4, return; end
    x=[cfg.ref_pos(:);0];
    for it=1:8
        rho=vecnorm(SP-x(1:3)',2,2); Hh=[-(SP-x(1:3)')./rho, ones(numel(PR),1)];
        dz=PR-(rho+x(4)); Wm=diag(W); dx=(Hh'*Wm*Hh)\(Hh'*Wm*dz); x=x+dx;
        if norm(dx(1:3))<1e-3, break; end
    end
    pos=x(1:3);
end
