function validate_1hz_profile(q_mult, obs_path)
% VALIDATE_1HZ_PROFILE  Run a LOCKED 1 Hz EKF profile, unchanged, on a held-out
% session. This is the cross-session validation step: the Q multiplier is chosen
% on session 'domnesti2' and frozen; running it healthy on the independent
% 'domnesti' session is genuine validation, not tuning.
%
%   validate_1hz_profile(1000)              % prompts for the held-out .obs
%   validate_1hz_profile(1000, obs_path)
%
% q_mult : the FROZEN process-noise multiplier selected on the tuning session.
%
% EXPERIMENTAL — does not modify config.m or the validated pipeline.
% AUTHOR: RG (GNSS thesis)

    if nargin < 1 || isempty(q_mult)
        error('Provide the locked Q multiplier, e.g. validate_1hz_profile(1000).');
    end
    run('config.m'); cfg.verbose=false;
    addpath(fullfile(cfg.root,'utils')); addpath(fullfile(cfg.root,'stage4_recovery'));

    if nargin < 2 || isempty(obs_path)
        [fn,fp]=uigetfile({'*.obs'},'Select HELD-OUT session .obs (validation)', ...
            fullfile(cfg.root,'data','rinex','hardware'));
        if isequal(fn,0), return; end
        obs_path=fullfile(fp,fn);
    end
    [fp,stem,~]=fileparts(obs_path);
    nav_path=fullfile(fp,[stem '.nav']);
    if ~isfile(nav_path), nd=dir(fullfile(fp,'*.nav')); nav_path=fullfile(fp,nd(1).name); end

    approx=parse_approx_position(obs_path);
    if ~isempty(approx), cfg.ref_pos=approx(:); end

    obs=rinex_read_obs(obs_path,cfg); nav=rinex_read_nav(nav_path,cfg);
    epochs=unique(obs.GPS.time); nep=numel(epochs);
    constellations={'GPS','Galileo','BeiDou','GLONASS'};
    dt_real=median(seconds(diff(epochs)));

    % Apply the FROZEN profile: dt from file + locked Q multiplier
    cfg.ekf.dt=dt_real;
    cfg.ekf.Q_pos=cfg.ekf.Q_pos*q_mult;  cfg.ekf.Q_vel=cfg.ekf.Q_vel*q_mult;
    cfg.ekf.Q_clk=cfg.ekf.Q_clk*q_mult;  cfg.ekf.Q_clk_drift=cfg.ekf.Q_clk_drift*q_mult;
    if isfield(cfg.ekf,'Q_isb'), cfg.ekf.Q_isb=cfg.ekf.Q_isb*q_mult; end

    % WLS reference + EKF
    noise_map=struct('GPS',cfg.ekf.meas_noise_GPS,'Galileo',cfg.ekf.meas_noise_Galileo, ...
                     'BeiDou',cfg.ekf.meas_noise_BeiDou,'GLONASS',cfg.ekf.meas_noise_GLONASS);
    wls_pos=nan(nep,3);
    for ei=1:nep
        p=wls_epoch(obs,nav,epochs(ei),constellations,noise_map,cfg);
        if ~isempty(p), wls_pos(ei,:)=p(:)'; end
    end
    e=ekf_runner(obs,nav,{},cfg);
    sep=vecnorm(e.pos-wls_pos,2,2);
    macc=median(e.n_accepted(2:end));
    wls_scat=vecnorm(wls_pos-mean(wls_pos,1,'omitnan'),2,2);

    % Health verdict on held-out data
    healthy = (macc>=8) && (prctile(sep(~isnan(sep)),95) < 30) && ...
              (median(sep,'omitnan') < 10);

    fprintf('\n=====================================================\n');
    fprintf('  CROSS-SESSION VALIDATION (held-out: %s)\n', stem);
    fprintf('  Frozen profile: dt=%.2fs, Q_mult=%g (selected on tuning session)\n', dt_real, q_mult);
    fprintf('=====================================================\n');
    fprintf('Epochs:                  %d\n', nep);
    fprintf('WLS scatter:             median %.2f m, p95 %.2f m\n', ...
        median(wls_scat,'omitnan'), prctile(wls_scat(~isnan(wls_scat)),95));
    fprintf('Median accepted:         %d / %.0f  (need >= 8)\n', macc, mean(arrayfun(@(t) sum(obs.GPS.time==t),epochs)));
    fprintf('EKF-WLS separation:      median %.2f m, p95 %.2f m, max %.2f m\n', ...
        median(sep,'omitnan'), prctile(sep(~isnan(sep)),95), max(sep));
    fprintf('HPL:                     median %.2f m, p95 %.2f m\n', ...
        median(e.hpl,'omitnan'), prctile(e.hpl(~isnan(e.hpl)),95));
    fprintf('-----------------------------------------------------\n');
    if healthy
        fprintf('  [VALIDATED] The frozen 1 Hz profile stays HEALTHY on the\n');
        fprintf('  independent session it was NOT tuned on. Accept rate %d,\n', macc);
        fprintf('  EKF-WLS median %.2f m. This is genuine cross-session validation.\n', median(sep,'omitnan'));
    else
        fprintf('  [NOT VALIDATED] The frozen profile does not stay healthy on the\n');
        fprintf('  held-out session (accept %d, EKF-WLS median %.1f m). The profile\n', macc, median(sep,'omitnan'));
        fprintf('  may be overfit to the tuning session; report feasibility only.\n');
    end
    fprintf('=====================================================\n');

    out=struct('stem',stem,'q_mult',q_mult,'dt',dt_real,'healthy',healthy, ...
        'median_accepted',macc,'sep_median',median(sep,'omitnan'), ...
        'sep_p95',prctile(sep(~isnan(sep)),95),'wls_scatter_median',median(wls_scat,'omitnan'), ...
        'created',datetime('now'));
    od=fullfile(cfg.root,'results','hardware'); if ~isfolder(od), mkdir(od); end
    save(fullfile(od,sprintf('%s_crossval_q%g.mat',stem,q_mult)),'out');
    fprintf('Saved: %s_crossval_q%g.mat\n', stem, q_mult);
end

%% helpers
function approx=parse_approx_position(p)
    approx=[]; fid=fopen(p,'r'); if fid<0, return; end; c=onCleanup(@() fclose(fid));
    while true, l=fgetl(fid); if ~ischar(l), break; end
        if contains(l,'APPROX POSITION XYZ'), n=sscanf(l(1:60),'%f'); if numel(n)>=3, approx=n(1:3); end; return; end
        if contains(l,'END OF HEADER'), return; end
    end
end
function pos=wls_epoch(obs,nav,t_e,cons,nm,cfg)
    pos=[]; PR=[]; SP=[]; W=[];
    for ci=1:numel(cons), c=cons{ci}; if ~isfield(obs,c), continue; end
        m=(obs.(c).time==t_e); pr=obs.(c).pseudorange_L1(m); pn=obs.(c).prn(m);
        for k=1:numel(pn)
            if isnan(pr(k))||pr(k)<=0, continue; end
            [pc,sp]=corrected_pseudorange(pr(k),pn(k),c,t_e,cfg.ref_pos,nav,cfg);
            if isempty(pc)||any(isnan(sp))||isnan(pc), continue; end
            PR(end+1,1)=pc; SP(end+1,:)=sp(:)'; W(end+1,1)=1/nm.(c); %#ok<AGROW>
        end
    end
    if numel(PR)<4, return; end
    x=[cfg.ref_pos(:);0];
    for it=1:8
        rho=vecnorm(SP-x(1:3)',2,2); H=[-(SP-x(1:3)')./rho,ones(numel(PR),1)];
        dx=(H'*diag(W)*H)\(H'*diag(W)*(PR-(rho+x(4)))); x=x+dx;
        if norm(dx(1:3))<1e-3, break; end
    end
    pos=x(1:3);
end