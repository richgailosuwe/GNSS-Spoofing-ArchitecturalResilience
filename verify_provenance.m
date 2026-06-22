function verify_provenance()
% VERIFY_PROVENANCE  Audit tool: confirm all saved evidence is post-fix and consistent.
%
%   verify_provenance()
%
% Loads every saved scenario pipeline .mat and ablation .mat under results/,
% and prints, per file:
%   - file name
%   - created timestamp
%   - code_notes (provenance tag)
%   - key cfg fields that define the result: inter_const_threshold, K_H,
%     insufficient_geometry_policy, use_innovation_gate, per-constellation
%     measurement noise sigmas.
%
% Purpose: close the audit concern "which code/config produced this file?"
% by making the evidence chain inspectable at a glance. A file missing
% config_snapshot/created is flagged as STALE (pre-provenance, must regenerate).
%
% Run from project root after regeneration (Task 4).
%
% PROJECT: GNSS Thesis MATLAB Implementation, UPB
% AUTHOR:  RG

    config;   % needs cfg.root only

    fprintf('\n=====================================================================\n');
    fprintf('  PROVENANCE AUDIT — %s\n', string(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
    fprintf('=====================================================================\n');

    pvt_dir = fullfile(cfg.root, 'results', 'pvt');
    abl_dir = fullfile(cfg.root, 'results', 'ablation');

    n_ok = 0; n_stale = 0;

    % ---- Scenario pipeline files ----
    fprintf('\n--- Scenario pipeline files (results/pvt/) ---\n');
    files = dir(fullfile(pvt_dir, '*_pipeline.mat'));
    if isempty(files)
        fprintf('  (none found)\n');
    end
    for k = 1:numel(files)
        fp = fullfile(pvt_dir, files(k).name);
        [ok, stale] = report_file(fp, files(k).name, 'pipeline_result');
        n_ok = n_ok + ok; n_stale = n_stale + stale;
    end

    % ---- batch_summary ----
    bs = fullfile(pvt_dir, 'batch_summary.mat');
    if isfile(bs)
        fprintf('\n--- batch_summary.mat ---\n');
        d = dir(bs);
        fprintf('  %-38s  file mtime: %s\n', 'batch_summary.mat', ...
            string(datetime(d.datenum,'ConvertFrom','datenum','Format','yyyy-MM-dd HH:mm')));
    end

    % ---- Ablation files ----
    fprintf('\n--- Ablation files (results/ablation/) ---\n');
    afiles = dir(fullfile(abl_dir, '*_ablation.mat'));
    if isempty(afiles)
        fprintf('  (none found — run_ablation has not been saved yet)\n');
    end
    for k = 1:numel(afiles)
        fp = fullfile(abl_dir, afiles(k).name);
        [ok, stale] = report_file(fp, afiles(k).name, 'results');
        n_ok = n_ok + ok; n_stale = n_stale + stale;
    end

    fprintf('\n=====================================================================\n');
    fprintf('  SUMMARY: %d file(s) with provenance, %d STALE (missing snapshot)\n', n_ok, n_stale);
    if n_stale > 0
        fprintf('  *** %d stale file(s) must be regenerated before Chapter 6 use ***\n', n_stale);
    else
        fprintf('  All evidence files carry provenance. Evidence chain is clean.\n');
    end
    fprintf('=====================================================================\n');
end

% ------------------------------------------------------------------------
function [ok, stale] = report_file(fp, name, varname)
    ok = 0; stale = 0;
    S = load(fp);
    if ~isfield(S, varname)
        fprintf('  %-38s  [no "%s" struct — unexpected format]\n', name, varname);
        stale = 1; return;
    end
    R = S.(varname);

    has_snap = isfield(R, 'config_snapshot');
    has_created = isfield(R, 'created');

    if ~has_snap || ~has_created
        fprintf('  %-38s  *** STALE: missing %s%s ***\n', name, ...
            tern(~has_snap,'config_snapshot ',''), tern(~has_created,'created',''));
        stale = 1; return;
    end

    c = R.config_snapshot;
    notes = '(none)'; if isfield(R,'code_notes'), notes = R.code_notes; end

    fprintf('  %-38s\n', name);
    fprintf('      created    : %s\n', string(R.created));
    fprintf('      code_notes : %s\n', notes);
    fprintf('      -> %s\n', longform(notes));

    % Key cfg fields (guarded — print only if present)
    print_field(c, {'identify','inter_const_threshold'}, 'inter_const_threshold', '%.1f m');
    print_field(c, {'integrity','K_H'},                  'K_H (HPL)',             '%.3f');
    print_field(c, {'stage3','insufficient_geometry_policy'}, 'ig_policy',        '%s');
    print_field(c, {'stage3','use_innovation_gate'},     'use_gate',              '%d');
    print_field(c, {'ekf','meas_noise_GPS'},     'sigma2 GPS',     '%.1f');
    print_field(c, {'ekf','meas_noise_Galileo'}, 'sigma2 Galileo', '%.1f');
    print_field(c, {'ekf','meas_noise_BeiDou'},  'sigma2 BeiDou',  '%.1f');
    print_field(c, {'ekf','meas_noise_GLONASS'}, 'sigma2 GLONASS', '%.1f');
    ok = 1;
end

function print_field(c, path, label, fmt)
    v = c;
    for i = 1:numel(path)
        if isstruct(v) && isfield(v, path{i})
            v = v.(path{i});
        else
            return;   % field absent — skip silently
        end
    end
    fprintf(['      %-22s : ' fmt '\n'], label, v);
end

function s = longform(notes)
    % Human-readable expansion of the terse internal tag.
    if contains(notes,'#11') && contains(notes,'#12')
        s = 'Post Galileo BGD band-selection fix and insufficient-geometry fallback';
    elseif contains(notes,'#11')
        s = 'Post Galileo BGD band-selection fix';
    elseif contains(notes,'#12')
        s = 'Post insufficient-geometry fallback';
    else
        s = notes;
    end
end

function s = tern(c,a,b); if c, s=a; else, s=b; end; end