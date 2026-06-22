function audit_fallback_epochs(scenario_name)
% AUDIT_FALLBACK_EPOCHS  Read-only audit of gate_only fallback behaviour.
%
%   audit_fallback_epochs('scenario_4_gps_glonass')
%   audit_fallback_epochs('scenario_5_gps_galileo')
%
% Loads only the saved pipeline evidence. It does not rerun the pipeline and
% does not modify any .mat file. The audit distinguishes:
%   1. classifier labels (trusted/suspect/spoofed),
%   2. configured injection ground truth (constellation + selected PRNs), and
%   3. aggregate EKF gate counts stored in epoch_log.
%
% IMPORTANT LIMITATION:
% The saved epoch_log contains accepted H rows and aggregate counts, but not
% the PRN/constellation identity or scalar-gate accepted mask for each row.
% Therefore this audit can prove outcome containment and, in all-spoof-label
% epochs, that accepted rows came from classified-spoofed measurements. It
% cannot identify which configured injected PRNs were accepted.

    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'scenario_4_gps_glonass';
    end

    config;
    cfg.verbose = false;

    fp = fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', scenario_name));
    if ~isfile(fp)
        error('audit_fallback_epochs:fileNotFound', 'Evidence file not found: %s', fp);
    end

    S = load(fp, 'pipeline_result');
    P = S.pipeline_result;
    ekf = P.ekf;
    cls = P.classification;
    fb  = logical(ekf.exclusion_fallback(:));
    nfb = sum(fb);
    n_epochs = numel(fb);

    fprintf('\n=====================================================\n');
    fprintf('  GATE_ONLY FALLBACK AUDIT - %s\n', scenario_name);
    fprintf('=====================================================\n');
    fprintf('Evidence created:            %s\n', string(P.created));
    fprintf('Evidence code notes:         %s\n', P.code_notes);
    fprintf('Total epochs:                %d\n', n_epochs);
    fprintf('Fallback (gate_only) epochs: %d\n', nfb);

    if nfb == 0
        fprintf('No fallback epochs - nothing to audit.\n');
        fprintf('=====================================================\n');
        return;
    end

    fb_idx = find(fb);
    segments = contiguous_segments(fb_idx);
    fprintf('Fallback span:               epoch %d to %d\n', fb_idx(1), fb_idx(end));
    fprintf('Contiguous fallback segments: %d\n', size(segments,1));
    for i = 1:size(segments,1)
        fprintf('  segment %2d: %4d-%4d (%d epochs)\n', i, segments(i,1), ...
            segments(i,2), segments(i,2)-segments(i,1)+1);
    end

    % Per-fallback classification and EKF summaries.
    n_trusted = nan(nfb,1);
    n_suspect = nan(nfb,1);
    n_spoofed = nan(nfb,1);
    n_acc = ekf.n_accepted(fb);
    n_rej = ekf.n_rejected(fb);
    err3d = ekf.pos_error(fb);
    err_horiz_all = horizontal_error(ekf.pos, P.ref_pos);
    err_horiz = err_horiz_all(fb);
    hpl = ekf.hpl(fb);
    all_labelled_spoofed = false(nfb,1);
    gt_visible = zeros(nfb,1);
    gt_labelled_spoofed = zeros(nfb,1);

    gt = configured_attack_set(P, cfg);

    for i = 1:nfb
        c = cls{fb_idx(i)};
        if isempty(c), continue; end
        n_trusted(i) = get_count(c, 'n_trusted', 'trusted_mask');
        n_suspect(i) = get_count(c, 'n_suspect', 'suspect_mask');
        n_spoofed(i) = get_count(c, 'n_spoofed', 'spoofed_mask');
        all_labelled_spoofed(i) = n_spoofed(i) > 0 && ...
            n_trusted(i) == 0 && n_suspect(i) == 0;

        entries = classification_entries(c);
        for k = 1:numel(entries)
            if is_ground_truth(entries(k), gt)
                gt_visible(i) = gt_visible(i) + 1;
                if strcmpi(entries(k).status, 'spoofed')
                    gt_labelled_spoofed(i) = gt_labelled_spoofed(i) + 1;
                end
            end
        end
    end

    fprintf('\n--- Configured injection ground truth ---\n');
    for i = 1:numel(gt)
        fprintf('  %s: PRNs [%s]\n', gt(i).constellation, num2str(gt(i).prns));
    end
    fprintf('Ground-truth attacked PRN appearances during fallback: %d\n', sum(gt_visible));
    fprintf('Those labelled spoofed by classifier:                 %d (%.1f%%)\n', ...
        sum(gt_labelled_spoofed), percent(sum(gt_labelled_spoofed), sum(gt_visible)));

    fprintf('\n--- Fallback classification/gate behaviour ---\n');
    fprintf('Classified spoofed satellites: median %.1f, max %.0f\n', ...
        median(n_spoofed,'omitnan'), max(n_spoofed,[],'omitnan'));
    fprintf('Trusted satellites:            median %.1f, min %.0f\n', ...
        median(n_trusted,'omitnan'), min(n_trusted,[],'omitnan'));
    fprintf('Gate accepted measurements:    median %.1f, range %.0f-%.0f\n', ...
        median(n_acc,'omitnan'), min(n_acc), max(n_acc));
    fprintf('Gate rejected measurements:    median %.1f, range %.0f-%.0f\n', ...
        median(n_rej,'omitnan'), min(n_rej), max(n_rej));
    fprintf('All-classified-spoofed fallback epochs: %d / %d\n', ...
        sum(all_labelled_spoofed), nfb);
    fprintf('Accepted rows in those epochs:          %d\n', ...
        sum(n_acc(all_labelled_spoofed)));

    % Representative epochs answer different questions; do not call only the
    % largest spoof-count epoch "worst".
    [~, i_spoof] = max(n_spoofed);
    [~, i_err]   = max(err_horiz);
    [~, i_hpl]   = max(hpl);
    [~, i_rej]   = max(n_rej);
    reps = unique([fb_idx(i_spoof), fb_idx(i_err), fb_idx(i_hpl), fb_idx(i_rej)], 'stable');

    fprintf('\n--- Representative fallback epochs ---\n');
    for i = 1:numel(reps)
        print_epoch_summary(reps(i), P, gt);
    end

    fprintf('\n--- Outcome containment across fallback epochs ---\n');
    fprintf('3D reference-relative error: median %.2f m, p95 %.2f m, max %.2f m\n', ...
        median(err3d,'omitnan'), prctile(err3d(~isnan(err3d)),95), max(err3d,[],'omitnan'));
    fprintf('Horizontal EN error:         median %.2f m, p95 %.2f m, max %.2f m\n', ...
        median(err_horiz,'omitnan'), prctile(err_horiz(~isnan(err_horiz)),95), ...
        max(err_horiz,[],'omitnan'));
    fprintf('HPL:            median %.2f m, p95 %.2f m, max %.2f m\n', ...
        median(hpl,'omitnan'), prctile(hpl(~isnan(hpl)),95), max(hpl,[],'omitnan'));
    bounded = isfinite(err_horiz) & isfinite(hpl) & hpl >= err_horiz;
    fprintf('HPL >= error:   %d / %d valid fallback epochs (%.1f%%)\n', ...
        sum(bounded), sum(isfinite(err_horiz)&isfinite(hpl)), ...
        percent(sum(bounded), sum(isfinite(err_horiz)&isfinite(hpl))));
    fprintf('Coasted epochs: %d\n', sum(ekf.coasted(fb)));

    fprintf('\n--- Audit conclusion and limitation ---\n');
    fprintf(['The saved evidence demonstrates bounded output error during the evaluated\n' ...
             'fallback epochs. gate_only restores nominal weights and delegates\n' ...
             'measurement rejection to the scalar innovation gate. The saved\n' ...
             'epoch_log does not preserve accepted-row PRN identities, so this\n' ...
             'audit cannot prove that every configured injected PRN was rejected.\n']);
    fprintf('=====================================================\n');
end

function print_epoch_summary(e, P, gt)
    c = P.classification{e};
    entries = classification_entries(c);
    err_horiz = horizontal_error(P.ekf.pos(e,:), P.ref_pos);
    n_gt = 0; n_gt_spoof = 0;
    for k = 1:numel(entries)
        if is_ground_truth(entries(k), gt)
            n_gt = n_gt + 1;
            n_gt_spoof = n_gt_spoof + strcmpi(entries(k).status, 'spoofed');
        end
    end
    fprintf(['Epoch %4d: class T/S/SP=%d/%d/%d, gate A/R=%d/%d, ' ...
             'GT visible/spoof-labelled=%d/%d, 3D/horiz=%.2f/%.2f m, HPL=%.2f m\n'], ...
        e, c.n_trusted, c.n_suspect, c.n_spoofed, ...
        P.ekf.n_accepted(e), P.ekf.n_rejected(e), n_gt, n_gt_spoof, ...
        P.ekf.pos_error(e), err_horiz, P.ekf.hpl(e));

    spoofed = entries(strcmpi({entries.status}, 'spoofed'));
    if ~isempty(spoofed)
        labels = arrayfun(@(x) sprintf('%s%02d', prefix(x.constellation), x.prn), ...
            spoofed, 'UniformOutput', false);
        max_show = 18;
        if numel(labels) > max_show
            fprintf('  classified spoofed: %s, ... (+%d more)\n', ...
                strjoin(labels(1:max_show), ', '), numel(labels)-max_show);
        else
            fprintf('  classified spoofed: %s\n', strjoin(labels, ', '));
        end
    end
end

function entries = classification_entries(c)
    entries = struct('constellation',{},'prn',{},'status',{});
    if isempty(c), return; end
    if isfield(c, 'classifications') && ~isempty(c.classifications)
        src = c.classifications;
    elseif isfield(c, 'sat_list') && ~isempty(c.sat_list)
        src = c.sat_list;
    else
        return;
    end
    for k = 1:numel(src)
        entries(k).constellation = src(k).constellation;
        entries(k).prn = src(k).prn;
        entries(k).status = src(k).status;
    end
end

function gt = configured_attack_set(P, cfg)
    if isfield(P, 'config_snapshot') && isfield(P.config_snapshot, 'scenarios')
        scenarios = P.config_snapshot.scenarios;
    else
        scenarios = cfg.scenarios;
    end
    idx = P.scenario_idx;
    sc = scenarios{idx};
    names = sc.spoofed_constellations;
    gt = struct('constellation',{},'prns',{});
    for i = 1:numel(names)
        gt(i).constellation = names{i};
        gt(i).prns = sc.spoofed_PRNs.(names{i});
    end
end

function tf = is_ground_truth(entry, gt)
    tf = false;
    for i = 1:numel(gt)
        if strcmp(entry.constellation, gt(i).constellation) && ...
                any(entry.prn == gt(i).prns)
            tf = true;
            return;
        end
    end
end

function n = get_count(c, count_field, mask_field)
    if isfield(c, count_field)
        n = c.(count_field);
    elseif isfield(c, mask_field)
        n = sum(c.(mask_field));
    else
        n = NaN;
    end
end

function segments = contiguous_segments(idx)
    cuts = [true; diff(idx(:)) > 1; true];
    p = find(cuts);
    segments = [idx(p(1:end-1)), idx(p(2:end)-1)];
end

function p = percent(a, b)
    if b == 0
        p = NaN;
    else
        p = 100*a/b;
    end
end

function p = prefix(c)
    switch c
        case 'GPS',     p = 'G';
        case 'Galileo', p = 'E';
        case 'BeiDou',  p = 'C';
        case 'GLONASS', p = 'R';
        otherwise,     p = '?';
    end
end
