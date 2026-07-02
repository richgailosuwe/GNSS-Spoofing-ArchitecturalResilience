function generate_all_figures()
% GENERATE_ALL_FIGURES  Batch figure generation from saved evidence .mat files.
%
%   generate_all_figures()
%
% Self-contained: loads results/pvt/*_pipeline.mat and results/ablation/
% *_ablation.mat and produces Chapter 6 figures, saving PNGs (300 dpi) to
% results/figures/. Does NOT recompute the pipeline — plots only from saved
% data, so it is safe to run unattended after run_all_scenarios + ablations.
%
% Figures produced:
%   1. Authentic baseline error + HPL timeline     err_hpl_timeline_baseline_authentic.png
%   2. Per-scenario error + HPL timeline           err_hpl_timeline_<name>.png
%   3. Per-scenario ablation A/B/C comparison      ablation_<name>.png
%   4. Combined Stanford diagram (all scenarios)   stanford_combined.png
%   5. Per-scenario Stanford diagram               stanford_<name>.png
%
% FIGURE CATALOGUE (what each output shows, for thesis reference):
%
%   err_hpl_timeline_<scenario>.png
%       Horizontal position error and HPL vs epoch for one scenario. Shows
%       whether HPL bounds the error (HPL line above error line = integrity
%       maintained) and how both evolve through the attack window. The two
%       reference lines (185.2 / 370.4 m) give descriptive scale. Use to
%       demonstrate the integrity layer tracking and bounding position error.
%
%   ablation_<scenario>.png
%       A/B/C 3D reference-relative error curves overlaid: full pipeline (A) vs gate-only
%       (B) vs no mitigation (C). The recovery proof: C drifts away under
%       attack while A and B stay low. Use to show that mitigation prevents
%       the unmitigated drift, and that A ~= B (gate is the main protection).
%
%   stanford_<scenario>.png  and  stanford_combined.png
%       Each epoch as an (error, HPL) point on the Stanford integrity plane.
%       Points ABOVE the diagonal = error bounded by HPL (safe); BELOW =
%       misleading information (HPL failed to bound error; should be empty).
%       Reference lines mark 185.2 m and 2x = 370.4 m. Combined overlays all
%       scenarios. Use to show no epoch falls in the unsafe region.
%
%   batch_summary_bars.png
%       Grouped bar chart comparing final, median and p95 horizontal error
%       and p95 HPL across all scenarios (mitigated full-pipeline result). Ref lines
%       at 185.2 / 370.4 m. Use as the Chapter 6 cross-scenario overview: at a
%       glance, which attacks the mitigated system handles within scale.
%
% NOTE: the chi-squared absorption timeline (plot_chi2_timeline.m) is NOT
% generated here — it recomputes from a special position-drag injection path
% and should be run separately/interactively. This wrapper only uses saved PVT.
%

    config;
    REF_SCALE = 185.2;     % RNP-0.1 reference scale [m] (descriptive)
    REF_2X    = 370.4;     % 2 x reference scale [m] (descriptive, not certified)

    pvt_dir = fullfile(cfg.root, 'results', 'pvt');
    abl_dir = fullfile(cfg.root, 'results', 'ablation');
    fig_dir = fullfile(cfg.root, 'results', 'figures');
    if ~isfolder(fig_dir), mkdir(fig_dir); end

    manifest = {};

    fprintf('\n=====================================================\n');
    fprintf('  GENERATE ALL FIGURES\n');
    fprintf('=====================================================\n');
    fprintf('Figure catalogue (what each plot shows):\n');
    fprintf('  err_hpl_timeline_baseline_authentic.png : authentic horizontal error & HPL.\n');
    fprintf('  err_hpl_timeline_<s>.png : error & HPL vs epoch; checks HPL bounds error.\n');
    fprintf('  ablation_<s>.png         : A full / B gate-only / C none; the recovery proof.\n');
    fprintf('  stanford_<s>.png         : per-epoch (error,HPL) integrity plane; above diagonal=safe.\n');
    fprintf('  stanford_combined.png    : all scenarios overlaid on the Stanford plane.\n');
    fprintf('  batch_summary_bars.png   : final/median/p95 error + p95 HPL per scenario.\n');
    fprintf('-----------------------------------------------------\n');

    % Resolve scenario names from cfg (authoritative order).
    scen_names = cell(numel(cfg.scenarios),1);
    for i = 1:numel(cfg.scenarios), scen_names{i} = cfg.scenarios{i}.name; end

    % Accumulator for combined Stanford.
    comb = struct('label',{},'err',{},'hpl',{},'color',{});
    cmap = lines(numel(scen_names)+1);

    % Accumulator for batch summary bar plot (per-scenario error metrics).
    bar_labels  = {};
    bar_final   = [];
    bar_median  = [];
    bar_p95     = [];
    bar_hpl_p95 = [];

    % ===================================================================
    % Authentic baseline: horizontal EN error + HPL from saved evidence
    % ===================================================================
    baseline_fp = fullfile(pvt_dir, 'baseline_authentic.mat');
    if isfile(baseline_fp)
        B = load(baseline_fp, 'baseline');
        ekf_b = B.baseline.ekf;
        err_b = horizontal_error(ekf_b.pos, cfg.ref_pos);
        hpl_b = ekf_b.hpl(:);
        nep_b = numel(err_b);

        f = figure('Color','w','Position',[100 100 900 460],'Visible','off');
        hold on; grid on; box on;
        p_err = plot(1:nep_b, err_b, '-', 'LineWidth',1.2, 'Color',[0.10 0.45 0.70]);
        p_hpl = plot(1:nep_b, hpl_b, '-', 'LineWidth',1.2, 'Color',[0.85 0.45 0.0]);
        yline(REF_SCALE, ':', sprintf('RNP-0.1 ref %.1f m',REF_SCALE), ...
            'Color',[0.5 0.5 0.5],'FontSize',8,'LabelHorizontalAlignment','right');
        yline(REF_2X, '--', sprintf('2x ref %.1f m',REF_2X), ...
            'Color',[0.75 0.1 0.1],'FontSize',8,'LabelHorizontalAlignment','right');
        xlabel('Epoch'); ylabel('Metres');
        title('Authentic baseline - horizontal error and HPL','FontWeight','normal');
        legend([p_err p_hpl], {'Horizontal error','HPL'}, ...
            'Location','north','Orientation','horizontal');
        op = fullfile(fig_dir, 'err_hpl_timeline_baseline_authentic.png');
        exportgraphics(f, op, 'Resolution',300); close(f);
        manifest{end+1} = op; %#ok<AGROW>
        fprintf('  [ok]  %s\n', op);
    else
        fprintf('  [skip] baseline_authentic.mat not found\n');
    end

    % ===================================================================
    % 1 + 4: per-scenario timeline + per-scenario Stanford, from pipeline .mat
    % ===================================================================
    for i = 1:numel(scen_names)
        name = scen_names{i};
        fp = fullfile(pvt_dir, sprintf('%s_pipeline.mat', name));
        if ~isfile(fp)
            fprintf('  [skip] %s — pipeline .mat not found\n', name);
            continue;
        end
        S = load(fp);
        if ~isfield(S,'pipeline_result')
            fprintf('  [skip] %s — no pipeline_result\n', name);
            continue;
        end
        ekf = S.pipeline_result.ekf;
        % HPL is horizontal, so every error compared with it must be the
        % horizontal EN component. ekf.pos_error is a 3D ECEF norm.
        ref = S.pipeline_result.ref_pos;
        err = horizontal_error(ekf.pos, ref);
        hpl = ekf.hpl(:);
        nep = numel(err);

        % --- Figure 1: error + HPL timeline ---
        f = figure('Color','w','Position',[100 100 900 460],'Visible','off');
        hold on; grid on; box on;
        ph1 = plot(1:nep, err, '-', 'LineWidth',1.2, 'Color',[0.10 0.45 0.70]);
        ph2 = plot(1:nep, hpl, '-', 'LineWidth',1.2, 'Color',[0.85 0.45 0.0]);
        % Reference-line labels on the RIGHT here (left edge has the HPL bootstrap
        % spike); legend placed top-centre horizontal to avoid covering either.
        yline(REF_SCALE, ':',  sprintf('RNP-0.1 ref %.1f m',REF_SCALE), ...
              'Color',[0.5 0.5 0.5],'FontSize',8,'LabelHorizontalAlignment','right');
        yline(REF_2X,    '--', sprintf('2x ref %.1f m',REF_2X), ...
              'Color',[0.75 0.1 0.1],'FontSize',8,'LabelHorizontalAlignment','right');
        if isfield(cfg.scenarios{i},'start_epoch')
            xlt = xline(cfg.scenarios{i}.start_epoch, ':', 'attack onset', ...
                  'Color',[0.4 0.4 0.4],'FontSize',8);
            xlt.LabelVerticalAlignment = 'bottom';
            xlt.LabelHorizontalAlignment = 'right';
        end
        xlabel('Epoch'); ylabel('Metres');
        title(sprintf('%s — horizontal error and HPL', strrep(name,'_','\_')), ...
              'FontWeight','normal');
        legend([ph1 ph2],{'Horizontal error','HPL'},'Location','north','Orientation','horizontal');
        op = fullfile(fig_dir, sprintf('err_hpl_timeline_%s.png', name));
        exportgraphics(f, op, 'Resolution',300); close(f);
        manifest{end+1} = op; %#ok<AGROW>
        fprintf('  [ok]  %s\n', op);

        % --- Figure 4: per-scenario Stanford ---
        f = figure('Color','w','Position',[100 100 560 540],'Visible','off');
        hold on; box on;
        ax_max = max([REF_2X*1.05, max(hpl,[],'omitnan')*1.1, max(err,[],'omitnan')*1.1]);
        ax_max = ceil(ax_max/50)*50;
        plot([0 ax_max],[0 ax_max],'k-','LineWidth',1.0);
        plot([0 ax_max],[REF_2X REF_2X],'--','Color',[0.75 0.1 0.1]);
        plot([0 ax_max],[REF_SCALE REF_SCALE],':','Color',[0.85 0.45 0.0]);
        v = ~isnan(err)&~isnan(hpl);
        plot(err(v),hpl(v),'.','Color',[0.10 0.45 0.70],'MarkerSize',8);
        text(ax_max*0.02, REF_2X+ax_max*0.012, sprintf('2 \\times RNP-0.1 ref = %.1f m',REF_2X),...
             'FontSize',8,'Color',[0.75 0.1 0.1]);
        xlabel('Horizontal error [m]'); ylabel('HPL [m]');
        title(sprintf('Stanford — %s', strrep(name,'_','\_')),'FontWeight','normal');
        xlim([0 ax_max]); ylim([0 ax_max]); axis square;
        op = fullfile(fig_dir, sprintf('stanford_%s.png', name));
        exportgraphics(f, op, 'Resolution',300); close(f);
        manifest{end+1} = op; %#ok<AGROW>
        fprintf('  [ok]  %s\n', op);

        % accumulate for combined Stanford
        comb(end+1) = struct('label',strrep(name,'_','\_'),'err',err(v),'hpl',hpl(v),...
                             'color',cmap(i,:)); %#ok<AGROW>

        % accumulate for batch summary bars
        bar_labels{end+1} = strrep(name,'_','\_');           %#ok<AGROW>
        bar_final(end+1)  = err(end);                          %#ok<AGROW>
        bar_median(end+1) = median(err,'omitnan');             %#ok<AGROW>
        bar_p95(end+1)    = prctile(err(~isnan(err)),95);      %#ok<AGROW>
        bar_hpl_p95(end+1)= prctile(hpl(~isnan(hpl)),95);      %#ok<AGROW>
    end

    % ===================================================================
    % 3: combined Stanford diagram
    % ===================================================================
    if ~isempty(comb)
        f = figure('Color','w','Position',[100 100 640 600],'Visible','off');
        hold on; box on;
        all_e = vertcat(comb.err); all_h = vertcat(comb.hpl);
        ax_max = max([REF_2X*1.05, max(all_h)*1.1, max(all_e)*1.1]);
        ax_max = ceil(ax_max/50)*50;
        plot([0 ax_max],[0 ax_max],'k-','LineWidth',1.0);
        plot([0 ax_max],[REF_2X REF_2X],'--','Color',[0.75 0.1 0.1]);
        plot([0 ax_max],[REF_SCALE REF_SCALE],':','Color',[0.85 0.45 0.0]);
        h = gobjects(numel(comb),1);
        for s = 1:numel(comb)
            h(s) = plot(comb(s).err, comb(s).hpl, '.', 'Color',comb(s).color, ...
                        'MarkerSize',8,'DisplayName',comb(s).label);
        end
        text(ax_max*0.02, REF_2X+ax_max*0.012, sprintf('2 \\times RNP-0.1 ref = %.1f m',REF_2X),...
             'FontSize',8,'Color',[0.75 0.1 0.1]);
        xlabel('Horizontal error [m]'); ylabel('HPL [m]');
        title('Stanford integrity diagram — all scenarios','FontWeight','normal');
        xlim([0 ax_max]); ylim([0 ax_max]); axis square;
        legend(h,'Location','southeast','FontSize',8);
        op = fullfile(fig_dir,'stanford_combined.png');
        exportgraphics(f, op, 'Resolution',300); close(f);
        manifest{end+1} = op;
        fprintf('  [ok]  %s\n', op);
    end

    % ===================================================================
    % 5: batch summary bar plot (per-scenario error metrics)
    % ===================================================================
    if ~isempty(bar_final)
        f = figure('Color','w','Position',[100 100 920 480],'Visible','off');
        hold on; grid on; box on;
        M = [bar_final(:), bar_median(:), bar_p95(:), bar_hpl_p95(:)];
        b = bar(M, 'grouped');
        b(1).FaceColor=[0.10 0.45 0.70];
        b(2).FaceColor=[0.20 0.60 0.20];
        b(3).FaceColor=[0.85 0.45 0.0];
        b(4).FaceColor=[0.55 0.35 0.65];
        yline(REF_SCALE, ':',  sprintf('RNP-0.1 ref %.1f m',REF_SCALE),...
              'Color',[0.5 0.5 0.5],'FontSize',8);
        yline(REF_2X,    '--', sprintf('2x ref %.1f m',REF_2X),...
              'Color',[0.75 0.1 0.1],'FontSize',8);
        set(gca,'XTick',1:numel(bar_labels),'XTickLabel',bar_labels,...
            'XTickLabelRotation',20,'FontSize',9);
        ylabel('Horizontal error [m]');
        title('Per-scenario error summary (mitigated full pipeline)','FontWeight','normal');
        legend({'final err','median err','p95 err','p95 HPL'},'Location','northwest');
        op = fullfile(fig_dir,'batch_summary_bars.png');
        exportgraphics(f, op, 'Resolution',300); close(f);
        manifest{end+1} = op;
        fprintf('  [ok]  %s\n', op);
    end

    % ===================================================================
    % 2: per-scenario ablation A/B/C comparison, from ablation .mat
    % ===================================================================
    for i = 1:numel(scen_names)
        name = scen_names{i};
        fp = fullfile(abl_dir, sprintf('%s_ablation.mat', name));
        if ~isfile(fp), continue; end
        S = load(fp);
        if ~isfield(S,'results') || ~isfield(S.results,'peA'), continue; end
        R = S.results;
        nep = numel(R.peA);
        f = figure('Color','w','Position',[100 100 900 460],'Visible','off');
        hold on; grid on; box on;
        % Draw C first (background), then B, then A LAST and slightly thicker so
        % the full-pipeline trace stays visible where A and B overlap (A~=B is the
        % finding; both must be visible to show it).
        pC = plot(1:nep, R.peC, '-','LineWidth',1.0,'Color',[0.75 0.10 0.10]);
        pB = plot(1:nep, R.peB, '-','LineWidth',1.6,'Color',[0.20 0.60 0.20]);
        pA = plot(1:nep, R.peA, '-','LineWidth',1.0,'Color',[0.10 0.45 0.70]);
        % Reference lines: labels at LEFT edge (clear of the busy right-side curve).
        yline(REF_SCALE, ':',  sprintf('RNP-0.1 ref %.1f m',REF_SCALE),...
              'Color',[0.5 0.5 0.5],'FontSize',8,'LabelHorizontalAlignment','left');
        yline(REF_2X,    '--', sprintf('2x ref %.1f m',REF_2X),...
              'Color',[0.75 0.1 0.1],'FontSize',8,'LabelHorizontalAlignment','left');
        if isfield(R,'start_epoch')
            xl = xline(R.start_epoch, ':', 'attack onset','Color',[0.4 0.4 0.4],'FontSize',8);
            xl.LabelVerticalAlignment = 'bottom';
            xl.LabelHorizontalAlignment = 'right';
        end
        xlabel('Epoch'); ylabel('3D reference-relative position error [m]');
        title(sprintf('Mitigation ablation — %s', strrep(name,'_','\_')),'FontWeight','normal');
        legend([pA pB pC],{'A: full pipeline','B: gate-only','C: no mitigation'},...
               'Location','north','Orientation','horizontal');
        op = fullfile(fig_dir, sprintf('ablation_%s.png', name));
        exportgraphics(f, op, 'Resolution',300); close(f);
        manifest{end+1} = op; %#ok<AGROW>
        fprintf('  [ok]  %s\n', op);
    end

    % ===================================================================
    % Manifest
    % ===================================================================
    fprintf('\n--- Figure manifest (%d files) ---\n', numel(manifest));
    for k = 1:numel(manifest)
        fprintf('  %s\n', manifest{k});
    end
    fprintf('=====================================================\n');
end
