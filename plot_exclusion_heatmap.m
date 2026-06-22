function plot_exclusion_heatmap(scenario_name)
% PLOT_EXCLUSION_HEATMAP  Satellite trust/suspect/spoofed status over time.
%
% Loads saved pipeline evidence only and writes one PNG to results/figures.
% It does not rerun the pipeline and does not modify any .mat evidence.
%
% Default:
%   plot_exclusion_heatmap('scenario_1_gps')

    if nargin < 1 || isempty(scenario_name)
        scenario_name = 'scenario_1_gps';
    end

    config;
    fig_dir = fullfile(cfg.root, 'results', 'figures');
    if ~isfolder(fig_dir), mkdir(fig_dir); end

    fp = fullfile(cfg.paths.pvt, sprintf('%s_pipeline.mat', scenario_name));
    S = load(fp);
    P = S.pipeline_result;
    C = P.classification;
    n_epochs = numel(C);

    [sat_ids, sat_labels] = collect_satellite_ids(C);
    n_sat = numel(sat_ids);
    status_mat = zeros(n_sat, n_epochs);  % 0 = not visible / no classification

    id_to_row = containers.Map(sat_ids, num2cell(1:n_sat));
    for e = 1:n_epochs
        if isempty(C{e}) || ~isfield(C{e}, 'classifications'), continue; end
        cls = C{e}.classifications;
        for k = 1:numel(cls)
            id = sat_id(cls(k).constellation, cls(k).prn);
            if ~isKey(id_to_row, id), continue; end
            r = id_to_row(id);
            status_mat(r, e) = status_code(cls(k).status);
        end
    end

    attack_epoch = get_attack_epoch(P, cfg);

    f = figure('Color','w','Position',[100 100 1180 720],'Visible','off');
    imagesc(status_mat);
    axis tight;
    colormap([0.92 0.92 0.92; 0.18 0.62 0.25; 0.95 0.65 0.15; 0.82 0.15 0.12]);
    clim([-0.5 3.5]);
    cb = colorbar;
    cb.Ticks = [0 1 2 3];
    cb.TickLabels = {'not visible','trusted','suspect','spoofed'};
    hold on;
    xline(attack_epoch, 'k:', 'attack onset', ...
        'LineWidth', 1.2, 'LabelVerticalAlignment','bottom', ...
        'LabelHorizontalAlignment','right');

    tick_step = max(1, ceil(n_sat / 35));
    tick_idx = 1:tick_step:n_sat;
    yticks(tick_idx);
    yticklabels(sat_labels(tick_idx));
    set(gca, 'FontSize', 8, 'TickLength', [0 0]);
    xlabel('Epoch');
    ylabel('Satellite');
    title(sprintf('Satellite classification heatmap — %s', strrep(scenario_name,'_','\_')), ...
        'FontWeight','normal');

    out = fullfile(fig_dir, sprintf('exclusion_heatmap_%s.png', scenario_name));
    exportgraphics(f, out, 'Resolution', 300);
    close(f);
    fprintf('Saved: %s\n', out);
end

function [sat_ids, sat_labels] = collect_satellite_ids(C)
    ids = {};
    const_rank = containers.Map({'GPS','Galileo','BeiDou','GLONASS'}, [1 2 3 4]);
    ranks = [];
    prns = [];

    for e = 1:numel(C)
        if isempty(C{e}) || ~isfield(C{e}, 'classifications'), continue; end
        cls = C{e}.classifications;
        for k = 1:numel(cls)
            id = sat_id(cls(k).constellation, cls(k).prn);
            if any(strcmp(ids, id)), continue; end
            ids{end+1} = id; %#ok<AGROW>
            if isKey(const_rank, cls(k).constellation)
                ranks(end+1) = const_rank(cls(k).constellation); %#ok<AGROW>
            else
                ranks(end+1) = 99; %#ok<AGROW>
            end
            prns(end+1) = cls(k).prn; %#ok<AGROW>
        end
    end

    [~, order] = sortrows([ranks(:), prns(:)]);
    sat_ids = ids(order);
    sat_labels = sat_ids;
end

function id = sat_id(constellation, prn)
    switch constellation
        case 'GPS'
            prefix = 'G';
        case 'Galileo'
            prefix = 'E';
        case 'BeiDou'
            prefix = 'C';
        case 'GLONASS'
            prefix = 'R';
        otherwise
            prefix = '?';
    end
    id = sprintf('%s%02d', prefix, prn);
end

function code = status_code(status)
    switch lower(char(status))
        case 'trusted'
            code = 1;
        case 'suspect'
            code = 2;
        case 'spoofed'
            code = 3;
        otherwise
            code = 0;
    end
end

function attack_epoch = get_attack_epoch(P, cfg)
    attack_epoch = 120;
    if isfield(P, 'scenario_idx') && P.scenario_idx <= numel(cfg.scenarios)
        if isfield(cfg.scenarios{P.scenario_idx}, 'start_epoch')
            attack_epoch = cfg.scenarios{P.scenario_idx}.start_epoch;
        end
    end
end
