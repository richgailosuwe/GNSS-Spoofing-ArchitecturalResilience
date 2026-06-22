function inspect_mat_fields()
% INSPECT_MAT_FIELDS  Print the actual field structure of one pipeline .mat
% and one ablation .mat, so figure code can use real field names (not assumed).

    config;
    pvt = fullfile(cfg.root,'results','pvt','scenario_1_gps_pipeline.mat');
    abl = fullfile(cfg.root,'results','ablation','scenario_1_gps_ablation.mat');

    fprintf('\n================ PIPELINE .mat ================\n');
    if isfile(pvt)
        S = load(pvt);
        topvars = fieldnames(S);
        fprintf('top-level variables: %s\n', strjoin(topvars,', '));
        % Find the main struct (pipeline_result or ans or similar)
        for i=1:numel(topvars)
            v = S.(topvars{i});
            if isstruct(v)
                fprintf('\n  struct "%s" fields:\n', topvars{i});
                show_struct(v, '    ');
            end
        end
    else
        fprintf('  not found: %s\n', pvt);
    end

    fprintf('\n================ ABLATION .mat ================\n');
    if isfile(abl)
        S = load(abl);
        topvars = fieldnames(S);
        fprintf('top-level variables: %s\n', strjoin(topvars,', '));
        for i=1:numel(topvars)
            v = S.(topvars{i});
            if isstruct(v)
                fprintf('\n  struct "%s" fields:\n', topvars{i});
                show_struct(v, '    ');
            end
        end
    else
        fprintf('  not found: %s\n', abl);
    end
    fprintf('\n==============================================\n');
end

function show_struct(s, indent)
    f = fieldnames(s);
    for i=1:numel(f)
        val = s.(f{i});
        sz = sprintf('%dx%d', size(val,1), size(val,2));
        cls = class(val);
        if isstruct(val)
            fprintf('%s%-22s [struct]\n', indent, f{i});
            sub = fieldnames(val);
            fprintf('%s    sub-fields: %s\n', indent, strjoin(sub,', '));
        else
            fprintf('%s%-22s [%s %s]\n', indent, f{i}, sz, cls);
        end
    end
end