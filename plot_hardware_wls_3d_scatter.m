function plot_hardware_wls_3d_scatter()
% PLOT_HARDWARE_WLS_3D_SCATTER  3-D WLS scatter about session mean.
%
% Reads saved real-hardware validation .mat files only and writes a figure to
% results/figures/. No pipeline rerun, no evidence modification.

config;

sessions = {
    'Football pitch A', 'june18footballpitchdomnesti2_validation.mat';
    'Football pitch B', 'june18footballpitchdomnesti_validation.mat';
    'Rooftop A (2 epochs)', 'june16rooftop_validation.mat';
    'Rooftop B',            'june21rooftop_validation.mat'
};

fig_dir = fullfile(cfg.root, 'results', 'figures');
if ~isfolder(fig_dir), mkdir(fig_dir); end

figure('Color', 'w', 'Position', [100 100 1450 980]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for s = 1:size(sessions, 1)
    label = sessions{s, 1};
    path_mat = fullfile(cfg.root, 'results', 'hardware', sessions{s, 2});
    S = load(path_mat);
    fn = fieldnames(S);
    R = S.(fn{1});

    pos = R.wls_pos;
    pos = pos(all(isfinite(pos), 2), :);
    ref = mean(pos, 1, 'omitnan');
    enu = ecef_to_enu_about_ref(pos, ref);
    scatter3d = vecnorm(enu, 2, 2);

    med3d = median(scatter3d, 'omitnan');
    p953d = prctile(scatter3d, 95);

    nexttile;
    scatter3(enu(:,1), enu(:,2), enu(:,3), 10, scatter3d, 'filled', ...
        'MarkerFaceAlpha', 0.70, 'MarkerEdgeAlpha', 0.70);
    hold on;
    plot3(0, 0, 0, 'kx', 'LineWidth', 2.0, 'MarkerSize', 12);
    grid on;
    axis vis3d;
    view(38, 24);
    colormap(gca, turbo);
    cb = colorbar;
    cb.Label.String = '3-D scatter [m]';
    cb.TickLabelInterpreter = 'none';

    lim = max(prctile(abs(enu), 99, 'all'), 10);
    xlim([-lim lim]);
    ylim([-lim lim]);
    zlim([-lim lim]);

    xlabel('East about WLS mean [m]');
    ylabel('North about WLS mean [m]');
    zlabel('Up about WLS mean [m]');
    title({label, sprintf('n=%d, median %.2f m, p95 %.2f m', size(pos, 1), med3d, p953d)}, ...
        'FontWeight', 'bold');
end

sgtitle({'Real hardware WLS 3-D scatter about session mean', ...
    'Standalone ZED-F9P; precision/stability only, no surveyed truth', ...
    'Axes clipped to 99th-percentile coordinate spread for cluster visibility'}, ...
    'FontWeight', 'bold');

out_png = fullfile(fig_dir, 'hardware_wls_3d_scatter_all_sessions.png');
exportgraphics(gcf, out_png, 'Resolution', 220);
fprintf('Saved: %s\n', out_png);
end

function enu = ecef_to_enu_about_ref(pos, ref)
% Convert ECEF positions to local ENU deviations about ref.
ref = ref(:)';
[lat, lon, ~] = ecef2lla_simple(ref);
R = [ -sin(lon),            cos(lon),           0;
      -sin(lat)*cos(lon),  -sin(lat)*sin(lon),  cos(lat);
       cos(lat)*cos(lon),   cos(lat)*sin(lon),  sin(lat) ];
enu = (R * (pos - ref)')';
end
