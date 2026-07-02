function [H, pr_pred, isb_col_per_row] = build_meas_model_isb(sat_positions, constellations, x_pred, cfg)
% BUILD_MEAS_MODEL_ISB  Stage 4 — measurement model with inter-system bias (ISB).
%
% Single source of truth for the EKF observation matrix H and the predicted
% pseudorange pr_pred.  Both ekf_runner.m (full innovation vector) and
% ekf_update.m (accepted-measurement update) call this, so the two cannot
% drift apart.
%
% MEASUREMENT MODEL (Method 1: GPS master clock + N-1 ISB states):
%   For a GPS satellite:
%     z_i = ||sat_i - pos|| + clk_GPS + noise
%     H_i = [ -e_i^T , 0_3^T , 1 , 0 , 0 ... 0 ]
%   For a non-GPS satellite of constellation c:
%     z_i = ||sat_i - pos|| + clk_GPS + ISB_c + noise
%     H_i = [ -e_i^T , 0_3^T , 1 , 0 , ... 1 (in the ISB_c column) ... ]
%
% STATE LAYOUT (n_states = 8 + n_isb):
%   1:3 = position, 4:6 = velocity, 7 = clk_GPS, 8 = clk_drift,
%   9..(8+n_isb) = ISB states, ordered by cfg.ekf.isb_order.
%
% BACKWARD COMPATIBILITY:
%   When numel(x_pred) == 8 (GPS-only configuration), no ISB columns are
%   added and pr_pred = rng + clk_GPS, i.e. exactly the original single-clock
%   model.  cfg.ekf.isb_order is only read when the state is extended, so a
%   minimal cfg (as in test_ekf.m) works unchanged.
%
% INPUTS
%   sat_positions  [m x 3] satellite ECEF positions [m]
%   constellations {m x 1} cell of labels ('GPS'|'Galileo'|'BeiDou'|'GLONASS')
%                  one per row of sat_positions.  May be omitted/empty when
%                  the state is 8 (all rows treated as GPS).
%   x_pred         [n_states x 1] predicted state
%   cfg            config struct (reads cfg.ekf.isb_order only if n_states>8)
%
% OUTPUTS
%   H               [m x n_states] observation matrix
%   pr_pred         [m x 1] predicted pseudorange (incl. clk and ISB)
%   isb_col_per_row [m x 1] ISB state-column index used by each row (0 = none)
%
% STAGE:    4 — EKF Position Recovery (ISB measurement model)

    n_meas   = size(sat_positions, 1);
    n_states = numel(x_pred);
    pos_pred = x_pred(1:3);
    clk_pred = x_pred(7);

    use_isb = (n_states > 8);
    if use_isb
        isb_order = cfg.ekf.isb_order;     % e.g. {'Galileo','BeiDou','GLONASS'}
    else
        isb_order = {};
    end

    if nargin < 2 || isempty(constellations)
        constellations = repmat({'GPS'}, n_meas, 1);
    end

    H               = zeros(n_meas, n_states);
    pr_pred         = zeros(n_meas, 1);
    isb_col_per_row = zeros(n_meas, 1);

    for i = 1:n_meas
        r_vec    = sat_positions(i,:)' - pos_pred;
        rng      = norm(r_vec);
        e_i      = r_vec / rng;

        H(i,1:3) = -e_i';      % position partials
        H(i,7)   = 1;          % GPS master clock column

        isb_val = 0;
        if use_isb
            cn = constellations{i};
            if ~strcmp(cn, 'GPS')
                idx = find(strcmp(isb_order, cn), 1);   % position within isb_order
                if ~isempty(idx)
                    col                = 8 + idx;        % ISB state column
                    H(i, col)          = 1;
                    isb_col_per_row(i) = col;
                    isb_val            = x_pred(col);
                end
            end
        end

        pr_pred(i) = rng + clk_pred + isb_val;
    end
end
