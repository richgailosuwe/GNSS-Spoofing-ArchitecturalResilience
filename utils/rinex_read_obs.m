% to run fcn; obs = rinex_read_obs('data/rinex/observation/authentic.obs', cfg);
function obs = rinex_read_obs(obs_file, cfg)
% rinex_read_obs  Load and parse a RINEX observation file.
%
%   obs = rinex_read_obs(obs_file, cfg)
%
%   INPUT:
%     obs_file  - full path to the .obs RINEX file (string)
%     cfg       - configuration struct from config.m
%
%   OUTPUT:
%     obs       - struct with fields:
%                   .raw        raw output from rinexread()
%                   .GPS        GPS measurements struct
%                   .Galileo    Galileo measurements struct
%                   .BeiDou     BeiDou measurements struct
%                   .GLONASS    GLONASS measurements struct
%                   .epochs     datetime array of observation times
%
%   Each constellation struct contains:
%                   .pseudorange   [n_epochs x n_sats] metres
%                   .carrier_phase [n_epochs x n_sats] cycles
%                   .doppler       [n_epochs x n_sats] Hz
%                   .cn0           [n_epochs x n_sats] dB-Hz
%                   .prn           [1 x n_sats] satellite numbers
%                   .n_epochs      scalar
%                   .n_sats        scalar
%
%   Field names verified against BUCU00ROU RINEX file.

if cfg.verbose
    fprintf('      Reading observation file: %s\n', obs_file);
end

%% ── VALIDATE FILE ────────────────────────────────────────────────────────
if ~isfile(obs_file)
    error('rinex_read_obs: file not found:\n  %s', obs_file);
end

%% ── READ RAW RINEX ───────────────────────────────────────────────────────
raw    = rinexread(obs_file);
obs.raw = raw;

%% ── FIELD MAP (verified against your BUCU00ROU file) ─────────────────────
% Each row: {pseudorange_L1, pseudorange_L2, carrier_L1, carrier_L2, doppler, cn0}
field_map.GPS     = {'C1C', 'C2W', 'L1C', 'L2W', 'D1C', 'S1C'};
field_map.Galileo = {'C1C', 'C7Q', 'L1C', 'L7Q', 'D1C', 'S1C'};
field_map.BeiDou  = {'C1P', 'C2I', 'L1P', 'L2I', 'D1P', 'S1P'};
field_map.GLONASS = {'C1C', 'C2P', 'L1C', 'L2P', 'D1C', 'S1C'};

%% ── EXTRACT EACH CONSTELLATION ───────────────────────────────────────────
constellations = {'GPS', 'Galileo', 'BeiDou', 'GLONASS'};

for k = 1:length(constellations)
    name = constellations{k};

    if cfg.const.(name) && isfield(raw, name)
        obs.(name) = extract_constellation(raw.(name), name, field_map.(name));
        if cfg.verbose
            fprintf('      %-8s %d satellites, %d epochs\n', ...
                [name ':'], obs.(name).n_sats, obs.(name).n_epochs);
        end
    else
        obs.(name) = empty_constellation(name);
        if cfg.verbose
            fprintf('      %-8s not present or disabled\n', [name ':']);
        end
    end
end

%% ── EXTRACT EPOCH TIMES ──────────────────────────────────────────────────
for k = 1:length(constellations)
    name = constellations{k};
    if isfield(raw, name) && height(raw.(name)) > 0
        % Store UNIQUE epoch times only
        obs.epochs   = unique(raw.(name).Time);
        obs.n_epochs = length(obs.epochs);
        break;
    end
end

if cfg.verbose
    fprintf('      Total epochs: %d\n', obs.n_epochs);
    fprintf('      Time start:   %s\n', datestr(obs.epochs(1)));
    fprintf('      Time end:     %s\n', datestr(obs.epochs(end)));
end

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: extract_constellation
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function c = extract_constellation(tt, name, fields)
% Extract measurements from a rinexread timetable into a clean struct.
%
%   fields = {pr_L1, pr_L2, cp_L1, cp_L2, doppler, cn0}

    c.name = name;

    % Number of epochs
    c.n_epochs = height(tt);
    c.time = tt.Time;   % full time column — needed for epoch+PRN indexing

    % Extract satellite PRN numbers from SatelliteID column
    c.prn    = tt.SatelliteID;
    c.n_sats = length(unique(c.prn));

    % ── Pseudorange L1 ────────────────────────────────────────────────────
    c.pseudorange_L1  = safe_extract(tt, fields{1});

    % ── Pseudorange L2 ────────────────────────────────────────────────────
    c.pseudorange_L2  = safe_extract(tt, fields{2});

    % ── Carrier phase L1 ──────────────────────────────────────────────────
    c.carrier_phase_L1 = safe_extract(tt, fields{3});

    % ── Carrier phase L2 ──────────────────────────────────────────────────
    c.carrier_phase_L2 = safe_extract(tt, fields{4});

    % ── Doppler L1 ────────────────────────────────────────────────────────
    c.doppler         = safe_extract(tt, fields{5});

    % ── C/N0 L1 ───────────────────────────────────────────────────────────
    c.cn0             = safe_extract(tt, fields{6});

    % ── Elevation (not always present) ────────────────────────────────────
    if ismember('Elevation', tt.Properties.VariableNames)
        c.elevation = tt.Elevation;
    else
        c.elevation = nan(c.n_epochs, 1);
    end

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: safe_extract
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function data = safe_extract(tt, field_name)
% Extract a column from a timetable by field name.
% Returns NaN column if field does not exist.

    if ismember(field_name, tt.Properties.VariableNames)
        data = tt.(field_name);
    else
        warning('rinex_read_obs: field "%s" not found — filling with NaN', ...
            field_name);
        data = nan(height(tt), 1);
    end

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: empty_constellation
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function c = empty_constellation(name)
% Return empty struct when constellation is disabled or not in file.

    c.name            = name;
    c.pseudorange_L1  = [];
    c.pseudorange_L2  = [];
    c.carrier_phase_L1= [];
    c.carrier_phase_L2= [];
    c.doppler         = [];
    c.cn0             = [];
    c.elevation       = [];
    c.prn             = [];
    c.n_epochs        = 0;
    c.n_sats          = 0;

end