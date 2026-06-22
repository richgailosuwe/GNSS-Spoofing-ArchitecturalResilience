%to test fucn run nav = rinex_read_nav('data/rinex/navigation/authentic.nav', cfg);
function nav = rinex_read_nav(nav_file, cfg)
% rinex_read_nav  Load and parse a RINEX navigation file.
%
%   nav = rinex_read_nav(nav_file, cfg)
%
%   INPUT:
%     nav_file  - full path to the .nav RINEX file (string)
%     cfg       - configuration struct from config.m
%
%   OUTPUT:
%     nav       - struct with fields:
%                   .raw        raw output from rinexread()
%                   .GPS        GPS ephemeris struct
%                   .Galileo    Galileo ephemeris struct
%                   .BeiDou     BeiDou ephemeris struct
%                   .GLONASS    GLONASS ephemeris struct
%
%   Each constellation struct contains:
%                   .prn        satellite PRN numbers
%                   .toe        time of ephemeris (datetime)
%                   .data       full ephemeris timetable

if cfg.verbose
    fprintf('      Reading navigation file: %s\n', nav_file);
end

%% ── VALIDATE FILE ────────────────────────────────────────────────────────
if ~isfile(nav_file)
    error('rinex_read_nav: file not found:\n  %s', nav_file);
end

%% ── READ RAW RINEX ───────────────────────────────────────────────────────
raw     = rinexread(nav_file);
nav.raw = raw;

%% ── EXTRACT EACH CONSTELLATION ───────────────────────────────────────────
constellations = {'GPS', 'Galileo', 'BeiDou', 'GLONASS'};

for k = 1:length(constellations)
    name = constellations{k};

    if cfg.const.(name) && isfield(raw, name)
        nav.(name) = extract_nav(raw.(name), name);
        if cfg.verbose
            fprintf('      %-8s %d ephemeris records\n', ...
                [name ':'], height(raw.(name)));
        end
    else
        nav.(name) = empty_nav(name);
        if cfg.verbose
            fprintf('      %-8s not present or disabled\n', [name ':']);
        end
    end
end

end % main function

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: extract_nav
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function n = extract_nav(tt, name)
% Extract ephemeris data from a rinexread navigation timetable.

    n.name = name;
    n.data = tt;

    % Satellite PRN numbers
    if ismember('SatelliteID', tt.Properties.VariableNames)
        n.prn = tt.SatelliteID;
    else
        n.prn = [];
    end

    % Time of ephemeris
    n.toe = tt.Time;

    % Total records
    n.n_records = height(tt);

end

%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
%% LOCAL HELPER: empty_nav
%% ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
function n = empty_nav(name)
% Return empty struct when constellation is disabled or not in file.

    n.name      = name;
    n.data      = [];
    n.prn       = [];
    n.toe       = [];
    n.n_records = 0;

end