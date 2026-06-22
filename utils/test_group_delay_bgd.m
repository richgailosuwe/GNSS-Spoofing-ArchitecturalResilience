function test_group_delay_bgd()
% TEST_GROUP_DELAY_BGD  Failing-first test for Galileo BGD band selection.
%
% Verifies get_group_delay returns the BGD term matching the CLOCK SOURCE of
% the selected Galileo navigation record:
%   F/NAV (DataSources bit 8, E5a,E1 clock) -> c * BGDE5aE1
%   I/NAV (DataSources bit 9, E5b,E1 clock) -> c * BGDE5bE1
%
% Spec basis (verified):
%   RINEX 3.02/4.0x: F/NAV clock is valid for E5a-E1; I/NAV clock for E5b-E1.
%   Broadcast Orbit 5 'DataSources' bits 8/9 indicate the frequency pair.
%   Real DataSources values: 0x102=258 (F/NAV, E5a,E1), 0x205=517 (I/NAV, E5b,E1).
%
% This test isolates a single nav row per case so row-selection is
% deterministic (the authentic file has both F/NAV and I/NAV rows at the
% same Toe for a given PRN).
%
% PROJECT: GNSS Thesis MATLAB Implementation, UPB
% AUTHOR:  RG

    PROJECT_ROOT = 'C:\Users\RG\Documents\MATLAB\MATLAB IMPLEMENTATION';
    addpath(PROJECT_ROOT);
    addpath(fullfile(PROJECT_ROOT, 'utils'));
    cd(PROJECT_ROOT);
    config;

    C_LIGHT = 299792458.0;

    fprintf('=======================================================\n');
    fprintf('  GALILEO BGD BAND-SELECTION TEST\n');
    fprintf('  %s\n', datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
    fprintf('=======================================================\n\n');

    % --- Load real Galileo nav records ------------------------------------
    nav = rinex_read_nav(fullfile(cfg.paths.nav,'authentic.nav'), cfg);
    ds  = nav.Galileo.data.DataSources;

    i_inav = find(ds == 517, 1);   % I/NAV (E5b,E1 clock) -> expect BGDE5bE1
    i_fnav = find(ds == 258, 1);   % F/NAV (E5a,E1 clock) -> expect BGDE5aE1
    assert(~isempty(i_inav), 'No I/NAV (ds=517) row found in authentic.nav');
    assert(~isempty(i_fnav), 'No F/NAV (ds=258) row found in authentic.nav');

    % --- Build isolated single-row nav structs for deterministic selection -
    nav_inav = make_single_row_nav(nav, i_inav);
    nav_fnav = make_single_row_nav(nav, i_fnav);

    prn_inav = nav.Galileo.prn(i_inav);
    prn_fnav = nav.Galileo.prn(i_fnav);
    t_inav   = nav.Galileo.toe(i_inav);   % query at Toe -> selects this row
    t_fnav   = nav.Galileo.toe(i_fnav);

    exp_inav = C_LIGHT * nav.Galileo.data.BGDE5bE1(i_inav);   % correct = E5b,E1
    exp_fnav = C_LIGHT * nav.Galileo.data.BGDE5aE1(i_fnav);   % correct = E5a,E1

    % =====================================================================
    % TEST 1: I/NAV row must use BGDE5bE1  (this FAILS on current code)
    % =====================================================================
    fprintf('-------------------------------------------------------\n');
    fprintf('TEST 1: I/NAV (ds=517) -> expect c*BGDE5bE1\n');
    fprintf('-------------------------------------------------------\n');
    got_inav = get_group_delay(nav_inav, prn_inav, 'Galileo', t_inav);
    wrong_inav = C_LIGHT * nav.Galileo.data.BGDE5aE1(i_inav);  % what bug returns
    fprintf('  PRN              : %d\n', prn_inav);
    fprintf('  expected (E5bE1) : %.6f m\n', exp_inav);
    fprintf('  got              : %.6f m\n', got_inav);
    fprintf('  (bug would give  : %.6f m using E5aE1)\n', wrong_inav);
    pass1 = abs(got_inav - exp_inav) < 1e-6;
    fprintf('\n  TEST 1 RESULT: %s\n\n', pf(pass1));

    % =====================================================================
    % TEST 2: F/NAV row must use BGDE5aE1  (must STAY correct after fix)
    % =====================================================================
    fprintf('-------------------------------------------------------\n');
    fprintf('TEST 2: F/NAV (ds=258) -> expect c*BGDE5aE1\n');
    fprintf('-------------------------------------------------------\n');
    got_fnav = get_group_delay(nav_fnav, prn_fnav, 'Galileo', t_fnav);
    fprintf('  PRN              : %d\n', prn_fnav);
    fprintf('  expected (E5aE1) : %.6f m\n', exp_fnav);
    fprintf('  got              : %.6f m\n', got_fnav);
    pass2 = abs(got_fnav - exp_fnav) < 1e-6;
    fprintf('\n  TEST 2 RESULT: %s\n\n', pf(pass2));

    % =====================================================================
    % TEST 3: I/NAV and F/NAV must give DIFFERENT results for same PRN
    %         (guards against a fix that collapses both to one band)
    % =====================================================================
    fprintf('-------------------------------------------------------\n');
    fprintf('TEST 3: I/NAV vs F/NAV must differ (PRN %d)\n', prn_inav);
    fprintf('-------------------------------------------------------\n');
    fprintf('  I/NAV gd : %.6f m\n', got_inav);
    fprintf('  F/NAV gd : %.6f m\n', got_fnav);
    differ = abs(got_inav - got_fnav) > 1e-9;
    fprintf('  differ   : %d (expected 1)\n', differ);
    fprintf('\n  TEST 3 RESULT: %s\n\n', pf(differ));

    % --- Summary ----------------------------------------------------------
    fprintf('=======================================================\n');
    fprintf('  SUMMARY\n');
    fprintf('  Test 1 (I/NAV uses BGDE5bE1): %s\n', pf(pass1));
    fprintf('  Test 2 (F/NAV uses BGDE5aE1): %s\n', pf(pass2));
    fprintf('  Test 3 (bands differ):       %s\n', pf(differ));
    fprintf('  Overall: %s\n', pf(pass1 && pass2 && differ));
    fprintf('=======================================================\n');
end

% =========================================================================
function nav_one = make_single_row_nav(nav, idx)
% Build a Galileo-only nav struct containing exactly one ephemeris row,
% so get_group_delay's closest-Toe selection is deterministic.
    nav_one = nav;
    g = nav.Galileo;
    g.prn  = g.prn(idx);
    g.toe  = g.toe(idx);
    g.data = g.data(idx, :);
    if isfield(g, 'SatelliteID'), g.SatelliteID = g.SatelliteID(idx); end
    nav_one.Galileo = g;
end

function s = pf(c)
    if c, s = 'PASS'; else, s = 'FAIL'; end
end