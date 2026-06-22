% main.m
% Default entry point for the GNSS spoofing detection pipeline.
% Runs Scenario 1 (GPS-only spoofing) in verbose mode.
%
% For batch runs across all five scenarios use:
%   run_all_scenarios
%
% For a specific scenario use:
%   run_pipeline(1)              % Scenario 1, quiet
%   run_pipeline(1, true)        % Scenario 1, verbose
%   run_pipeline('scenario_3_beidou', true)
%
% Scenarios:
%   1 = scenario_1_gps           GPS only
%   2 = scenario_2_galileo       Galileo only
%   3 = scenario_3_beidou        BeiDou only
%   4 = scenario_4_gps_glonass   GPS + GLONASS
%   5 = scenario_5_gps_galileo   GPS + Galileo (stress test)
%
% PROJECT:  GNSS Thesis, Universitatea Politehnica Bucuresti
% AUTHOR:   RG

clc; clear; close all;
run_pipeline(1, true);