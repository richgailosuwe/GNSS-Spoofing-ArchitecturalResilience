function osnma_result = osnma_status(obs_epoch, cfg)
% OSNMA_STATUS  Stage 0 — Galileo OSNMA authentication status per satellite.
%
% OSNMA (Open Service Navigation Message Authentication) is Galileo's
% cryptographic signal authentication protocol based on the TESLA
% (Timed Efficient Stream Loss-tolerant Authentication) chain.  Each
% Galileo satellite broadcasts navigation message data alongside MAC
% (Message Authentication Code) tags.  A receiver verifies these tags
% against a root public key published by the European Union Agency for
% the Space Programme (EUSPA), confirming the navigation data originates
% from a real Galileo satellite and has not been modified or spoofed.
%
% Reference: Galileo OSNMA Signal-In-Space Interface Control Document,
% v1.1, European Union Agency for the Space Programme, 2023.
% https://www.gsc-europa.eu/sites/default/files/sites/all/files/
%   Galileo_OSNMA_SIS_ICD_v1.1.pdf
%
% =========================================================================
% HARDWARE SCOPE STATEMENT (mandatory for thesis honesty):
%
% Full OSNMA verification requires:
%   (1) Raw Galileo I/NAV navigation message bits (INAV pages)
%   (2) TESLA key chain received in real time over the signal
%   (3) EUSPA public key (PKID) loaded into the receiver
%   (4) Sub-frame timing accurate to the OSNMA timing constraints
%
% The RINEX observation format pre-dates OSNMA and carries none of these.
% The u-blox ZED-F9P-04B-01 used in this project runs firmware HPG 1.32,
% which does NOT support OSNMA.  OSNMA support was introduced in HPG 1.50
% (released July 2024, u-blox ZED-F9P HPG 1.50 Release Note,
% UBXDOC-963802114-12826).  The HPG 1.32 interface description contains
% no UBX-SEC-OSNMA message and no CFG-GAL-USE_OSNMA configuration item.
%
% Upgrading the ZED-F9P firmware to HPG 1.50 would enable OSNMA via the
% UBX-SEC-OSNMA message (class 0x27, ID 0x05), which reports per-satellite
% authentication status.  That upgrade path is documented in the thesis
% as a hardware validation item for future work.
%
% THIS MODULE operates in SIMULATION MODE only.  It simulates the
% authentication status that an OSNMA-capable receiver would report,
% using the Stage 2 classification results as a proxy for the
% cryptographic verdict.  This tests the pipeline's handling of the
% OSNMA output (the integration point) without performing the
% cryptographic verification itself.
%
% SIMULATION LOGIC:
%   trusted   ->  AUTH_OK       (navigation data authentic)
%   suspect   ->  AUTH_UNKNOWN  (authentication inconclusive)
%   spoofed   ->  AUTH_FAIL     (navigation data authentication failed)
%
% In a real OSNMA-capable receiver:
%   AUTH_OK      = TESLA chain verified against EUSPA root key
%   AUTH_UNKNOWN = insufficient INAV pages received yet (receiver cold start,
%                  or satellite newly acquired)
%   AUTH_FAIL    = MAC tag mismatch — spoofing or data corruption detected
%
% The simulation maps the Stage 2 classification to these states, which
% is conservative: a spoofed GPS satellite would not produce AUTH_FAIL
% from OSNMA (OSNMA only authenticates Galileo), so in a real pipeline
% only Galileo satellites would have non-AUTH_UNKNOWN status.  The
% simulation is therefore a best-case scenario for OSNMA coverage, not
% a claim that OSNMA can detect GPS spoofing.
%
% INPUTS
%   obs_epoch   struct — current epoch observations, with fields:
%                 .Galileo.prn    [nx1] Galileo PRNs visible this epoch
%                 (other constellations present but not authenticated by OSNMA)
%   cfg         config struct with fields:
%                 cfg.stage0.mode        'simulation' (only supported mode)
%                 cfg.stage0.classify    struct from classify_spoofed_sats
%                                        (required in simulation mode)
%
% OUTPUTS
%   osnma_result  struct with fields:
%     .mode            char — 'simulation' (always, until HPG 1.50 upgrade)
%     .sat_status      struct array, one per Galileo satellite:
%       .prn           double
%       .auth_status   char — 'AUTH_OK' | 'AUTH_UNKNOWN' | 'AUTH_FAIL'
%       .auth_code     uint8 — 0=OK, 1=UNKNOWN, 2=FAIL (matches UBX-SEC-OSNMA
%                              authStatus field definition in HPG 1.50)
%     .n_authenticated double — count of AUTH_OK satellites
%     .n_unknown       double — count of AUTH_UNKNOWN satellites
%     .n_failed        double — count of AUTH_FAIL satellites
%     .osnma_alert     logical — true if any Galileo satellite returns AUTH_FAIL
%     .hw_note         char — scope limitation statement for logging
%
% PIPELINE INTEGRATION:
%   osnma_result feeds Stage 1 (combine_detectors) and Stage 2
%   (classify_spoofed_sats) as an additional input channel.
%   Satellites with AUTH_FAIL should be pre-classified as 'spoofed'
%   before RAIM-FDE and inter-constellation checks run.
%   This is the integration point described in Chapter 4, Section 4.1.
%
% STAGE:    0 — OSNMA Cryptographic Authentication

    %% --- Validate mode -------------------------------------------------------
    if ~isfield(cfg, 'stage0')
        cfg.stage0 = struct();
    end
    if ~isfield(cfg.stage0, 'mode')
        cfg.stage0.mode = 'simulation';
    end

    if ~strcmp(cfg.stage0.mode, 'simulation')
        error(['osnma_status: only ''simulation'' mode is supported. ' ...
               'Hardware mode requires ZED-F9P firmware HPG 1.50 or later. ' ...
               'Current hardware runs HPG 1.32 which does not support OSNMA. ' ...
               'See thesis Section 4.1 for upgrade path.']);
    end

    hw_note = ['SIMULATION MODE: ZED-F9P HPG 1.32 does not support OSNMA. ' ...
               'Auth status derived from Stage 2 classification, not cryptographic ' ...
               'verification. Upgrade to HPG 1.50 for real OSNMA output.'];

    %% --- Build Galileo satellite list ----------------------------------------
    if ~isfield(obs_epoch, 'Galileo') || isempty(obs_epoch.Galileo.prn)
        % No Galileo satellites visible — OSNMA cannot authenticate anything.
        osnma_result.mode            = 'simulation';
        osnma_result.sat_status      = struct('prn',{},'auth_status',{},'auth_code',{});
        osnma_result.n_authenticated = 0;
        osnma_result.n_unknown       = 0;
        osnma_result.n_failed        = 0;
        osnma_result.osnma_alert     = false;
        osnma_result.hw_note         = hw_note;
        return
    end

    gal_prns = obs_epoch.Galileo.prn(:);

    %% --- Build classification lookup from Stage 2 output --------------------
    % In simulation mode, Stage 2 classify_result must be provided.
    if ~isfield(cfg.stage0, 'classify') || isempty(cfg.stage0.classify)
        % No classification available — default all to AUTH_UNKNOWN.
        % This models receiver cold-start or first-epoch behaviour.
        sat_status = build_unknown_status(gal_prns);
        osnma_result = package_result(sat_status, 'simulation', hw_note);
        return
    end

    classify = cfg.stage0.classify;

    % Build lookup map: Galileo PRN -> trust status from Stage 2.
    status_map = containers.Map('KeyType','double','ValueType','char');
    for k = 1:numel(classify.sat_list)
        s = classify.sat_list(k);
        if strcmp(s.constellation, 'Galileo')
            status_map(s.prn) = s.status;
        end
    end

    %% --- Map trust classification to OSNMA auth status ----------------------
    % AUTH code values match the authStatus field encoding in the
    % UBX-SEC-OSNMA message defined for HPG 1.50:
    %   0 = authenticated (AUTH_OK)
    %   1 = authentication status unknown (AUTH_UNKNOWN)
    %   2 = authentication failed (AUTH_FAIL)
    % Source: u-blox HPG 1.50 Interface Description, Section 3.17.1,
    % UBX-SEC-OSNMA (0x27 0x05).  Field: authStatus, type U1.

    sat_status = struct('prn',{},'auth_status',{},'auth_code',{});

    for k = 1:numel(gal_prns)
        prn = gal_prns(k);
        entry.prn = prn;

        if isKey(status_map, prn)
            trust = status_map(prn);
        else
            trust = 'trusted';  % not in classifier output: conservative default
        end

        switch trust
            case 'trusted'
                entry.auth_status = 'AUTH_OK';
                entry.auth_code   = uint8(0);
            case 'suspect'
                entry.auth_status = 'AUTH_UNKNOWN';
                entry.auth_code   = uint8(1);
            case 'spoofed'
                entry.auth_status = 'AUTH_FAIL';
                entry.auth_code   = uint8(2);
            otherwise
                entry.auth_status = 'AUTH_UNKNOWN';
                entry.auth_code   = uint8(1);
        end

        sat_status(end+1) = entry; %#ok<AGROW>
    end

    osnma_result = package_result(sat_status, 'simulation', hw_note);

end

%% ============================================================================
%  LOCAL HELPERS
%% ============================================================================

function sat_status = build_unknown_status(gal_prns)
% BUILD_UNKNOWN_STATUS  Returns AUTH_UNKNOWN for all Galileo PRNs.
%   Models receiver cold-start before any authentication has completed.
    sat_status = struct('prn',{},'auth_status',{},'auth_code',{});
    for k = 1:numel(gal_prns)
        entry.prn         = gal_prns(k);
        entry.auth_status = 'AUTH_UNKNOWN';
        entry.auth_code   = uint8(1);
        sat_status(end+1) = entry; %#ok<AGROW>
    end
end

function result = package_result(sat_status, mode, hw_note)
% PACKAGE_RESULT  Assembles the output struct and computes summary counts.
    codes = uint8([sat_status.auth_code]);
    result.mode            = mode;
    result.sat_status      = sat_status;
    result.n_authenticated = sum(codes == 0);
    result.n_unknown       = sum(codes == 1);
    result.n_failed        = sum(codes == 2);
    result.osnma_alert     = any(codes == 2);
    result.hw_note         = hw_note;
end
