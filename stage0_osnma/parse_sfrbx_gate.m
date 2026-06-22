function result = parse_sfrbx_gate(ubx_filepath, n_pages)
% PARSE_SFRBX_GATE  Day-one gate: confirm OSNMA bits are non-zero in
%                   Galileo E1-B SFRBX pages from a ZED-F9P HPG 1.32 log.
%
% PURPOSE — MUST RUN BEFORE ANY OSNMA CRYPTO IMPLEMENTATION:
%   HPG 1.32 passes raw I/NAV bits through SFRBX verbatim, but some
%   receiver/firmware combinations zero-fill the OSNMA field in the
%   reserved bits even while correctly passing navigation data. If the
%   OSNMA field is all zeros, the full implementation (tesla_key_chain,
%   mac_verify, osnma_verify) cannot be validated with this capture and
%   a new capture is required.
%
% TWO-LAYER PARSE:
%   Layer (a): UBX binary -> SFRBX message payload (this file).
%              Protocol: u-blox F9 HPG 1.32 / PROTVER 27.31.
%              UBX frame: SYNC(2) CLASS(1) ID(1) LEN(2) PAYLOAD(N) CK(2)
%              SFRBX: class=0x02, id=0x13
%              Payload: gnssId(1) svId(1) reserved(1) freqId(1)
%                       numWords(1) chn(1) version(1) reserved(1)
%                       dwrd[numWords] (4 bytes each, little-endian uint32)
%   Layer (b): raw I/NAV page bits -> OSNMA field location.
%              Source: Galileo OS SIS ICD [AD.1] + EUSPA OSNMA SIS ICD [AD.2].
%              A Galileo E1-B page = 250 bits total per page part, but
%              SFRBX carries the 8 32-bit data words = 256 bits of the
%              I/NAV page body.  The OSNMA field occupies bits [114:153]
%              (0-indexed from MSB, 40 bits) in the EVEN page part of
%              certain I/NAV word types.
%
% *** IMPORTANT: layer (b) bit offsets are NOT verified from the u-blox
%     IDD in this file (the IDD is not in the project file set).  This
%     function therefore PRINTS THE RAW HEX WORDS for manual inspection
%     and performs a HEURISTIC check (are any bits in positions
%     approximately consistent with the OSNMA field non-zero?).
%     The heuristic is: extract bits [112:151] (40-bit window centred on
%     the OSNMA position per Galileo OS SIS ICD Table 2) from the first
%     data word pair and check for non-zero.  This is sufficient for the
%     day-one gate; the full bit-exact extraction is in parse_inav_page.m
%     which requires the u-blox IDD bit layout to be confirmed first.
%
% INPUTS
%   ubx_filepath  char — full path to the .ubx binary log file
%   n_pages       double — number of Galileo E1-B pages to inspect
%                 (default: 20; use a small number for speed)
%
% OUTPUTS
%   result  struct:
%     .n_gal_sfrbx        total Galileo SFRBX messages found
%     .n_e1b              subset with sigId consistent with E1-B
%     .n_nonzero_osnma    pages where extracted OSNMA candidate bits != 0
%     .gate_pass          logical — true if n_nonzero_osnma >= 1
%     .pages              struct array of parsed pages (up to n_pages)
%     .raw_hex_sample     cell — hex strings of first 5 pages' dwrd arrays
%
% HOW TO INTERPRET THE RESULT:
%   gate_pass = true  -> OSNMA bits appear present; proceed with implementation
%   gate_pass = false -> OSNMA bits appear zero; check firmware config or
%                        recapture with OSNMA explicitly enabled in receiver
%
% SOURCE REFERENCES:
%   [UBX-IDD]  u-blox F9 HPG 1.32 Interface Description, PROTVER 27.31
%              UBX-RXM-SFRBX (class 0x02, id 0x13)
%   [GAL-ICD]  Galileo OS SIS ICD, Issue 2.1 — I/NAV page structure
%   [OSNMA-ICD] EUSPA OSNMA SIS ICD v1.1 — OSNMA field in I/NAV
%
% PROJECT:  GNSS Thesis — Stage 0 OSNMA Implementation
% AUTHOR:   RG

    if nargin < 2, n_pages = 20; end

    UBX_SYNC1  = uint8(0xB5);
    UBX_SYNC2  = uint8(0x62);
    SFRBX_CLASS = uint8(0x02);
    SFRBX_ID    = uint8(0x13);
    GAL_GNSSID  = uint8(2);    % gnssId=2 for Galileo (confirmed from log)

    result.n_gal_sfrbx     = 0;
    result.n_e1b            = 0;
    result.n_nonzero_osnma  = 0;
    result.gate_pass        = false;
    result.pages            = struct('svId',{},'sigId',{},'numWords',{},...
                                     'dwrd',{},'osnma_candidate_bits',{},...
                                     'osnma_nonzero',{});
    result.raw_hex_sample   = {};

    %% --- Open file ---------------------------------------------------------
    fid = fopen(ubx_filepath, 'rb');
    if fid < 0
        error('parse_sfrbx_gate: cannot open file: %s', ubx_filepath);
    end
    cleanup = onCleanup(@() fclose(fid));

    fprintf('parse_sfrbx_gate: scanning %s\n', ubx_filepath);
    fprintf('Looking for UBX-RXM-SFRBX (0x02 0x13) with gnssId=2 (Galileo)\n\n');

    %% --- Scan UBX stream ---------------------------------------------------
    n_collected = 0;
    byte_pos    = 0;

    while ~feof(fid) && n_collected < n_pages
        % Find sync bytes
        b1 = fread(fid, 1, 'uint8');
        if isempty(b1), break; end
        byte_pos = byte_pos + 1;

        if b1 ~= UBX_SYNC1, continue; end

        b2 = fread(fid, 1, 'uint8');
        if isempty(b2), break; end
        byte_pos = byte_pos + 1;
        if b2 ~= UBX_SYNC2, continue; end

        % Read class, id, length
        hdr = fread(fid, 4, 'uint8');
        if numel(hdr) < 4, break; end
        byte_pos = byte_pos + 4;

        msg_class = uint8(hdr(1));
        msg_id    = uint8(hdr(2));
        msg_len   = uint16(hdr(3)) + uint16(hdr(4)) * 256;

        % Read payload + checksum
        payload = fread(fid, msg_len + 2, 'uint8');
        byte_pos = byte_pos + msg_len + 2;
        if numel(payload) < msg_len + 2, break; end

        % Only process SFRBX messages
        if msg_class ~= SFRBX_CLASS || msg_id ~= SFRBX_ID
            continue
        end

        % Parse SFRBX header (8 bytes)
        if msg_len < 8, continue; end
        gnssId   = uint8(payload(1));
        svId     = uint8(payload(2));
        % reserved1 = payload(3)
        freqId   = uint8(payload(4));
        numWords = uint8(payload(5));
        % chn      = payload(6)
        % version  = payload(7)
        % reserved2= payload(8)

        % Only process Galileo
        if gnssId ~= GAL_GNSSID, continue; end
        result.n_gal_sfrbx = result.n_gal_sfrbx + 1;

        % Read data words (little-endian uint32, 4 bytes each)
        if msg_len < 8 + numWords * 4, continue; end
        dwrd = zeros(numWords, 1, 'uint32');
        for w = 1:numWords
            base = 8 + (w-1)*4 + 1;
            dwrd(w) = uint32(payload(base)) + ...
                      uint32(payload(base+1)) * 256 + ...
                      uint32(payload(base+2)) * 65536 + ...
                      uint32(payload(base+3)) * 16777216;
        end

        % Determine sigId heuristically from numWords and freqId
        % Per u-blox PROTVER 27.31: Galileo E1-B has numWords=8, freqId=0
        % E5b has numWords=8, freqId=1.
        % NOTE: exact sigId field position in the SFRBX header varies by
        % PROTVER; in some versions it is the reserved1 byte (payload(3)).
        % We use numWords==8 && freqId==0 as the E1-B heuristic.
        % THIS MUST BE VERIFIED AGAINST THE ACTUAL IDD BEFORE PRODUCTION USE.
        is_e1b = (numWords == 8 && freqId == 0);

        if ~is_e1b, continue; end
        result.n_e1b = result.n_e1b + 1;

        % Extract bit representation of all dwrd (MSB first per Galileo ICD)
        all_bits = '';
        for w = 1:numWords
            all_bits = [all_bits, dec2bin(dwrd(w), 32)]; %#ok<AGROW>
        end
        % all_bits is now a 256-char string (8 words * 32 bits)

        % OSNMA field heuristic: Galileo OS SIS ICD specifies that in the
        % E1-B EVEN page part, the OSNMA reserved field occupies bits
        % starting at bit offset 114 (0-indexed from MSB of the page body).
        % The page body in SFRBX starts at bit 0 of word 1.
        % 40 OSNMA bits: bits [114:153] in the 256-bit SFRBX payload.
        % NOTE: This offset is APPROXIMATE pending IDD verification.
        % The heuristic window [112:155] (44 bits) gives margin for ±2 bit
        % alignment uncertainty.
        if length(all_bits) >= 156
            osnma_candidate = all_bits(113:156);  % 1-indexed: bits 112-155
        else
            osnma_candidate = repmat('0', 1, 44);
        end
        osnma_nonzero = any(osnma_candidate == '1');

        if osnma_nonzero
            result.n_nonzero_osnma = result.n_nonzero_osnma + 1;
        end

        % Store page using struct() to guarantee field order matches preallocated array
        n_collected = n_collected + 1;
        result.pages(n_collected) = struct( ...
            'svId',                 svId, ...
            'sigId',                freqId, ...
            'numWords',             numWords, ...
            'dwrd',                 {dwrd}, ...
            'osnma_candidate_bits', osnma_candidate, ...
            'osnma_nonzero',        osnma_nonzero);

        % Build hex string for sample
        hex_str = '';
        for w = 1:numWords
            hex_str = [hex_str, sprintf('%08X ', dwrd(w))]; %#ok<AGROW>
        end
        if numel(result.raw_hex_sample) < 5
            result.raw_hex_sample{end+1} = sprintf('SVid=%d | %s', svId, strtrim(hex_str));
        end
    end

    %% --- Gate decision ---------------------------------------------------
    result.gate_pass = (result.n_nonzero_osnma >= 1);

    %% --- Report ----------------------------------------------------------
    fprintf('=== SFRBX GATE CHECK RESULTS ===\n');
    fprintf('Total Galileo SFRBX messages found: %d\n', result.n_gal_sfrbx);
    fprintf('E1-B pages (numWords=8, freqId=0):  %d\n', result.n_e1b);
    fprintf('Pages inspected (first %d):          %d\n', n_pages, n_collected);
    fprintf('Pages with non-zero OSNMA candidate: %d\n', result.n_nonzero_osnma);
    fprintf('\n');

    if result.gate_pass
        fprintf('GATE: PASS -- OSNMA bits appear present in E1-B pages.\n');
        fprintf('Proceed with parse_inav_page.m to verify exact bit offsets,\n');
        fprintf('then implement tesla_key_chain.m and mac_verify.m.\n');
    else
        fprintf('GATE: FAIL -- OSNMA candidate bits are all zero.\n');
        fprintf('Possible causes:\n');
        fprintf('  1. Receiver firmware zeros OSNMA field (check HPG 1.32 behaviour)\n');
        fprintf('  2. OSNMA not yet enabled (CFG-GAL-USE_OSNMA was set mid-session)\n');
        fprintf('  3. Bit offset heuristic is wrong -- inspect raw_hex_sample manually\n');
        fprintf('  4. Only SFRBX pages from before OSNMA enablement were parsed\n');
        fprintf('     (try n_pages=200 to reach later pages in the file)\n');
    end

    fprintf('\nRaw hex sample (first 5 E1-B pages, 8 words each):\n');
    for k = 1:numel(result.raw_hex_sample)
        fprintf('  Page %d: %s\n', k, result.raw_hex_sample{k});
    end
    fprintf('\nNOTE: OSNMA bit offset [112:155] is a heuristic pending IDD\n');
    fprintf('      verification. Inspect raw hex words manually to confirm.\n');
    fprintf('      Each word is little-endian uint32 as stored by u-blox SFRBX.\n');
end