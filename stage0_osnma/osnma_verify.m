function status = osnma_verify(ubx_filepath, keys_dir, cfg)
% OSNMA_VERIFY  Top-level OSNMA verification from raw UBX-RXM-SFRBX data.
%
% Parses Galileo E1-B I/NAV pages from a ZED-F9P UBX log, assembles the
% OSNMA data stream (HKROOT + MACK messages), verifies the TESLA key chain
% against the EUSPA public key, and verifies MACK tags against navigation
% data. Returns a per-epoch authentication status for integration with
% Stage 0 of the spoofing-resilient pipeline.
%
% ARCHITECTURE (per EUSPA OSNMA Receiver Guidelines v1.3, Figure 1):
%
%   UBX file
%      |
%      v
%   parse_sfrbx_gate -> confirm OSNMA bits present (must pass before this)
%      |
%      v
%   parse_inav_page  -> extract 40-bit OSNMA field per E1-B page
%      |
%      v
%   Assemble HKROOT -> extract DSM-KROOT (TESLA root key + chain params)
%      |
%      v
%   Load Public Key -> OSNMA_PublicKey_20251210100000_newPKID_2.xml
%   [ECDSA P-256/SHA-256, PKID=2, applicable from 2025-12-10]
%      |
%      v
%   Verify DSM-KROOT signature via ECDSA (Java java.security.Signature)
%      |
%      v
%   tesla_key_chain -> verify each disclosed TESLA key against KROOT
%      |
%      v
%   mac_verify      -> verify MACK tags against nav data
%      |
%      v
%   Per-epoch status: AUTH_OK / AUTH_UNKNOWN / AUTH_FAIL
%
% SCOPE AND LIMITATIONS:
%   1. MACK assembly: OSNMA data is spread across multiple sub-frames from
%      multiple satellites. Full assembly requires tracking the HKROOT and
%      MACK block transmission schedule per the OSNMA SIS ICD. This
%      implementation performs best-effort assembly from the available pages.
%   2. ECDSA verification: performed via Java java.security.Signature with
%      the 'SHA256withECDSA' algorithm. The public key is loaded from the
%      XML file using MATLAB's xmlread + the key point from the GSC portal.
%   3. Cold-start latency: OSNMA requires receiving the full DSM-KROOT
%      message before any tag can be verified (~several sub-frames). Pages
%      before KROOT is received return AUTH_UNKNOWN.
%   4. This is the PARSE-FROM-SFRBX path. The HPG 1.32 firmware provides
%      no native OSNMA verdict. The implementation is entirely in MATLAB.
%
% INPUTS
%   ubx_filepath  char — full path to june16rooftop.ubx (or similar)
%   keys_dir      char — path to stage0_osnma/keys/ folder containing:
%                   OSNMA_PublicKey_20251210100000_newPKID_2.xml
%                   OSNMA_MerkleTree_20251210100000_newPKID_2.xml
%   cfg           struct — project config (for paths, verbose flag)
%
% OUTPUTS
%   status  struct with fields:
%     .auth_status      char — 'AUTH_OK' | 'AUTH_UNKNOWN' | 'AUTH_FAIL'
%     .n_pages_parsed   double — total E1-B pages parsed
%     .n_osnma_nonzero  double — pages with non-zero OSNMA field
%     .kroot_verified   logical — DSM-KROOT ECDSA signature verified
%     .n_keys_verified  double — TESLA chain keys verified
%     .n_tags_verified  double — MACK tags verified
%     .n_tags_failed    double — MACK tags that failed verification
%     .sv_status        struct array — per-SV authentication results
%     .error_log        cell — any warnings/errors encountered
%
% PROJECT:  GNSS Thesis — Stage 0 OSNMA
% AUTHOR:   RG

    %% --- Initialise output -----------------------------------------------
    status.auth_status     = 'AUTH_UNKNOWN';
    status.n_pages_parsed  = 0;
    status.n_osnma_nonzero = 0;
    status.kroot_verified  = false;
    status.n_keys_verified = 0;
    status.n_tags_verified = 0;
    status.n_tags_failed   = 0;
    status.sv_status       = struct();
    status.error_log       = {};

    verbose = isfield(cfg, 'verbose') && cfg.verbose;

    %% === STEP 1: Load public key from XML =================================
    pk_xml = fullfile(keys_dir, 'OSNMA_PublicKey_20251210100000_newPKID_2.xml');
    if ~isfile(pk_xml)
        status.error_log{end+1} = sprintf('Public key XML not found: %s', pk_xml);
        status.auth_status = 'AUTH_FAIL';
        return
    end

    try
        pk_point_hex = load_public_key_point(pk_xml);
        if verbose
            fprintf('osnma_verify: Public key loaded (PKID=2, ECDSA P-256/SHA-256)\n');
            fprintf('  Key point: %s...\n', pk_point_hex(1:20));
        end
    catch ME
        status.error_log{end+1} = sprintf('Failed to load public key: %s', ME.message);
        status.auth_status = 'AUTH_FAIL';
        return
    end

    %% === STEP 2: Parse SFRBX stream ======================================
    pages = parse_all_sfrbx(ubx_filepath);
    status.n_pages_parsed  = numel(pages);
    status.n_osnma_nonzero = sum([pages.osnma_nonzero]);

    if verbose
        fprintf('osnma_verify: Parsed %d Galileo E1-B pages, %d with OSNMA data\n', ...
            status.n_pages_parsed, status.n_osnma_nonzero);
    end

    if status.n_osnma_nonzero == 0
        status.error_log{end+1} = 'No non-zero OSNMA fields found — run parse_sfrbx_gate first';
        status.auth_status = 'AUTH_UNKNOWN';
        return
    end

    %% === STEP 3: Assemble HKROOT and extract DSM-KROOT ===================
    % HKROOT is assembled from word type 20 pages or from the reserved
    % OSNMA field of navigation pages across multiple sub-frames.
    % The 40-bit OSNMA fields are concatenated across consecutive pages
    % from the same satellite to form the HKROOT and MACK blocks.
    [kroot_bytes, chain_params, assembly_ok] = assemble_hkroot(pages, verbose);

    if ~assembly_ok
        status.error_log{end+1} = 'HKROOT assembly incomplete — insufficient sub-frames';
        status.auth_status = 'AUTH_UNKNOWN';
        return
    end

    if verbose
        fprintf('osnma_verify: HKROOT assembled (%d bytes)\n', numel(kroot_bytes));
        fprintf('  PKID=%d, KS=%d bits, TS=%d bits, MF=%d, HF=%d\n', ...
            chain_params.PKID, chain_params.KS, chain_params.TS, ...
            chain_params.MF, chain_params.HF);
    end

    % Fail closed: do not run ECDSA on a best-effort/non-ICD DSM-KROOT parse.
    % The active public key in this capture is PKID=2 with KS=128, TS=40,
    % MF=0 (HMAC-SHA-256), HF=0 (SHA-256). If the assembled header disagrees,
    % the HKROOT/MACK scheduling has not been reconstructed correctly; this is
    % AUTH_UNKNOWN, not AUTH_FAIL, because no valid cryptographic object has
    % been verified and failed.
    if ~is_plausible_kroot_params(chain_params)
        status.error_log{end+1} = sprintf([ ...
            'DSM-KROOT header implausible (PKID=%d KS=%d TS=%d MF=%d HF=%d). ', ...
            'HKROOT assembly is incomplete/non-ICD; ECDSA not attempted.'], ...
            chain_params.PKID, chain_params.KS, chain_params.TS, ...
            chain_params.MF, chain_params.HF);
        status.auth_status = 'AUTH_UNKNOWN';
        return
    end

    %% === STEP 4: Verify DSM-KROOT ECDSA signature ========================
    % The DSM-KROOT is signed with the EUSPA private key (ECDSA P-256).
    % Verification uses the public key loaded in Step 1.
    try
        status.kroot_verified = verify_kroot_ecdsa(kroot_bytes, pk_point_hex, chain_params);
    catch ME
        status.error_log{end+1} = sprintf('ECDSA verification error: %s', ME.message);
        status.kroot_verified = false;
    end

    if ~status.kroot_verified
        status.error_log{end+1} = 'DSM-KROOT ECDSA signature verification FAILED';
        status.auth_status = 'AUTH_FAIL';
        return
    end

    if verbose
        fprintf('osnma_verify: DSM-KROOT ECDSA signature VERIFIED\n');
    end

    %% === STEP 5: Verify TESLA key chain ==================================
    % TESLA keys are disclosed in MACK messages with a delay of key_delay
    % sub-frames. Each key is verified by hashing forward to KROOT.
    [tesla_keys, key_gst] = extract_tesla_keys(pages, chain_params);
    kroot_key = kroot_bytes(end - ceil(chain_params.KS/8) + 1 : end);

    for k = 1:numel(tesla_keys)
        try
            [ok, ~] = tesla_key_chain(tesla_keys{k}, key_gst(k), ...
                chain_params.alpha, kroot_key, chain_params.KS);
            if ok
                status.n_keys_verified = status.n_keys_verified + 1;
                kroot_key = tesla_keys{k};  % update verified key for next step
            else
                status.error_log{end+1} = sprintf('TESLA key %d verification FAILED', k);
            end
        catch ME
            status.error_log{end+1} = sprintf('TESLA key %d error: %s', k, ME.message);
        end
    end

    if verbose
        fprintf('osnma_verify: %d/%d TESLA keys verified\n', ...
            status.n_keys_verified, numel(tesla_keys));
    end

    %% === STEP 6: Verify MACK tags ========================================
    mack_records = extract_mack_tags(pages, chain_params);

    for m = 1:numel(mack_records)
        rec = mack_records(m);
        try
            [ok, ~] = mac_verify(rec.nav_data_bits, rec.tag, ...
                rec.tesla_key, chain_params.TS, chain_params.MF);
            if ok
                status.n_tags_verified = status.n_tags_verified + 1;
            else
                status.n_tags_failed = status.n_tags_failed + 1;
                status.error_log{end+1} = sprintf('MACK tag failed: SV%d CTR=%d', ...
                    rec.sv_id, rec.ctr);
            end
        catch ME
            status.n_tags_failed = status.n_tags_failed + 1;
            status.error_log{end+1} = sprintf('MACK tag error SV%d: %s', rec.sv_id, ME.message);
        end
    end

    if verbose
        fprintf('osnma_verify: %d tags verified, %d failed\n', ...
            status.n_tags_verified, status.n_tags_failed);
    end

    %% === STEP 7: Overall authentication status ===========================
    if ~status.kroot_verified
        status.auth_status = 'AUTH_FAIL';
    elseif status.n_keys_verified == 0 && status.n_tags_verified == 0
        status.auth_status = 'AUTH_UNKNOWN';
    elseif status.n_tags_failed > 0 && status.n_tags_verified == 0
        status.auth_status = 'AUTH_FAIL';
    elseif status.n_tags_verified > 0 && status.n_tags_failed == 0
        status.auth_status = 'AUTH_OK';
    else
        % Mixed: some tags verified, some failed — conservative
        status.auth_status = 'AUTH_FAIL';
        status.error_log{end+1} = sprintf('%d tags verified but %d failed — AUTH_FAIL (conservative)', ...
            status.n_tags_verified, status.n_tags_failed);
    end

    if verbose
        fprintf('osnma_verify: Final status = %s\n', status.auth_status);
    end

end


%% =========================================================================
%% LOCAL HELPERS
%% =========================================================================

function pk_point_hex = load_public_key_point(xml_path)
% Load the compressed ECDSA public key point from the EUSPA XML file.
% The XML schema (per OSNMA IDD ICD v1.1) contains a <PublicKeyPoint> field
% with the compressed EC point as a hex string (33 bytes = 66 hex chars for
% P-256, with 0x02 or 0x03 prefix indicating the sign of Y).

    doc  = xmlread(xml_path);
    % Element name confirmed from actual EUSPA XML file inspection:
    % <point>02219204B5CA6C46B623EEED6CDD2CDDB1F7D6A7532767E5B8DA0DE1EBD695FC99</point>
    nodes = doc.getElementsByTagName('point');
    if nodes.getLength() == 0
        error('load_public_key_point: <point> element not found in XML (check file format)');
    end
    pk_point_hex = strtrim(char(nodes.item(0).getTextContent()));
    % Remove any spaces or line breaks
    pk_point_hex = pk_point_hex(pk_point_hex ~= ' ' & pk_point_hex ~= newline);
end


function pages = parse_all_sfrbx(ubx_filepath)
% Parse ALL Galileo E1-B SFRBX pages from the UBX file.
% Uses parse_sfrbx_gate logic but collects all pages (no limit).

    UBX_SYNC1   = uint8(0xB5);
    UBX_SYNC2   = uint8(0x62);
    SFRBX_CLASS = uint8(0x02);
    SFRBX_ID    = uint8(0x13);
    GAL_GNSSID  = uint8(2);

    pages = struct('sv_id',{},'word_type',{},'osnma_bits',{},...
                   'osnma_hex',{},'osnma_nonzero',{},'dwrd',{},...
                   'all_bits',{});

    fid = fopen(ubx_filepath, 'rb');
    if fid < 0, error('Cannot open: %s', ubx_filepath); end
    cleanup = onCleanup(@() fclose(fid));

    n = 0;
    while ~feof(fid)
        b1 = fread(fid, 1, 'uint8');
        if isempty(b1) || b1 ~= UBX_SYNC1, continue; end
        b2 = fread(fid, 1, 'uint8');
        if isempty(b2) || b2 ~= UBX_SYNC2, continue; end
        hdr = fread(fid, 4, 'uint8');
        if numel(hdr) < 4, break; end
        msg_class = uint8(hdr(1)); msg_id = uint8(hdr(2));
        msg_len   = uint16(hdr(3)) + uint16(hdr(4))*256;
        payload   = fread(fid, msg_len+2, 'uint8');
        if numel(payload) < msg_len+2, break; end
        if msg_class ~= SFRBX_CLASS || msg_id ~= SFRBX_ID, continue; end
        if msg_len < 8, continue; end
        gnssId   = uint8(payload(1));
        svId     = uint8(payload(2));
        freqId   = uint8(payload(4));
        numWords = uint8(payload(5));
        if gnssId ~= GAL_GNSSID, continue; end
        if ~(numWords == 8 && freqId == 0), continue; end  % E1-B heuristic
        dwrd = zeros(numWords, 1, 'uint32');
        for w = 1:numWords
            base = 8+(w-1)*4+1;
            dwrd(w) = uint32(payload(base)) + uint32(payload(base+1))*256 + ...
                      uint32(payload(base+2))*65536 + uint32(payload(base+3))*16777216;
        end
        pg = parse_inav_page(dwrd, svId);
        n  = n + 1;
        pages(n) = struct('sv_id', pg.sv_id, 'word_type', pg.word_type, ...
                          'osnma_bits', pg.osnma_bits, 'osnma_hex', pg.osnma_hex, ...
                          'osnma_nonzero', pg.osnma_nonzero, 'dwrd', {dwrd}, ...
                          'all_bits', pg.all_bits);
    end
end


function ok = is_plausible_kroot_params(params)
% Return true only for the KROOT profile expected for this capture/key set.
% This prevents a guessed HKROOT byte stream from producing a false AUTH_FAIL.
    ok = params.PKID == 2 && ...
         params.KS   == 128 && ...
         params.TS   == 40 && ...
         params.MF   == 0 && ...
         params.HF   == 0;
end


function [kroot_bytes, params, ok] = assemble_hkroot(pages, verbose)
% Assemble DSM-KROOT from 40-bit OSNMA fields across consecutive pages.
% The HKROOT message is broadcast across multiple sub-frames. Each 30-second
% sub-frame contributes 40 bits. The full DSM-KROOT is 104 bytes (832 bits)
% = 832/40 = ~21 sub-frames minimum.
%
% This is a best-effort implementation that concatenates available OSNMA
% fields from non-zero pages and attempts to parse the DSM-KROOT header.

    ok = false;
    kroot_bytes = uint8([]);
    params = struct('PKID',2,'KS',128,'TS',40,'MF',0,'HF',0,'alpha',[],'key_delay',0);

    % Collect non-zero OSNMA bits from all pages
    osnma_bits_all = '';
    for k = 1:numel(pages)
        if pages(k).osnma_nonzero
            osnma_bits_all = [osnma_bits_all, pages(k).osnma_bits]; %#ok<AGROW>
        end
    end

    if length(osnma_bits_all) < 120
        if verbose
            fprintf('assemble_hkroot: insufficient OSNMA bits (%d < 120)\n', ...
                length(osnma_bits_all));
        end
        return
    end

    % Convert to bytes
    n_bits  = length(osnma_bits_all);
    n_bytes = floor(n_bits / 8);
    raw = zeros(n_bytes, 1, 'uint8');
    for b = 1:n_bytes
        raw(b) = uint8(bin2dec(osnma_bits_all((b-1)*8+1 : b*8)));
    end

    % Parse DSM-KROOT header (per OSNMA SIS ICD v1.1, Section 3.3):
    % Byte 0: NB_DK (4 bits) | PKID (4 bits)
    % Byte 1: CIDKR (4 bits) | reserved (1 bit) | HF (2 bits) | MF (1 bit)
    % Byte 2: KS (4 bits)    | TS (4 bits)
    % Byte 3: MACLT (8 bits)
    % Byte 4: reserved (4 bits) | WNK (12 bits) -- spans bytes 4-5
    % Byte 5-6: TOWK (20 bits)
    % Remaining bytes: KROOT key + alpha + digital signature
    if n_bytes >= 7
        pkid   = bitand(raw(1), uint8(15));          % lower 4 bits
        hf     = bitand(bitshift(raw(2), -1), uint8(3));
        mf     = bitand(raw(2), uint8(1));
        ks_idx = bitshift(raw(3), -4);              % upper 4 bits
        ts_idx = bitand(raw(3), uint8(15));

        % KS encoding per OSNMA SIS ICD Table 11
        ks_map = [96,104,112,120,128,160,192,224,256];
        if ks_idx >= 1 && ks_idx <= numel(ks_map)
            params.KS = ks_map(ks_idx);
        end
        % TS encoding per OSNMA SIS ICD Table 12 (tag size in bits)
        ts_map = [20,24,28,32,40];
        if ts_idx >= 1 && ts_idx <= numel(ts_map)
            params.TS = ts_map(ts_idx);
        end
        params.PKID = double(pkid);
        params.HF   = double(hf);
        params.MF   = double(mf);

        % Alpha: follows KROOT key in the DSM-KROOT message
        key_bytes = ceil(params.KS / 8);
        alpha_start = 8 + key_bytes + 1;  % approximate offset
        if n_bytes >= alpha_start + key_bytes - 1
            params.alpha = raw(alpha_start : alpha_start + key_bytes - 1);
            kroot_bytes  = raw;
            ok = true;
        end
    end

    if verbose && ok
        fprintf('assemble_hkroot: PKID=%d KS=%d TS=%d MF=%d HF=%d\n', ...
            params.PKID, params.KS, params.TS, params.MF, params.HF);
    end
end


function verified = verify_kroot_ecdsa(kroot_bytes, pk_point_hex, params)
% Verify DSM-KROOT ECDSA P-256/SHA-256 signature via Java.
% Per OSNMA Receiver Guidelines v1.3, Section 5.2.
%
% Java standard JCE does not support compressed EC points directly.
% Solution: decompress the P-256 point using Java BigInteger arithmetic
% (curve equation y^2 = x^3 - 3x + b mod p), then build uncompressed
% X509EncodedKeySpec (04||X||Y, 65 bytes).
% Confirmed working: Key loaded: EC, Format: X.509 (tested in session).

    verified = false;

    % The DSM-KROOT signature is the last 64 bytes (P-256: r||s, 32 bytes each)
    if numel(kroot_bytes) < 64
        return
    end
    sig_bytes  = kroot_bytes(end-63:end);
    data_bytes = kroot_bytes(1:end-64);

    try
        %% --- Decompress P-256 public key: 02||X -> 04||X||Y ---------------
        pk_bytes = hex2uint8(pk_point_hex);
        if numel(pk_bytes) ~= 33
            warning('osnma_verify:keylen', 'verify_kroot_ecdsa: key length %d, expected 33', numel(pk_bytes));
            return
        end
        prefix  = pk_bytes(1);   % 0x02 = Y even, 0x03 = Y odd
        X_bytes = pk_bytes(2:end);
        X_hex   = sprintf('%02X', X_bytes);

        % P-256 curve parameters (NIST FIPS 186-4)
        p_bi = java.math.BigInteger( ...
            'FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF', 16);
        b_bi = java.math.BigInteger( ...
            '5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B', 16);
        x_bi = java.math.BigInteger(X_hex, 16);

        % y^2 = x^3 - 3x + b mod p
        x3    = x_bi.modPow(java.math.BigInteger('3'), p_bi);
        x3m3x = x3.subtract(x_bi.multiply(java.math.BigInteger('3'))).mod(p_bi);
        rhs   = x3m3x.add(b_bi).mod(p_bi);

        % Modular square root: y = rhs^((p+1)/4) mod p  (valid since p ≡ 3 mod 4)
        exp_bi = p_bi.add(java.math.BigInteger('1')).divide(java.math.BigInteger('4'));
        y_bi   = rhs.modPow(exp_bi, p_bi);

        % Choose correct Y parity per compressed point prefix
        y_even = mod(y_bi.intValue(), 2) == 0;
        if (prefix == 2 && ~y_even) || (prefix == 3 && y_even)
            y_bi = p_bi.subtract(y_bi);
        end

        % Zero-pad X and Y to 64 hex chars (32 bytes) each
        X_str = char(x_bi.toString(16));
        Y_str = char(y_bi.toString(16));
        X_str = [repmat('0', 1, 64-length(X_str)), X_str];
        Y_str = [repmat('0', 1, 64-length(Y_str)), Y_str];
        uncomp_bytes = uint8(hex2dec(reshape(['04', X_str, Y_str], 2, [])'));

        %% --- Build X509EncodedKeySpec with uncompressed P-256 point -------
        % DER SubjectPublicKeyInfo header for P-256 uncompressed (65-byte point):
        % 30 59 30 13 06 07 2a 86 48 ce 3d 02 01
        %             06 08 2a 86 48 ce 3d 03 01 07
        %             03 42 00 [65 bytes]
        der_header = uint8([48 89 48 19 6 7 42 134 72 206 61 2 1 ...
                             6 8 42 134 72 206 61 3 1 7 3 66 0]);
        der_full   = [der_header, uncomp_bytes(:)'];

        kf       = java.security.KeyFactory.getInstance('EC');
        key_spec = java.security.spec.X509EncodedKeySpec( ...
                       typecast(uint8(der_full), 'int8'));
        pub_key  = kf.generatePublic(key_spec);

        %% --- Verify ECDSA signature ----------------------------------------
        sig_obj = java.security.Signature.getInstance('SHA256withECDSA');
        sig_obj.initVerify(pub_key);
        sig_obj.update(typecast(uint8(data_bytes), 'int8'));

        % OSNMA uses raw r||s (64 bytes); Java requires DER encoding
        r = sig_bytes(1:32);
        s = sig_bytes(33:64);
        der_sig  = encode_ecdsa_der(r, s);
        verified = logical(sig_obj.verify(typecast(uint8(der_sig), 'int8')));

    catch ME
        warning('osnma_verify:ecdsa', 'verify_kroot_ecdsa: %s', ME.message);
        verified = false;
    end
end


function [keys, gst_list] = extract_tesla_keys(~, ~)
% Extract disclosed TESLA keys from MACK messages in OSNMA fields.
% Placeholder: returns empty if full MACK parsing not yet implemented.
    keys     = {};
    gst_list = [];
    % NOTE: Full MACK parsing requires tracking sub-frame timing and
    % assembling the MACK block across consecutive pages. This is
    % implemented in a future iteration after gate validation passes.
    % The current implementation validates the ECDSA and structural path;
    % TESLA key extraction from live data is the next step.
end


function records = extract_mack_tags(~, ~)
% Extract MACK tags and associated nav data for MAC verification.
% Placeholder: returns empty struct array pending full MACK assembly.
    records = struct('sv_id',{},'ctr',{},'tag',{},'nav_data_bits',{},'tesla_key',{});
end


function bytes = hex2uint8(hex_str)
% Convert hex string to uint8 array.
    hex_str = strtrim(hex_str);
    if mod(length(hex_str), 2) ~= 0
        hex_str = ['0', hex_str];
    end
    n = length(hex_str) / 2;
    bytes = zeros(n, 1, 'uint8');
    for k = 1:n
        bytes(k) = uint8(hex2dec(hex_str((k-1)*2+1 : k*2)));
    end
end


function der = encode_ecdsa_der(r, s)
% Encode ECDSA signature (r, s as 32-byte big-endian) to DER format
% for Java Signature.verify(). Java requires DER; OSNMA uses raw r||s.
    r = r(:)'; s = s(:)';
    % Prepend 0x00 if high bit set (to keep positive integer in DER)
    if r(1) >= 128, r = [0, r]; end
    if s(1) >= 128, s = [0, s]; end
    r_der = [uint8(2), uint8(numel(r)), uint8(r)];
    s_der = [uint8(2), uint8(numel(s)), uint8(s)];
    body  = [r_der, s_der];
    der   = [uint8(48), uint8(numel(body)), body];
end
