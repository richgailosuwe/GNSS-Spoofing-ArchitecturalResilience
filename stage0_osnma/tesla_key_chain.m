function [verified, key_verified] = tesla_key_chain(K_I, GST_SF, alpha, K_J, key_size_bits)
% TESLA_KEY_CHAIN  Verify a TESLA chain key against a previously verified key.
%
% The TESLA key chain is a one-way hash chain where each key K_I is derived
% from the next key K_{I+1} by hashing. Verification goes backward: given a
% received key K_I and a verified key K_J (J > I, i.e. K_J was disclosed
% AFTER K_I), compute K_I from K_J by hashing backward (J-I) times, then
% compare.
%
% TESLA KEY CHAIN VERIFICATION (per EUSPA OSNMA Receiver Guidelines v1.3,
% Section 5.4, Table 6):
%
%   For each step from j=J down to j=I+1:
%     message = K_j || GST_SF(j) || alpha
%     K_{j-1} = truncate(H(message), key_size_bits)
%
%   Where:
%     K_j      = current chain key (key_size_bits long)
%     GST_SF(j) = Galileo System Time at sub-frame j, format:
%                  WN (12 bits) || TOW (20 bits) = 32 bits total
%     alpha    = unpredictable chain pattern (key_size_bits long)
%     H        = hash function specified in DSM-KROOT HF field
%                (SHA-256 for PKID=2, ECDSA P-256/SHA-256 chain)
%     truncate = take the most significant key_size_bits bits
%
% This function implements ONE step of the chain (I+1 -> I).
% Call iteratively for multi-step verification.
%
% SOURCE: EUSPA OSNMA Receiver Guidelines v1.3, Section 5.4, Table 6.
%         Hash function: SHA-256 (java.security.MessageDigest, confirmed
%         available in this MATLAB environment via crypto backend check).
%
% CRYPTO BACKEND: Java (java.security.MessageDigest 'SHA-256').
%   MATLAB hash() function is NOT available in this environment.
%   Java SHA-256 confirmed available (first byte of SHA-256([1,2,3,4]) = 0x9F).
%
% INPUTS
%   K_I           uint8 [key_size_bytes x 1] — received TESLA key to verify
%   GST_SF        uint32 — GST of sub-frame as 32-bit value:
%                   bits [0:11]  = WN  (Galileo week number, 12 bits)
%                   bits [12:31] = TOW (time of week in seconds, 20 bits)
%                   Format: (WN << 20) | TOW
%   alpha         uint8 [key_size_bytes x 1] — chain pattern from DSM-KROOT
%   K_J           uint8 [key_size_bytes x 1] — previously verified key (J=I+1)
%                   This is either KROOT (the root key from DSM-KROOT,
%                   verified via ECDSA) or a previously verified chain key.
%   key_size_bits double — TESLA key size in bits (from DSM-KROOT KS field)
%                   Typical values: 96, 104, 112, 120, 128 bits
%
% OUTPUTS
%   verified      logical — true if K_I matches the computed value from K_J
%   key_verified  uint8 [key_size_bytes x 1] — computed K_I from K_J
%                   (use this as K_J input for the next chain step)
%
% USAGE EXAMPLE (verify K_2 against K_3):
%   [ok, K2_comp] = tesla_key_chain(K2_received, GST_SF_3, alpha, K3_verified, 128);
%   if ok, disp('K2 verified'); end
%
% PROJECT:  GNSS Thesis — Stage 0 OSNMA
% AUTHOR:   RG

    key_size_bytes = ceil(key_size_bits / 8);

    %% --- Validate inputs --------------------------------------------------
    if numel(K_I) ~= key_size_bytes
        error('tesla_key_chain: K_I length %d does not match key_size_bits=%d (expect %d bytes)', ...
            numel(K_I), key_size_bits, key_size_bytes);
    end
    if numel(K_J) ~= key_size_bytes
        error('tesla_key_chain: K_J length %d does not match key_size_bits=%d (expect %d bytes)', ...
            numel(K_J), key_size_bits, key_size_bytes);
    end
    % alpha length is NOT validated against key_size_bytes.
    % Per OSNMA SIS ICD v1.1, alpha is a fixed-length field defined in
    % DSM-KROOT, and its length may differ from the key size in some
    % configurations (e.g. Annex A test vectors use 6-byte alpha with
    % 16-byte keys). The message K_J || GST_SF || alpha is concatenated
    % as-is; the ICD defines the correct length for each configuration.

    %% --- Build message: K_J || GST_SF || alpha ----------------------------
    % GST_SF as 4-byte big-endian (MSB first per OSNMA ICD bit ordering)
    gst_bytes = uint8([
        bitand(bitshift(uint32(GST_SF), -24), uint32(255));
        bitand(bitshift(uint32(GST_SF), -16), uint32(255));
        bitand(bitshift(uint32(GST_SF),  -8), uint32(255));
        bitand(uint32(GST_SF), uint32(255))
    ]);

    message = [K_J(:); gst_bytes(:); alpha(:)];

    %% --- SHA-256 via Java (MATLAB hash() not available) ------------------
    % Source: java.security.MessageDigest, confirmed available in this env.
    try
        md = java.security.MessageDigest.getInstance('SHA-256');
        md.update(typecast(message, 'int8'));
        digest_java = md.digest();
        % Convert Java byte array to MATLAB uint8
        digest = uint8(typecast(digest_java, 'uint8'));
        if numel(digest) ~= 32
            % Java byte[] signed -> unsigned conversion
            digest = uint8(mod(int32(digest_java) + 256, 256));
        end
    catch e
        error('tesla_key_chain: Java SHA-256 failed: %s', e.message);
    end

    %% --- Truncate to key_size_bits (take MSB) ----------------------------
    % Truncation: keep the most significant key_size_bits bits of the
    % 256-bit SHA-256 digest. Since digest is byte-aligned and key_size_bits
    % is always a multiple of 8 in current OSNMA configurations, this is
    % simply the first key_size_bytes bytes.
    if mod(key_size_bits, 8) ~= 0
        % Non-byte-aligned truncation: zero out LSBs of last byte
        n_full_bytes  = floor(key_size_bits / 8);
        n_extra_bits  = mod(key_size_bits, 8);
        mask          = uint8(bitshift(uint8(255), -(8 - n_extra_bits)));
        key_verified  = [digest(1:n_full_bytes); bitand(digest(n_full_bytes+1), mask)];
    else
        key_verified = digest(1:key_size_bytes);
    end

    %% --- Compare with received K_I ----------------------------------------
    verified = isequal(key_verified, K_I(:));

end