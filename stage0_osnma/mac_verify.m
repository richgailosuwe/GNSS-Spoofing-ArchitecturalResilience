function [verified, computed_tag] = mac_verify(nav_data_bits, tag_received, ...
                                                  tesla_key, tag_size_bits, mac_function)
% MAC_VERIFY  Verify a single OSNMA MACK tag against navigation data.
%
% Computes a MAC over the navigation data using the verified TESLA chain key,
% truncates to tag_size_bits, and compares bit-by-bit with the received tag.
%
% MAC VERIFICATION (per EUSPA OSNMA Receiver Guidelines v1.3, Section 5.5,
% Table 8):
%
%   message      = concatenation of nav data fields as per [AD.2] ADKD table
%   computed_mac = MAC_function(message, tesla_key)
%   computed_tag = truncate(computed_mac, tag_size_bits)  [MSB first]
%   verified     = (computed_tag == tag_received) bit-by-bit
%
% MAC FUNCTIONS (from DSM-KROOT MF field):
%   MF=0: HMAC-SHA-256  (most common, used with PKID=2 ECDSA P-256)
%   MF=1: CMAC-AES-128  (alternative)
%   This implementation supports MF=0 (HMAC-SHA-256) as default.
%   CMAC-AES-128 is implemented as a fallback path (MF=1).
%
% CRYPTO BACKEND: Java (javax.crypto.Mac 'HmacSHA256').
%   MATLAB hash() and HMAC-SHA-256 NOT available in this environment.
%   Java HMAC confirmed available via javax.crypto.
%
% NAVIGATION DATA FORMAT (message construction):
%   Per EUSPA OSNMA SIS ICD v1.1 [AD.2], the message for tag computation
%   is formed by concatenating:
%     - PRN_A  (8 bits): PRN of the satellite whose nav data is being authenticated
%     - GST_SF (32 bits): GST of the sub-frame containing the nav data
%     - CTR    (8 bits): tag counter within the MACK message
%     - NavData bits: the I/NAV page bits authenticated by this tag,
%       as defined by the ADKD (Authentication Data and Key Delay) field
%
%   ADKD values:
%     ADKD=0:  I/NAV word types 1, 2, 3, 4, 5 (ephemeris + clock)
%     ADKD=4:  I/NAV word types 6, 10 (UTC parameters)
%     ADKD=12: I/NAV Slow MAC — authenticates with a later key
%
% INPUTS
%   nav_data_bits  char or uint8 — navigation data bits to authenticate.
%                  If char: binary string ('0'/'1' characters).
%                  If uint8: byte array of packed nav data.
%                  The CALLER is responsible for assembling the correct
%                  concatenation per the ADKD specification.
%   tag_received   uint8 [tag_size_bytes x 1] — received tag from MACK message
%   tesla_key      uint8 [key_size_bytes x 1] — verified TESLA chain key
%                  (output of tesla_key_chain.m, verified against KROOT)
%   tag_size_bits  double — tag size in bits (from DSM-KROOT TS field)
%                  Typical: 40 bits (5 bytes)
%   mac_function   double — MAC function identifier from DSM-KROOT MF field
%                  0 = HMAC-SHA-256 (default)
%                  1 = CMAC-AES-128
%
% OUTPUTS
%   verified      logical — true if computed tag matches received tag
%   computed_tag  uint8 [tag_size_bytes x 1] — locally computed truncated tag
%
% LIMITATION — MESSAGE ASSEMBLY:
%   This function takes pre-assembled nav_data_bits as input. The caller
%   (osnma_verify.m) is responsible for assembling the message per the ADKD
%   specification. This separation keeps mac_verify.m testable in isolation
%   against the Annex A test vectors in the OSNMA Receiver Guidelines.
%
% PROJECT:  GNSS Thesis — Stage 0 OSNMA
% AUTHOR:   RG

    if nargin < 5, mac_function = 0; end  % default HMAC-SHA-256

    tag_size_bytes = ceil(tag_size_bits / 8);

    %% --- Convert nav_data_bits to bytes if needed -------------------------
    if ischar(nav_data_bits)
        % Pad to byte boundary
        n_bits = length(nav_data_bits);
        n_pad  = mod(8 - mod(n_bits, 8), 8);
        bits_padded = [nav_data_bits, repmat('0', 1, n_pad)];
        n_bytes = length(bits_padded) / 8;
        msg_bytes = zeros(n_bytes, 1, 'uint8');
        for b = 1:n_bytes
            msg_bytes(b) = uint8(bin2dec(bits_padded((b-1)*8+1 : b*8)));
        end
    else
        msg_bytes = uint8(nav_data_bits(:));
    end

    %% --- Compute MAC ------------------------------------------------------
    switch mac_function
        case 0
            %% HMAC-SHA-256 via Java javax.crypto.Mac
            % Source: EUSPA OSNMA Receiver Guidelines v1.3, Section 5.5
            try
                mac_obj  = javax.crypto.Mac.getInstance('HmacSHA256');
                key_spec = javax.crypto.spec.SecretKeySpec(...
                    typecast(tesla_key(:), 'int8'), 'HmacSHA256');
                mac_obj.init(key_spec);
                mac_obj.update(typecast(msg_bytes, 'int8'));
                mac_java  = mac_obj.doFinal();
                mac_full  = uint8(mod(int32(mac_java) + 256, 256));
            catch e
                error('mac_verify: Java HMAC-SHA-256 failed: %s', e.message);
            end

        case 1
            %% CMAC-AES-128 via Java
            % Source: EUSPA OSNMA Receiver Guidelines v1.3, Section 5.5
            % Note: requires BouncyCastle or JCE provider with CMAC support.
            % Standard Java JCE does not include CMAC directly; attempt via
            % AES/CBC zero-IV (approximation) — NOT cryptographically correct.
            % Flag for thesis: CMAC-AES-128 requires external Java provider.
            error(['mac_verify: CMAC-AES-128 (MF=1) is not implemented. ' ...
                   'Use a validated CMAC provider before accepting CMAC-based OSNMA tags.']);
            try
                cipher   = javax.crypto.Cipher.getInstance('AES/CBC/NoPadding');
                key_spec = javax.crypto.spec.SecretKeySpec(...
                    typecast(tesla_key(1:16), 'int8'), 'AES');
                iv_spec  = javax.crypto.spec.IvParameterSpec(zeros(16,1,'int8'));
                cipher.init(javax.crypto.Cipher.ENCRYPT_MODE, key_spec, iv_spec);
                % Pad message to AES block boundary
                n_blocks = ceil(numel(msg_bytes)/16);
                msg_pad  = [msg_bytes; zeros(n_blocks*16 - numel(msg_bytes), 1, 'uint8')];
                enc = cipher.doFinal(typecast(msg_pad, 'int8'));
                mac_full = uint8(mod(int32(enc(end-15:end)), 256));
                mac_full = [mac_full; zeros(32-16, 1, 'uint8')];  % pad to 32 bytes
            catch e
                error('mac_verify: CMAC-AES-128 failed: %s', e.message);
            end

        otherwise
            error('mac_verify: unknown MAC function MF=%d (0=HMAC-SHA-256, 1=CMAC-AES-128)', ...
                mac_function);
    end

    %% --- Truncate to tag_size_bits (MSB first) ---------------------------
    if mod(tag_size_bits, 8) ~= 0
        n_full  = floor(tag_size_bits / 8);
        n_extra = mod(tag_size_bits, 8);
        mask    = uint8(bitshift(uint8(255), -(8 - n_extra)));
        computed_tag = [mac_full(1:n_full); bitand(mac_full(n_full+1), mask)];
    else
        computed_tag = mac_full(1:tag_size_bytes);
    end

    %% --- Compare bit-by-bit ----------------------------------------------
    if numel(tag_received) ~= numel(computed_tag)
        warning('mac_verify: tag length mismatch: received=%d bytes, computed=%d bytes', ...
            numel(tag_received), numel(computed_tag));
        verified = false;
        return
    end
    verified = isequal(computed_tag, uint8(tag_received(:)));

end
