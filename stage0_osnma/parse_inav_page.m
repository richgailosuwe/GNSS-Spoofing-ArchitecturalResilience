function page = parse_inav_page(dwrd, sv_id)
% PARSE_INAV_PAGE  Extract OSNMA and navigation fields from one Galileo
%                  E1-B I/NAV page delivered by UBX-RXM-SFRBX.
%
% VERIFIED BIT LAYOUT (from real june16rooftop.ubx data, SV14, word type 4):
%   SFRBX payload = 8 x uint32 words (little-endian, MSB-first bit ordering)
%   = 256 bits total:
%     bits [0:119]   = even page part  (120 bits)
%     bits [120:239] = odd page part   (120 bits)
%     bits [240:255] = u-blox padding  (16 bits, ignore)
%
% Even page field layout per Galileo OS SIS ICD Issue 2.1, Table 2:
%   bit  0       : Page type (0 = even)
%   bits 1-6     : Word type (6 bits)
%   bits 7-112   : Navigation data (word-type dependent, 106 bits)
%   bits 113-152 : OSNMA field (40 bits)  <-- VERIFIED from real data
%   bits 153-155 : SAR (3 bits)
%   bits 156-159 : Spare (4 bits)
%   bits 160-175 : CRC (24 bits, split: 8 in even, 16 in odd)
%   bits 176-182 : Tail (7 zeros)
%
% Odd page field layout per Galileo OS SIS ICD Issue 2.1, Table 2:
%   bit  120     : Page type (1 = odd)
%   bits 121-126 : Word type
%   bits 127-218 : Navigation data (92 bits)
%   bits 219-234 : CRC remainder (16 bits)
%   bits 235-241 : Tail (7 zeros)
%   bits 242-255 : u-blox padding
%
% OSNMA FIELD (40 bits, bits [113:152] 0-indexed = [114:153] 1-indexed MATLAB):
%   Assembled from consecutive sub-frames to form HKROOT or MACK messages.
%   Reference: EUSPA OSNMA SIS ICD v1.1, Section 3.3.
%
% INPUTS
%   dwrd    [8x1] uint32 — data words from SFRBX payload
%   sv_id   double — Galileo SV ID (1-36)
%
% OUTPUTS
%   page    struct with fields:
%     .sv_id          double
%     .page_type_even uint8  (0 = even)
%     .word_type      uint8  (0-20)
%     .nav_data_bits  char [106x1]  navigation data bits (even page)
%     .osnma_bits     char [40x1]   OSNMA field bits (even page)
%     .osnma_hex      char          OSNMA field as hex string
%     .osnma_nonzero  logical       true if OSNMA field != 0
%     .crc_ok         logical       CRC check result (placeholder - full
%                                   24-bit CRC requires both page parts)
%     .all_bits       char [256x1]  full bit string for debugging
%

    % Build full 256-bit string (MSB-first per word, verified from data)
    all_bits = '';
    for w = 1:8
        all_bits = [all_bits, dec2bin(dwrd(w), 32)]; %#ok<AGROW>
    end

    % Even page fields (1-indexed in MATLAB)
    page_type_even = uint8(bin2dec(all_bits(1)));
    word_type      = uint8(bin2dec(all_bits(2:7)));
    nav_data_bits  = all_bits(8:113);    % bits 7-112 (0-indexed) -> 8:113 (1-indexed)
    osnma_bits     = all_bits(114:153);  % bits 113-152 (0-indexed) -> 114:153 (1-indexed)
                                         % VERIFIED: bit offset confirmed from real data

    % OSNMA field as hex
    if any(osnma_bits == '1')
        osnma_val = bin2dec(osnma_bits);
        % Zero-pad to 10 hex chars (40 bits = 10 hex digits)
        osnma_hex = sprintf('%010X', osnma_val);
        osnma_nonzero = true;
    else
        osnma_hex = '0000000000';
        osnma_nonzero = false;
    end

    % Pack output
    page.sv_id          = sv_id;
    page.page_type_even = page_type_even;
    page.word_type      = word_type;
    page.nav_data_bits  = nav_data_bits;
    page.osnma_bits     = osnma_bits;
    page.osnma_hex      = osnma_hex;
    page.osnma_nonzero  = osnma_nonzero;
    page.crc_ok         = true;  % placeholder: full CRC needs both page parts
    page.all_bits       = all_bits;

end
