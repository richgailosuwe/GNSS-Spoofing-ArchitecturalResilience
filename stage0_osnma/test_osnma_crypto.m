%% TEST_OSNMA_CRYPTO  Annex A test vector validation for OSNMA crypto functions.
%
% Validates tesla_key_chain.m and mac_verify.m against the exact test vectors
% published in EUSPA OSNMA Receiver Guidelines v1.3, Annex A (Sections A.5
% and A.6). These are the authoritative EUSPA-published reference values.
%
% Run from project root after config:
%   config
%   run('stage0_osnma/test_osnma_crypto.m')
%
% Expected result: PASS (3/3)
%
% CONFIGURATION (Annex A.2):
%   Tag size:  40 bits, Key size: 128 bits
%   Hash: SHA-256, MAC: HMAC-SHA-256
%
% TEST STRUCTURE:
%   Test 1 -- TESLA K2 -> K1 (Section A.5.2, step 1)
%   Test 2 -- TESLA K1 -> K0/KROOT (Section A.5.2, step 2)
%   Test 3 -- Tag0 MAC verification (Section A.6.5.1, ADKD0, K4)
%
% PROJECT:  GNSS Thesis -- Stage 0 OSNMA
% AUTHOR:   RG

fprintf('\n=== test_osnma_crypto.m ===\n');
fprintf('Source: EUSPA OSNMA Receiver Guidelines v1.3, Annex A\n\n');

n_tests = 0;
n_pass  = 0;
KS_bits = 128;
TS_bits = 40;

%% =========================================================================
%% TEST 1: TESLA key chain K2 -> K1  (Section A.5.2, step 1)
%% =========================================================================
fprintf('Test 1: TESLA key chain K2 -> K1 (Section A.5.2)\n');
n_tests = n_tests + 1;

% From Annex A.5.2:
%   Full message (208 bits) = K2 || GST_SF(K1) || alpha
%   0x2DC3A3CDB117FAADB83B5F0B6FEA88EB  4E054600  610BDF26D77B
%   K2  (128 bits = 16 bytes): 2DC3A3CDB117FAADB83B5F0B6FEA88EB
%   GST_SF(K1): WN=1248, TOW=345600 -> (1248<<20)|345600 = 0x4E054600
%   alpha (48 bits = 6 bytes): 610BDF26D77B
%   Expected K1: EFF999040E19B570835060BEBD23ED92
%
% Function signature: tesla_key_chain(K_I, GST_SF, alpha, K_J, key_size_bits)
%   K_I = key being verified (EARLIER in chain = smaller index)
%   K_J = already-verified key (LATER in chain = larger index)
%   Computes: hash(K_J || GST_SF || alpha), truncates, compares to K_I
%   So: K_I=K1, K_J=K2

K2          = hex2bytes('2DC3A3CDB117FAADB83B5F0B6FEA88EB');
alpha_A5    = hex2bytes('610BDF26D77B');
K1_expected = hex2bytes('EFF999040E19B570835060BEBD23ED92');

WN_K1  = uint32(1248);
TOW_K1 = uint32(345600);
GST_K1 = bitor(bitshift(WN_K1, 20), TOW_K1);  % = 0x4E054600

[ok1, K1_computed] = tesla_key_chain(K1_expected, GST_K1, alpha_A5, K2, KS_bits);

fprintf('  K2:          %s\n', bytes2hex(K2));
fprintf('  GST_SF(K1):  0x%08X (WN=%d TOW=%d)\n', GST_K1, WN_K1, TOW_K1);
fprintf('  alpha:       %s\n', bytes2hex(alpha_A5));
fprintf('  K1 expected: %s\n', bytes2hex(K1_expected));
fprintf('  K1 computed: %s\n', bytes2hex(K1_computed));

if ok1
    fprintf('  PASS -- K1 matches\n\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL -- K1 mismatch\n\n');
end

%% =========================================================================
%% TEST 2: TESLA key chain K1 -> K0/KROOT  (Section A.5.2, step 2)
%% =========================================================================
fprintf('Test 2: TESLA key chain K1 -> K0/KROOT (Section A.5.2)\n');
n_tests = n_tests + 1;

% From Annex A.5.2 second step:
%   Message = K1 || GST_SF(K0) || alpha
%   GST_SF(K0): WN=1248, TOW=345570  (30 s before K1 sub-frame)
%   Same alpha as step 1: 610BDF26D77B
%   Expected K0 (KROOT): 5BF8C9CBFCF70422081475FD445DF0FF

K0_expected = hex2bytes('5BF8C9CBFCF70422081475FD445DF0FF');

WN_K0  = uint32(1248);
TOW_K0 = uint32(345570);
GST_K0 = bitor(bitshift(WN_K0, 20), TOW_K0);

% Use K1_expected (the ICD ground-truth value, not K1_computed)
% to isolate this test from Test 1's result
[ok2, K0_computed] = tesla_key_chain(K0_expected, GST_K0, alpha_A5, K1_expected, KS_bits);

fprintf('  K1:          %s\n', bytes2hex(K1_expected));
fprintf('  GST_SF(K0):  0x%08X (WN=%d TOW=%d)\n', GST_K0, WN_K0, TOW_K0);
fprintf('  alpha:       %s\n', bytes2hex(alpha_A5));
fprintf('  K0 expected: %s\n', bytes2hex(K0_expected));
fprintf('  K0 computed: %s\n', bytes2hex(K0_computed));

if ok2
    fprintf('  PASS -- K0/KROOT matches\n\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL -- K0/KROOT mismatch\n\n');
end

%% =========================================================================
%% TEST 3: Tag0 MAC verification  (Section A.6.5.1, ADKD0)
%% =========================================================================
fprintf('Test 3: Tag0 MACK verification (Section A.6.5.1, ADKD0)\n');
n_tests = n_tests + 1;

% From Annex A.6.5.1:
%   Tag0 at TOW=345660, ADKD=0, verified with K4
%   K4 = 0x69C00AA7364237A65EBF006AD8DDBC73  (from Annex A.5.1 table)
%
%   Message m0 (600 bits = 75 bytes):
%   0x024E05463C0183A591051D692580076B3EEA8141BF03ADCB5AADB277AF6FCF21F
%     B98FF7E83AFFC370203B0D8E10EB14D1118E6B0E82001A000E5910006D31F0002
%     68054A02C22607F7FC00
%   NOTE: 600 bits = 75 bytes exactly (75*8=600). No padding needed.
%
%   Full HMAC-SHA-256 result:
%   0xE37BC4F858AE1E5CFDC46F054B1F47B9D2EA61E1EF09115CFE706852BFF23A83
%
%   Truncated to 40 bits (TS=40):
%   0xE37BC4F858

K4 = hex2bytes('69C00AA7364237A65EBF006AD8DDBC73');

% Message m0 (600 bits = 75 bytes) from Annex A.6.5.1:
m0_hex = ['024E05463C0183A591051D692580076B3EEA8141BF03ADCB5AADB277AF6FCF21F' ...
           'B98FF7E83AFFC370203B0D8E10EB14D1118E6B0E82001A000E5910006D31F0002' ...
           '68054A02C22607F7FC00'];
m0_bytes = hex2bytes(m0_hex);

tag_expected = hex2bytes('E37BC4F858');  % Tag0, 40 bits

fprintf('  K4:           %s\n', bytes2hex(K4));
fprintf('  m0 length:    %d bytes = %d bits (expect 75 bytes = 600 bits)\n', ...
    numel(m0_bytes), numel(m0_bytes)*8);
fprintf('  tag expected: %s\n', bytes2hex(tag_expected));

[ok3, tag_computed] = mac_verify(m0_bytes, tag_expected, K4, TS_bits, 0);

fprintf('  tag computed: %s\n', bytes2hex(tag_computed));

if ok3
    fprintf('  PASS -- Tag0 matches\n\n');
    n_pass = n_pass + 1;
else
    fprintf('  FAIL -- Tag0 mismatch\n\n');
end

%% =========================================================================
%% Summary
%% =========================================================================
fprintf('--- Results: %d/%d PASS ---\n', n_pass, n_tests);
if n_pass == n_tests
    fprintf('test_osnma_crypto: ALL PASS ✓\n');
    fprintf('\nConclusion:\n');
    fprintf('  SHA-256 TESLA key chain: cryptographically correct\n');
    fprintf('  HMAC-SHA-256 MAC/tag verification: cryptographically correct\n');
    fprintf('  Java crypto backend: ICD-compliant\n');
    fprintf('  Source: EUSPA OSNMA Receiver Guidelines v1.3, Annex A\n\n');
else
    fprintf('test_osnma_crypto: %d FAILURE(S) -- review output above\n\n', n_tests - n_pass);
end

%% =========================================================================
%% LOCAL HELPERS
%% =========================================================================

function bytes = hex2bytes(hex_str)
    hex_str = strtrim(hex_str);
    hex_str = hex_str(hex_str ~= ' ');
    if mod(length(hex_str), 2) ~= 0
        hex_str = ['0', hex_str];
    end
    n = length(hex_str) / 2;
    bytes = zeros(n, 1, 'uint8');
    for k = 1:n
        bytes(k) = uint8(hex2dec(hex_str((k-1)*2+1 : k*2)));
    end
end

function str = bytes2hex(bytes)
    str = sprintf('%02X', bytes(:));
end