
clear; clc; close all;

%% Parameters
numUsers = 4;             
bitsPerUser = 16;          
EbNo_dB = 10;              

%% Validate Walsh Code Length
if log2(numUsers) ~= round(log2(numUsers))
    error('Number of users must be a power of 2 (2, 4, 8, ...).');
end

spreadLength = numUsers;  

%% Generate Random
data = randi([0 1], numUsers, bitsPerUser);

% BPSK Modulatio
bpskData = 2 * data - 1;

%% Generate Walsh Codes
walshMatrix = hadamard(spreadLength);   
codes = walshMatrix(1:numUsers, :);     

%% Transmitter: Spread and Superimpose Signals
spreadData = zeros(numUsers, bitsPerUser * spreadLength);

for user = 1:numUsers
    for bit = 1:bitsPerUser
        spreadBit = bpskData(user, bit) * codes(user, :);
        idx = (bit-1)*spreadLength + 1 : bit*spreadLength;
        spreadData(user, idx) = spreadBit;
    end
end

% Superimpose all user signals
txSignal = sum(spreadData, 1);

%% Channel: Add AWGN Noise
% Calculate SNR from Eb/No (dB)
EbNo_linear = 10^(EbNo_dB/10);
bitsTotal = numUsers * bitsPerUser;
signalPower = mean(txSignal.^2);
noisePower = signalPower / (2 * EbNo_linear * log2(2));  % BPSK -> 1 bit/symbol

% Add noise
txSignalNoisy = txSignal + sqrt(noisePower) * randn(size(txSignal));

%% Receiver: Despread and Decode
rxData = zeros(numUsers, bitsPerUser);

for user = 1:numUsers
    for bit = 1:bitsPerUser
        idx = (bit-1)*spreadLength + 1 : bit*spreadLength;
        receivedSegment = txSignalNoisy(idx);
        correlation = dot(receivedSegment, codes(user, :));
        rxData(user, bit) = correlation > 0;  % Decision threshold at 0
    end
end

%% Results
fprintf('--- CDMA Simulation Results ---\n');
disp('Original Data:');
disp(data);
disp('Decoded Data:');
disp(rxData);

% BER Calculation
errors = sum(data ~= rxData, 'all');
ber = errors / (numUsers * bitsPerUser);
fprintf('Bit Error Rate (BER): %.4f (%d errors)\n', ber, errors);

%% Plot
figure;
subplot(3,1,1);
plot(txSignal, 'b');
title('Transmitted Signal (Clean)');
ylabel('Amplitude');
grid on;

subplot(3,1,2);
plot(txSignalNoisy, 'r');
title('Received Signal with Noise');
ylabel('Amplitude');
grid on;

subplot(3,1,3);
plot(txSignalNoisy - txSignal, 'k');
title('Noise Component');
xlabel('Sample Index');
ylabel('Noise');
grid on;
