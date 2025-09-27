%% 16-QAM Simulation in MATLAB

clc; clear; close all;

%% Parameters
M = 16;                  % Modulation order (16-QAM)
numSymbols = 10000;      % Number of symbols
EbNo = 10;               % Energy per bit to noise power spectral density ratio (dB)

%% Generate Random Data
data = randi([0 M-1], numSymbols, 1);  % Random integer data stream

%% 16-QAM Modulation
modData = qammod(data, M, 'UnitAveragePower', true);

%% Transmit through AWGN Channel
rxSig = awgn(modData, EbNo, 'measured');  % Additive White Gaussian Noise

%% 16-QAM Demodulation
demodData = qamdemod(rxSig, M, 'UnitAveragePower', true);

%% Bit Error Rate Calculation
numErrors = sum(data ~= demodData);
ber = numErrors / numSymbols;
disp(['Bit Error Rate (BER): ', num2str(ber)]);

%% Constellation Diagrams using scatter()
figure;
scatter(real(modData), imag(modData), 'bo');
xlabel('In-Phase'); ylabel('Quadrature');
title('Transmitted 16-QAM Constellation');
axis square; grid on;

figure;
scatter(real(rxSig), imag(rxSig), 'r.');
xlabel('In-Phase'); ylabel('Quadrature');
title(['Received 16-QAM Constellation with AWGN (Eb/No = ', num2str(EbNo), ' dB)']);
axis square; grid on;
