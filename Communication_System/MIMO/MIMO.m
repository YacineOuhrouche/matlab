% Parameters
M = 16;               % 16-QAM
k = log2(M);          % Bits per symbol
numSymbols = 5000;    % Number of symbols
SNRdB = 0:5:30;       % SNR sweep

berZF = zeros(size(SNRdB)); 
berMMSE = zeros(size(SNRdB));

for s = 1:length(SNRdB)
    snr = SNRdB(s);

    % Random data (2 streams for 2x2 MIMO)
    data = randi([0 M-1], numSymbols, 2);
    modData = qammod(data, M, 'UnitAveragePower', true);

    % Channel: 2x2 Rayleigh fading
    H = (randn(numSymbols,2,2) + 1i*randn(numSymbols,2,2))/sqrt(2);

    % Transmit through channel
    rxSig = zeros(numSymbols,2);
    for i = 1:numSymbols
        x = squeeze(modData(i,:)).';
        y = squeeze(H(i,:,:))*x; 
        y = awgn(y, snr, 'measured');   % Add AWGN properly
        rxSig(i,:) = y.';
    end

    % Detection
    rxDataZF = zeros(numSymbols,2);
    rxDataMMSE = zeros(numSymbols,2);
    for i = 1:numSymbols
        y = rxSig(i,:).';
        Hmat = squeeze(H(i,:,:));

        % Zero Forcing
        x_hatZF = pinv(Hmat)*y;
        rxDataZF(i,:) = x_hatZF.';

        % MMSE Detection
        N0 = 10^(-snr/10);  % Noise variance
        x_hatMMSE = (Hmat'*Hmat + N0*eye(2))\(Hmat')*y;
        rxDataMMSE(i,:) = x_hatMMSE.';
    end

    % Demodulation
    rxSymbolsZF = qamdemod(rxDataZF, M, 'UnitAveragePower', true);
    rxSymbolsMMSE = qamdemod(rxDataMMSE, M, 'UnitAveragePower', true);

    % BER
    [~, berZF(s)] = biterr(data(:), rxSymbolsZF(:));
    [~, berMMSE(s)] = biterr(data(:), rxSymbolsMMSE(:));
end

% Plot BER vs SNR
figure;
semilogy(SNRdB, berZF, '-o', 'LineWidth', 2); hold on;
semilogy(SNRdB, berMMSE, '-s', 'LineWidth', 2);
grid on; xlabel("SNR (dB)"); ylabel("Bit Error Rate (BER)");
legend("Zero Forcing","MMSE");
title("2x2 MIMO 16-QAM BER Performance");

% Scatter plot at high SNR
figure;
scatterplot(rxDataMMSE(:,1));
title("Received Constellation (MMSE, Stream 1)");
