clc; clear; close all;


%        FHSS SETTINGS
Rb          = 100;          % Bit rate
Tb          = 1/Rb;         % Bit duration 
hopsPerBit  = 3;            
numFreqs    = 8;            % Number of hop channels 
fc_min      = 1000;         % Lowest hop frequency 
df          = 300;          % Frequency spacing between channels
Fs          = 20000;        % Sampling frequency (Hz) 
Nbits       = 20;           % Number of data bits

rng(42);                     % Seed for reproducible hop sequence

%        DERIVED VARS

Tchip       = Tb / hopsPerBit;               % Hop (chip) duration
bits        = randi([0 1], 1, Nbits);        % Random data
fc_set      = fc_min + (0:numFreqs-1) * df;  % Carrier bank
numChips    = Nbits * hopsPerBit;            % Total hops
hop_idx     = randi(numFreqs, 1, numChips);  % Pseudorandom hop sequence (indices)
fc_chips    = fc_set(hop_idx);               % Hop frequencies per chip

% Time vectors
Ns_chip     = round(Tchip * Fs);             % Samples per chip
t_chip      = (0:Ns_chip-1) / Fs;            % Local time within each chip
Ttotal      = Nbits * Tb;                    % Total duration
time        = [];                            % Global time vector we'll build
fhss_signal = [];                            % Output FHSS signal

%   BUILD FHSS BPSK SIGNAL
A = 1; % amplitude

for k = 1:numChips
    % Which data bit applies in this chip?
    bitIdx = ceil(k / hopsPerBit);
    b      = bits(bitIdx);

    % BPSK 
    phase  = (b == 0) * pi;

    % Carrier for this chip
    fc     = fc_chips(k);

    % Chip waveform
    s_chip = A * cos(2*pi*fc*t_chip + phase);

    % Append 
    t0 = (k-1) * Tchip;
    fhss_signal = [fhss_signal, s_chip];
    time        = [time, t_chip + t0];
end

%         PLOTTING
figure('Color',[0.97 0.97 0.97], 'Position',[100 100 1000 800]);

% 1) Bits vs time
subplot(4,1,1);
stairs(0:Tb:Ttotal, [bits bits(end)], 'LineWidth', 2);
ylim([-0.2 1.2]); grid on;
xlabel('Time (s)'); ylabel('Bit');
title(sprintf('Digital Bits (Rb = %d bps, hops/bit = %d)', Rb, hopsPerBit));

% 2) Hop frequency track 
subplot(4,1,2);
edges_chip = 0:Tchip:Ttotal;
stairs(edges_chip, [fc_chips fc_chips(end)], 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Hop freq (Hz)');
title(sprintf('FHSS Hopping Pattern (%d channels from %d Hz, Δf = %d Hz)', ...
      numFreqs, fc_min, df));

% 3) FHSS time waveform
subplot(4,1,3);
plot(time, fhss_signal, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)'); ylabel('Amplitude');
title('FHSS Waveform (BPSK on hopping carrier)');

% 4) Spectrogram 
subplot(4,1,4);
win      = round(0.02*Fs);               
noverlap = round(0.75*win);
nfft     = 4096;
spectrogram(fhss_signal, win, noverlap, nfft, Fs, 'yaxis');
title('Spectrogram (Time–Frequency)'); ylim([0 max(fc_set)/1000*1.2]); % kHz axis
colormap turbo; colorbar('off'); axis tight;

