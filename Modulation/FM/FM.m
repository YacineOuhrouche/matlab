% FM Modulation Example - Clearer Frequency Change
fs = 1e4;         % Sampling frequency (Hz)
t = 0:1/fs:0.1;   % Time vector (0.1 s window to see detail)

% Message signal (low frequency so we can see the changes)
fm = 50;          % Message frequency (Hz)
Am = 1;           % Message amplitude
m = Am*cos(2*pi*fm*t);

% Carrier signal
fc = 500;         % Carrier frequency (Hz)
Ac = 1;           % Carrier amplitude

% Frequency deviation (big enough to see changes in time waveform)
kf = 200;          % Frequency sensitivity (Hz per unit amplitude)

% FM signal
y = Ac * cos(2*pi*fc*t + (kf/fm)*sin(2*pi*fm*t));

% --- Plots ---
figure;

subplot(3,1,1);
plot(t, m, 'b', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Amplitude');
title('Message Signal (m(t))');
grid on;

subplot(3,1,2);
plot(t, cos(2*pi*fc*t), 'r', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Amplitude');
title('Carrier Signal (c(t))');
grid on;

subplot(3,1,3);
plot(t, y, 'k', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Amplitude');
title('FM Signal ');
grid on;

