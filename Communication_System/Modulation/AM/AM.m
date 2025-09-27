%% Enhanced Amplitude Modulation (AM) with Envelope
clc; clear; close all;

%% Parameters
Fs = 50000;             % Higher sampling frequency for smooth curves
t = 0:1/Fs:0.01;        % Time vector (10 ms)
Am = 1;                 % Message amplitude
Ac = 2;                 % Carrier amplitude
fm = 200;               % Message frequency (Hz)
fc = 2000;              % Carrier frequency (Hz)
ka = 0.7;               % Modulation index

%% Message signal
m = Am * sin(2*pi*fm*t);

%% Carrier signal
c = Ac * cos(2*pi*fc*t);

%% AM signal
s = (Ac + ka*m) .* cos(2*pi*fc*t);

%% Plotting Signals
figure('Name','AM Signals ','NumberTitle','off','Position',[100 100 900 700]);

% Message signal
subplot(3,1,1);
plot(t, m, 'b','LineWidth',1.5);
grid on;
title('Message Signal m(t)');
xlabel('Time (s)');
ylabel('Amplitude');

% Carrier signal
subplot(3,1,2);
plot(t, c, 'r','LineWidth',1.5);
grid on;
title('Carrier Signal c(t)');
xlabel('Time (s)');
ylabel('Amplitude');

% AM signal with envelope
subplot(3,1,3);
plot(t, s, 'm','LineWidth',1.5); hold on;
plot(t, Ac + ka*m, 'k--','LineWidth',1.5);   % Upper envelope
plot(t, -(Ac + ka*m), 'k--','LineWidth',1.5); % Lower envelope
grid on;
title('AM Signal s(t) with Envelope');
xlabel('Time (s)');
ylabel('Amplitude');
legend('AM Signal','Envelope','Location','best');

 %Frequency Spectrum of AM signal
figure('Name','AM Spectrum','NumberTitle','off','Position',[150 150 800 400]);
N = length(s);
S_f = fftshift(fft(s)/N);
f = linspace(-Fs/2, Fs/2, N);
plot(f, abs(S_f), 'LineWidth',1.5);
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('Frequency Spectrum of AM Signal');
