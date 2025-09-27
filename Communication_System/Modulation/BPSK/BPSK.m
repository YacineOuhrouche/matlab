%BPSK
clc; clear; close all;

%% Parameters
Fs = 1000;            % Sampling frequency 
Tb = 0.1;             % Bit duration 
t = 0:1/Fs:Tb;        % Time vector for one bit
fc = 20;              % Carrier frequency 
A = 1;                % Amplitude

%% Message bits
bits = [1 0 1 1 0 0 1];   
N = length(bits);

%%  BPSK signal
psk_signal = [];
time = [];

for i = 1:N
    if bits(i) == 1
        phase = 0;
    else
        phase = pi;  
    end
    s = A * cos(2*pi*fc*t + phase);
    psk_signal = [psk_signal s];
    time = [time t + (i-1)*Tb];
end

%% Plot message bits
figure('Color',[0.95 0.95 0.95]);
subplot(3,1,1);
stairs([0:N]*Tb, [bits bits(end)], 'LineWidth', 2, 'Color', [0 0.45 0.74]);
ylim([-0.2 1.2]);
title('Digital Message Bits', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Bit', 'FontSize', 12);
grid on;

%% Plot carrier signal
subplot(3,1,2);
plot(time, cos(2*pi*fc*time), 'LineWidth', 2, 'Color', [0.85 0.33 0.1]);
title('Original Carrier Signal', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
grid on;

%% Plot BPSK signal
subplot(3,1,3);
plot(time, psk_signal, 'LineWidth', 2, 'Color', [0.47 0.67 0.19]);
title('BPSK Signal with Moderate Phase Changes', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
grid on;
