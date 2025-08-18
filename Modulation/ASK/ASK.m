clc; clear; close all;

%% Parameters
Fs = 1000;            % Sampling frequency 
Tb = 0.1;             % Bit duration 
t = 0:1/Fs:Tb;        % Time vector for one bit
fc = 50;              % Carrier frequency (
A0 = 0;               % Amplitude for bit 0
A1 = 1;               % Amplitude for bit 1

%% Message bits
bits = [1 0 1 1 0 0 1];   
N = length(bits);

%% Generate ASK signal
ask_signal = [];
time = [];

for i = 1:N
    carrier = cos(2*pi*fc*t);         
    if bits(i) == 1
        s = A1 * carrier;
    else
        s = A0 * carrier;
    end
    ask_signal = [ask_signal s];
    time = [time t + (i-1)*Tb];
end

%% Full carrier for comparison
full_carrier = repmat(cos(2*pi*fc*t), 1, N); 

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
plot(time, full_carrier, 'LineWidth', 2, 'Color', [0.85 0.33 0.1]);
title('Original Carrier Signal', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
grid on;

%% Plot ASK signal
subplot(3,1,3);
plot(time, ask_signal, 'LineWidth', 2, 'Color', [0.47 0.67 0.19]);
title('ASK Signal', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
grid on;
