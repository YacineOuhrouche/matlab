% fsk 
clc; clear; close all; 

% parameters
Fs = 2000;% sampling frequency
Tb = 0.1; %bit duration
t= 0:1/Fs:Tb; %time vector
f0 = 50; % frequnency for bit 0
f1 = 100; %idem for 1
A =1; %amplitude of carrier


%message bits
bits =[1 0 1 1 0 0 1];
N = length(bits);

%generate FSK
fsk = [];
time=[];

for i =1:N
    if bits(i) ==1
        s=A*cos(2*pi*f1*t);
    else 
        s =A*cos(2*pi*f0*t);
    end

    fsk = [fsk s];
    time =[time t+(i-1)*Tb];

end

%plot
%% Plot message bits
figure('Color',[0.95 0.95 0.95]);
subplot(3,1,1);
stairs([0:N]*Tb, [bits bits(end)], 'LineWidth', 2, 'Color', [0 0.45 0.74]);
ylim([-0.2 1.2]);
title('Digital Message Bits', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Bit', 'FontSize', 12);
grid on;

%% Plot frequencies for reference
subplot(3,1,2);
plot(time, cos(2*pi*f0*time), '--', 'LineWidth', 1.5, 'Color', [0.85 0.33 0.1]); hold on;
plot(time, cos(2*pi*f1*time), '--', 'LineWidth', 1.5, 'Color', [0.3 0.3 0.3]);
title('Reference Carrier Frequencies f0 & f1', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
grid on;
legend('f0 = 50 Hz','f1 = 100 Hz');

%% Plot FSK signal
subplot(3,1,3);
plot(time, fsk, 'LineWidth', 2, 'Color', [0.47 0.67 0.19]);
title('FSK Signal', 'FontSize', 14);
xlabel('Time [s]', 'FontSize', 12);
ylabel('Amplitude', 'FontSize', 12);
grid on;

