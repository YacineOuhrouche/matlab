%% switching_loss_efficiency_mosfet

clear; close all; clc;

%% Parameters
Vdc     = 400;       
Iload   = 10;        
Dvals   = [0.2 0.5 0.8];   % Duty cycles 
fswVals = logspace(3,6,50); % switching frequency (1 kHz ->1 MHz)

% MOSFET parameters
Rds_on  = 50e-3;     % MOSFET resistance 
tr      = 50e-9;     % rise time 
tf      = 50e-9;     % fall time 
Qg      = 100e-9;    %gate charge 
Vgs     = 10;        % gate drive voltage 
QgLoss  = Qg*Vgs;    %gate drive energy per switch (J)

%%   Loss Functions 
% Conduction loss MOSFET
function Pcond = conductionLoss(D, Iload, Rds_on)
    Pcond = D * Iload^2 * Rds_on;
end

% Switching loss (turn-on + turn-off) for MOSFET
function Psw = switchingLoss(Vdc, Iload, fsw, tr, tf, QgLoss)
    % Energy per transition ~ 0.5*V*I*(tr + tf)
    Eon  = 0.5 * Vdc * Iload * tr;
    Eoff = 0.5 * Vdc * Iload * tf;
    Esw  = Eon + Eoff + QgLoss; % include gate-drive loss
    Psw  = Esw * fsw;
end

%% ---------------- Main Sweep ----------------
results = struct();

for d = 1:length(Dvals)
    D = Dvals(d);
    etaHard = zeros(size(fswVals));
    etaSoft = zeros(size(fswVals));
    
    for k = 1:length(fswVals)
        fsw = fswVals(k);

        % Load power
        Pout = D * Vdc * Iload;

        % Hard switching losses
        Pcond = conductionLoss(D, Iload, Rds_on);
        Psw   = switchingLoss(Vdc, Iload, fsw, tr, tf, QgLoss);
        PlossHard = Pcond + Psw;
        etaHard(k) = Pout / (Pout + PlossHard);

        % Soft switching assumption: reduces Eon/Eoff by ~80%
        PswSoft = 0.2 * Psw; 
        PlossSoft = Pcond + PswSoft;
        etaSoft(k) = Pout / (Pout + PlossSoft);
    end
    
    results(d).D = D;
    results(d).etaHard = etaHard;
    results(d).etaSoft = etaSoft;
end

%%Plots  
figure('Name','MOSFET Switching Loss & Efficiency','NumberTitle','off','Position',[100 100 900 600]);

for d = 1:length(Dvals)
    subplot(1,2,1);
    semilogx(fswVals/1e3, results(d).etaHard*100, 'LineWidth', 2); hold on;
    semilogx(fswVals/1e3, results(d).etaSoft*100, '--', 'LineWidth', 2);
    xlabel('Switching Frequency (kHz)'); ylabel('Efficiency (%)');
    grid on; title('MOSFET Efficiency vs f_{sw}');
end
legendStr = {};
for d = 1:length(Dvals)
    legendStr{end+1} = sprintf('D=%.1f Hard', results(d).D);
    legendStr{end+1} = sprintf('D=%.1f Soft', results(d).D);
end
legend(legendStr,'Location','best');

subplot(1,2,2);
for d = 1:length(Dvals)
    lossHard = 100*(1-results(d).etaHard);
    lossSoft = 100*(1-results(d).etaSoft);
    semilogx(fswVals/1e3, lossHard,'LineWidth',2); hold on;
    semilogx(fswVals/1e3, lossSoft,'--','LineWidth',2);
end
xlabel('Switching Frequency (kHz)'); ylabel('Loss (%)');
grid on; title('MOSFET Loss vs f_{sw}');
legend(legendStr,'Location','northwest');

%% Results Summary 
fprintf('--- MOSFET Switching Loss and Efficiency Summary ---\n');
for d = 1:length(Dvals)
    fprintf('Duty cycle D = %.2f:\n', results(d).D);
    fprintf('  Efficiency at 20 kHz (Hard) = %.2f %%\n', 100*interp1(fswVals,results(d).etaHard,20e3));
    fprintf('  Efficiency at 100 kHz (Hard) = %.2f %%\n', 100*interp1(fswVals,results(d).etaHard,100e3));
    fprintf('  Efficiency at 100 kHz (Soft) = %.2f %%\n\n', 100*interp1(fswVals,results(d).etaSoft,100e3));
end
