%%  Regenerative Braking Simulation 

clear; close all; clc;

%% -------------------- Vehicle Parameters --------------------
m       = 1500;        % Vehicle mass [kg]
r_wheel = 0.3;         % Wheel radius [m]
g       = 9.81;        % Gravity [m/s^2]
Cd      = 0.32;        % Aerodynamic drag coefficient
A       = 2.2;         % Frontal area [m^2]
rho_air = 1.225;       % Air density [kg/m^3]
Cr      = 0.01;        % Rolling resistance coefficient
theta   = 0;           % Road slope [rad]
gear_ratio = 9;        % Gear reduction ratio

%% -------------------- Motor Parameters --------------------
p        = 4;          % Pole pairs
Rs       = 0.03;       % Stator resistance [Ohm]
Ld       = 0.0006;     % d-axis inductance [H]
Lq       = 0.0007;     % q-axis inductance [H]
lambda_m = 0.015;      % Permanent magnet flux [Wb]
Iq_max   = 180;        % Max q-axis current [A]
Id_max   = 50;         % Max d-axis current [A]
Te_max   = 250;        % Max motor torque [Nm]
eta_motor = 0.95;      % Motor efficiency

%% -------------------- Battery Parameters --------------------
V_batt   = 300;        % Nominal voltage [V]
C_batt   = 50;         % Capacity [Ah]
SOC      = 0.8;        % Initial SOC
SOC_min  = 0.2;        % Minimum SOC
SOC_max  = 1.0;        % Maximum SOC
I_max    = 100;        % Max charge current [A]
eta_batt = 0.9;        % Battery efficiency
eta_inv  = 0.95;       % Inverter efficiency

%% -------------------- Simulation Parameters --------------------
fsim = 1000;           % Simulation frequency [Hz]
dt   = 1/fsim;         
Tsim = 30;              % Total simulation time [s]
time = 0:dt:Tsim;
N    = length(time);

%% -------------------- Drive Cycle --------------------
V_max = 20; 
% Sinusoidal speed profile (placeholder for NEDC/WLTP)
V_ref = V_max * (0.5 + 0.5*sin(2*pi*time/Tsim));

%% -------------------- Preallocate Variables --------------------
v      = zeros(1,N);   % Vehicle speed [m/s]
omega  = zeros(1,N);   % Motor angular speed [rad/s]
Te     = zeros(1,N);   % Motor torque [Nm]
Tb     = zeros(1,N);   % Mechanical brake torque [Nm]
P_rec  = zeros(1,N);   % Regenerated power [W]
SOC_t  = zeros(1,N);   % Battery SOC
E_rec  = zeros(1,N);   % Cumulative energy recovered [J]
P_loss_motor = zeros(1,N); % Motor losses
P_loss_inv   = zeros(1,N); % Inverter losses

v(1)     = V_ref(1); 
SOC_t(1) = SOC;

%% -------------------- Simulation Loop --------------------
for k = 1:N-1
    %% Vehicle acceleration from reference speed
    a_des = (V_ref(k+1) - v(k))/dt;
    
    %% Resistive forces
    F_roll = m*g*Cr*cos(theta);
    F_slope = m*g*sin(theta);
    F_drag = 0.5*rho_air*Cd*A*v(k)^2;
    
    %% Required wheel torque
    F_req = m*a_des + F_roll + F_drag + F_slope;
    Te_req = F_req * r_wheel / gear_ratio;
    
    %% ----------------- Braking Control -----------------
    if Te_req < 0
        % Regenerative braking
        Te_m = max(Te_req, -Te_max);
        % Mechanical brake supplements if torque demand exceeds motor limit
        Tb_m = max(0, abs(Te_req) - abs(Te_m));
    else
        % Motoring
        Te_m = min(Te_req, Te_max);
        Tb_m = 0;
    end
    
    %% ----------------- Motor Dynamics -----------------
    % Rotor speed approximation
    omega(k+1) = v(k)/r_wheel * gear_ratio;
    Te(k) = Te_m;
    Tb(k) = Tb_m;
    
    % Electrical current limits (simplified)
    Iq = Te_m / (1.5 * p * lambda_m);
    Iq = max(min(Iq, Iq_max), -Iq_max);
    
    %% ----------------- Power & SOC -----------------
if Te_m < 0
    Pelec = -Te_m * v(k)/r_wheel / eta_inv; % Electrical power
    Pelec = Pelec * eta_batt;
    Pelec = min(Pelec, I_max*V_batt);        % Limit by max charge current
    P_rec(k) = Pelec;
    
    % Update SOC
    SOC_t(k+1) = SOC_t(k) + Pelec*dt/(C_batt*3600*V_batt);
    SOC_t(k+1) = min(max(SOC_t(k+1), SOC_min), SOC_max);
    
    % Cumulative recovered energy
    E_rec(k+1) = E_rec(k) + Pelec*dt;
else
    Pelec = 0;  % <-- Add this line
    SOC_t(k+1) = SOC_t(k);
    P_rec(k) = 0;
    E_rec(k+1) = E_rec(k);
end

%% ----------------- Thermal Losses -----------------
P_loss_motor(k) = (Te_m^2)/Te_max * (1-eta_motor)*abs(Te_m); % simplified
P_loss_inv(k)   = Pelec*(1-eta_inv);  % safe now, Pelec always defined

    %% ----------------- Vehicle Speed Update -----------------
    a_net = (Te_m + Tb_m)*gear_ratio/(m*r_wheel) - (F_roll + F_drag + F_slope)/m;
    v(k+1) = v(k) + a_net*dt;
end

%% -------------------- Plots --------------------
figure('Name','Regenerative Braking Simulation','Position',[100 100 1200 700]);

subplot(4,1,1);
plot(time,v,'LineWidth',1.5); hold on;
plot(time,V_ref,'--','LineWidth',1.2);
ylabel('Vehicle Speed [m/s]'); xlabel('Time [s]');
legend('Actual','Reference'); grid on; title('Vehicle Speed');

subplot(4,1,2);
plot(time,Te,'LineWidth',1.5); hold on;
plot(time,Tb,'--','LineWidth',1.2);
ylabel('Motor/Brake Torque [Nm]'); xlabel('Time [s]');
legend('Motor Torque','Brake Torque'); grid on; title('Torque Profile');

subplot(4,1,3);
plot(time,SOC_t,'LineWidth',1.5);
ylabel('Battery SOC'); xlabel('Time [s]'); grid on; title('State of Charge');

subplot(4,1,4);
plot(time,E_rec,'LineWidth',1.5);
ylabel('Cumulative Energy Recovered [J]'); xlabel('Time [s]');
grid on; title('Energy Recovery');

figure('Name','Power & Losses','Position',[100 100 1000 400]);
plot(time,P_rec,'LineWidth',1.5); hold on;
plot(time,P_loss_motor,'--','LineWidth',1.2);
plot(time,P_loss_inv,':','LineWidth',1.2);
ylabel('Power [W]'); xlabel('Time [s]');
legend('Recovered Power','Motor Loss','Inverter Loss'); grid on; title('Power and Thermal Losses');
