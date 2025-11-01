%% Regenerative Braking Simulation


clear; close all; clc;

%% Vehicle Parameters 
m       = 1500;      
r_wheel = 0.3;       % Wheel radius
g       = 9.81;     
Cd      = 0.32;      % Drag  coefficient
A       = 2.2;       %  Frontal area 
rho_air = 1.225;     % Air density 
Cr      = 0.01;      %  Rolling resistance coefficient

%% Motor Parameters
p       = 4;          % Pole pairs
Rs      = 0.03;       % Stator resistance 
Ld      = 0.0006;     %  d-axis inductance 
Lq      = 0.0007;     % q-axis inductance 
lambda_m= 0.015;      % Permanent magnet flux 
Iq_max  = 180;        % Max q-axis current 
Id_max  = 50;         % Max d-axis current 
Te_max  = 250;        % Max torque 

%% Battery Parameters 
V_batt   = 300;       
C_batt   = 50;        % Capacity ah
SOC      = 0.8;       % Initial state-of-charge
SOC_min  = 0.2;       % Minimum SOC 
SOC_max  = 1.0;       % Maximum SOC
eta_batt = 0.9;       % Battery effi

%% Simulation Parameters
fsim = 1000;          
dt   = 1/fsim;         
Tsim = 30;              
time = 0:dt:Tsim;
N    = length(time);

%% Drive Cycle 
V_max = 20;            
V_ref = V_max * (0.5 + 0.5*sin(2*pi*time/Tsim)); % Sinusoidal speed profile

%%  Preallocate Variables
v     = zeros(1,N);       % Vehicle speed
omega = zeros(1,N);       % Motor angular speed
Te    = zeros(1,N);       %  torque
Tb    = zeros(1,N);       % Mechanical braking torque
P_rec = zeros(1,N);       % Regenerated power
SOC_t = zeros(1,N);       % Battery SOC
v(1) = V_ref(1); SOC_t(1) = SOC;

%% Simulation Loop 
for k = 1:N-1
    % Vehicle acceleration from ref speed 
    a_des = (V_ref(k+1) - v(k))/dt;

    % Resistive forces 
    F_roll = m*g*Cr;
    F_drag = 0.5*rho_air*Cd*A*v(k)^2;
    
    %  Required wheel torque 
    F_req = m*a_des + F_roll + F_drag;
    Te_req = F_req*r_wheel;

    %  Regenerative braking logic 
    if Te_req < 0 % Braking
        Te_m = max(Te_req, -Te_max);  % Limited by motor torque
        Tb_m = 0;                      % Assume mechanical brake assists if needed
    else
        Te_m = min(Te_req, Te_max);    % Motoring
        Tb_m = 0;
    end

    % Motor dynamics (
    omega(k+1) = v(k+1)/r_wheel;  % Approximate rotor speed from vehicle speed
    Te(k) = Te_m; 
    Tb(k) = Tb_m;
    
    %  Power regenerated 
    if Te_m < 0
        Pelec = -Te_m*v(k)/r_wheel;  % Electrical power generated
        Pelec = Pelec*eta_batt;       % Battery charge efficiency
        P_rec(k) = Pelec;
        % Update SOC
        SOC_t(k+1) = SOC_t(k) + Pelec*dt/(C_batt*3600*V_batt);
        SOC_t(k+1) = min(max(SOC_t(k+1), SOC_min), SOC_max);
    else
        SOC_t(k+1) = SOC_t(k);
    end
    
    % Vehicle speed update 
    a_net = (Te_m + Tb_m)/(m*r_wheel) - (F_roll + F_drag)/m;
    v(k+1) = v(k) + a_net*dt;
end

%% Plots 
figure('Name','Regenerative Braking Simulation','Position',[100 100 1200 600]);

subplot(3,1,1);
plot(time,v,'LineWidth',1.5); hold on;
plot(time,V_ref,'--','LineWidth',1.2);
ylabel('Vehicle Speed [m/s]'); xlabel('Time [s]');
legend('Actual','Reference'); grid on; title('Vehicle Speed');

subplot(3,1,2);
plot(time,Te,'LineWidth',1.5); hold on;
plot(time,Tb,'--','LineWidth',1.2);
ylabel('Motor/Brake Torque [Nm]'); xlabel('Time [s]');
legend('Motor Torque','Brake Torque'); grid on; title('Torque Profile');

subplot(3,1,3);
plot(time,SOC_t,'LineWidth',1.5);
ylabel('Battery SOC'); xlabel('Time [s]'); grid on; title('State of Charge');

figure('Name','Regenerated Power','Position',[100 100 1000 400]);
plot(time,P_rec,'LineWidth',1.5);
ylabel('Recovered Power [W]'); xlabel('Time [s]'); grid on; title('Regenerative Power');
