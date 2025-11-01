%%  Regenerative Braking Simulation — Enhanced Realistic Model


clear; close all; clc;


%%  Vehicle Parameters
m       = 1500;      
r_wheel = 0.3;         % Wheel radius 
g       = 9.81;        
Cd      = 0.32;        %  Aerodynamic drag coefficient
A       = 2.2;         
rho_air = 1.225;       % Air  density 
Cr      = 0.01;        %   Rolling resistance coefficient
theta   = 0;           % Road slope ]
gear_ratio = 9;        % Gear reduction ratio



%% Motor Parameters 
p        = 4;          % Pole pairs
Rs       = 0.03;       % Stator resistance 
Ld       = 0.0006;     % d-axis inductance 
Lq       = 0.0007;     % q-axis inductance 
lambda_m = 0.015;      % Permanent magnet flux 
Iq_max   = 180;        % Max q-axis current 
Id_max   = 50;         % Max  d-axis current 
Te_max   = 250;        % Max motor torque 
eta_motor = 0.95;      % Motor eff
m_motor   = 15;        % Motor mass for thermal mod
c_motor   = 500;       %  Specific heat cap
T_motor0  = 25;        % Initial motor temp 



%% Battery Parameters 
V_nom    = 300;       
C_batt   = 50;         % Cap Ah
SOC      = 0.8;        % Ini SOC
SOC_min  = 0.2;        
SOC_max  = 1.0;        
I_max    = 100;        
eta_batt = 0.9;        % Bat effi
eta_inv  = 0.95;       % Inverter eff
R_batt   = 0.05;       % Internal battery resistance 



%% Simulation Parameters 
fsim = 1000;         
dt   = 1/fsim;         
Tsim = 30;              
time = 0:dt:Tsim;
N    = length(time);



%%  Drive Cycle 
% WLTP-like realistic cycle (simplified approximation)
V_ref = zeros(1,N);
for k = 1:N
    t = time(k);
    if t < 5
        V_ref(k) = 0.5*t;          % Accelerate to 2.5 m/s
    elseif t < 12
        V_ref(k) = 2.5 + 1.5*sin(pi*(t-5)/7); % Moderate speed variations
    elseif t < 20
        V_ref(k) = 4 + 1*sin(pi*(t-12)/8);    % Cruise / small oscillations
    else
        V_ref(k) = max(0,4 - 0.5*(t-20));    % Deceleration
    end
end
V_max = max(V_ref);



%%  Preallocate Variables 
v      = zeros(1,N);   
omega  = zeros(1,N);   
Te     = zeros(1,N);   
Tb     = zeros(1,N);   
P_rec  = zeros(1,N);   
SOC_t  = zeros(1,N);   
E_rec  = zeros(1,N);   
P_loss_motor = zeros(1,N); 
P_loss_inv   = zeros(1,N); 
T_motor      = T_motor0*ones(1,N); 

v(1)     = V_ref(1); 
SOC_t(1) = SOC;



%%  Simulation Loop 
for k = 1:N-1
    
    %% Vehicle acceleration
    a_des = (V_ref(k+1) - v(k))/dt;
    
    %% Resistive forces
    F_roll = m*g*Cr*cos(theta);
    F_slope = m*g*sin(theta);
    F_drag = 0.5*rho_air*Cd*A*v(k)^2;
    
    %% Required wheel torque
    F_req = m*a_des + F_roll + F_drag + F_slope;
    Te_req = F_req * r_wheel / gear_ratio;
    
    %% Braking Control
    if Te_req < 0
        % Regenerative braking
        Te_m = max(Te_req, -Te_max);
        Tb_m = max(0, abs(Te_req) - abs(Te_m)); % Mechanical supplement
    else
        Te_m = min(Te_req, Te_max);
        Tb_m = 0;
    end
    
    %%  Motor Dynamics
    omega(k+1) = v(k)/r_wheel * gear_ratio; 
    Te(k) = Te_m;
    Tb(k) = Tb_m;
    
    % Electrical currents 
    Iq = Te_m / (1.5 * p * lambda_m); 
    Iq = max(min(Iq, Iq_max), -Iq_max);
    
    %%  Battery Voltage & SOC 
    V_batt = V_nom - Iq*R_batt; % SOC-dependent voltage
    if Te_m < 0
        Pelec = -Te_m*v(k)/(r_wheel*eta_inv); 
        Pelec = Pelec*eta_batt;
        Pelec = min(Pelec, I_max*V_batt); % Charge current limit
        P_rec(k) = Pelec;
        
        SOC_t(k+1) = SOC_t(k) + Pelec*dt/(C_batt*3600*V_batt);
        SOC_t(k+1) = min(max(SOC_t(k+1), SOC_min), SOC_max);
        E_rec(k+1) = E_rec(k) + Pelec*dt;
    else
        Pelec = 0;
        SOC_t(k+1) = SOC_t(k);
        P_rec(k) = 0;
        E_rec(k+1) = E_rec(k);
    end
    
    %%  Thermal Losses 
    P_loss_motor(k) = (Te_m^2)/Te_max * (1-eta_motor)*abs(Te_m); 
    P_loss_inv(k)   = Pelec*(1-eta_inv); 
    
    % Motor temperature update
    T_motor(k+1) = T_motor(k) + P_loss_motor(k)*dt/(m_motor*c_motor);
    
    %% Vehicle Speed Update
    a_net = (Te_m + Tb_m)*gear_ratio/(m*r_wheel) - (F_roll + F_drag + F_slope)/m;
    v(k+1) = v(k) + a_net*dt;
end



%% Plots 
figure('Name','Regenerative Braking Simulation','Position',[100 100 1200 800]);

subplot(5,1,1);
plot(time,v,'LineWidth',1.5); hold on;
plot(time,V_ref,'--','LineWidth',1.2);
ylabel('Vehicle Speed [m/s]'); xlabel('Time [s]');
legend('Actual','Reference'); grid on; title('Vehicle Speed');

subplot(5,1,2);
plot(time,Te,'LineWidth',1.5); hold on;
plot(time,Tb,'--','LineWidth',1.2);
ylabel('Motor/Brake Torque [Nm]'); xlabel('Time [s]');
legend('Motor Torque','Brake Torque'); grid on; title('Torque Profile');

subplot(5,1,3);
plot(time,SOC_t,'LineWidth',1.5);
ylabel('Battery SOC'); xlabel('Time [s]'); grid on; title('State of Charge');

subplot(5,1,4);
plot(time,E_rec,'LineWidth',1.5);
ylabel('Cumulative Energy Recovered [J]'); xlabel('Time [s]');
grid on; title('Energy Recovery');

subplot(5,1,5);
plot(time,T_motor,'LineWidth',1.5); grid on;
ylabel('Motor Temp [C]'); xlabel('Time [s]'); title('Motor Temperature');

figure('Name','Power & Losses','Position',[100 100 1000 400]);
plot(time,P_rec,'LineWidth',1.5); hold on;
plot(time,P_loss_motor,'--','LineWidth',1.2);
plot(time,P_loss_inv,':','LineWidth',1.2);
ylabel('Power [W]'); xlabel('Time [s]');
legend('Recovered Power','Motor Loss','Inverter Loss'); grid on; title('Power and Thermal Losses');
