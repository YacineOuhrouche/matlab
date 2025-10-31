
% Field-Oriented Control (FOC) for PMSM - MATLAB script 


clear; close all; clc;

%% -------------------- Motor & Drive Parameters -------------------------
% PMSM electrical parameters
Rs   = 0.01;        % Stator resistance (Ohm)
Ld   = 0.0005;      % d-axis inductance (H)
Lq   = 0.0005;      % q-axis inductance (H) (surface-mounted: Ld ~ Lq)
lambda_m = 0.015;   % Flux linkage (Wb-turn) (per-phase)
pole_pairs = 4;     % number of pole pairs
J = 0.02;           % Rotor inertia (kg*m^2)
B = 0.001;          % Viscous friction coefficient (N*m*s)

% Inverter/DC link
Vdc = 300;          % DC bus voltage (V)

%% -------------------- Simulation Parameters ----------------------------
fsim = 10000;       % simulation sample frequency (Hz) for control loop
dt = 1/fsim;
Tsim = 1.2;         % total simulation time (s)
time = 0:dt:Tsim;
N = numel(time);

% Switching (SVPWM) period (we use averaged voltages per dt)
fsw = 10000;        % switching frequency for SVPWM reference (Hz)
Ts = 1/fsw;

%% -------------------- Control Gains (tune these) -----------------------
% Current controllers (inner loop) (PI)
Kp_id = 30; Ki_id = 800;    % id controller
Kp_iq = 30; Ki_iq = 800;    % iq controller

% Speed controller (outer loop) (PI)
Kp_spd = 0.5; Ki_spd = 10;

% Saturation limits
Vdq_limit = Vdc/sqrt(3);  % approximate max line-to-neutral voltage achievable
Iq_max = 200;        % A (example)
Id_max = 100;        % A

%% -------------------- References --------------------------------------
speed_ref_rpm = 1500;               % desired mechanical speed (rpm)
speed_ref = speed_ref_rpm*(2*pi)/60; % rad/s (mechanical)

% For a torque step test, could change speed_ref over time.
% Here we keep constant.

%% -------------------- Preallocate signals -----------------------------
ia = zeros(1,N); ib = zeros(1,N); ic = zeros(1,N);
id = zeros(1,N); iq = zeros(1,N);
vd = zeros(1,N); vq = zeros(1,N);
vdq_ref = zeros(2,N);
theta_r = zeros(1,N); omega_r = zeros(1,N);
torque = zeros(1,N);
speed_mech = zeros(1,N);  % mechanical angular speed (rad/s)
omega_e = zeros(1,N);      % electrical rad/s
dc_current = zeros(1,N);   % approximate DC current

% PI controller integrators
int_id = 0; int_iq = 0; int_spd = 0;

% initial conditions
speed_mech(1) = 0;      % start at zero speed
theta_r(1) = 0;

% initial currents
id(1) = 0; iq(1) = 0;

%% -------------------- Helper Functions (inline) -----------------------
clarke = @(a,b,c) (2/3)*[ a - 0.5*(b+c); (sqrt(3)/2)*(b-c) ]; % returns [alpha; beta]
park   = @(alpha,beta,theta) [ cos(theta) sin(theta); -sin(theta) cos(theta) ] * [alpha; beta];
invPark = @(vd,vq,theta) [ cos(theta) -sin(theta); sin(theta) cos(theta) ] * [vd; vq]; % gives [v_alpha; v_beta]
invClarke = @(valpha,vbeta) [ valpha; -0.5*valpha + (sqrt(3)/2)*vbeta; -0.5*valpha - (sqrt(3)/2)*vbeta ];

% SVPWM routine: compute phase duty cycles [Ta Tb Tc] in [0,1] for switching period Ts
function duty = svpwm_from_vab(valpha, vbeta, Vdc, Ts)
    % Convert valpha,vbeta to Vref and angle
    Vref = sqrt(valpha^2 + vbeta^2);
    angle = atan2(vbeta, valpha); % [-pi, pi]
    % Map angle to [0, 2*pi)
    if angle < 0, angle = angle + 2*pi; end
    sector = floor(angle / (pi/3)) + 1; % 1..6
    % relative angle within sector
    theta_s = angle - (sector-1)*pi/3;
    % Compute T1 and T2 using standard SVPWM formulas
    T1 = (sqrt(3)*Ts*(Vref/Vdc))*sin(pi/3 - theta_s);
    T2 = (sqrt(3)*Ts*(Vref/Vdc))*sin(theta_s);
    T0 = Ts - T1 - T2;
    % Compute switching times Ta,Tb,Tc depending on sector (center-aligned)
    switch sector
        case 1
            Ta = (T1+T2+T0/2)/Ts;
            Tb = (T2+T0/2)/Ts;
            Tc = (T0/2)/Ts;
        case 2
            Ta = (T1+T0/2)/Ts;
            Tb = (T1+T2+T0/2)/Ts;
            Tc = (T0/2)/Ts;
        case 3
            Ta = (T0/2)/Ts;
            Tb = (T1+T2+T0/2)/Ts;
            Tc = (T2+T0/2)/Ts;
        case 4
            Ta = (T0/2)/Ts;
            Tb = (T1+T0/2)/Ts;
            Tc = (T1+T2+T0/2)/Ts;
        case 5
            Ta = (T2+T0/2)/Ts;
            Tb = (T0/2)/Ts;
            Tc = (T1+T2+T0/2)/Ts;
        case 6
            Ta = (T1+T2+T0/2)/Ts;
            Tb = (T0/2)/Ts;
            Tc = (T1+T0/2)/Ts;
        otherwise
            Ta = 0.5; Tb = 0.5; Tc = 0.5;
    end
    duty = [Ta; Tb; Tc];
end

%% -------------------- Main simulation loop -----------------------------
for k = 1:N-1
    t = time(k);
    
    % electrical rotor angle and speed (mechanical->electrical)
    theta_r(k) = theta_r(k); % already stored
    omega_e(k) = pole_pairs * speed_mech(k); % electrical rad/s
    
    % --- Outer speed PI controller (mechanical speed) ---
    speed_err = speed_ref - speed_mech(k);
    int_spd = int_spd + speed_err * dt;
    iq_ref = Kp_spd*speed_err + Ki_spd*int_spd;
    % clamp iq_ref
    iq_ref = max(min(iq_ref, Iq_max), -Iq_max);
    id_ref = 0;  % field-oriented setpoint (can be used for flux weakening)
    
    % --- Inner current PI controllers (d-q frame) ---
    % Current errors
    err_id = id_ref - id(k);
    err_iq = iq_ref - iq(k);
    int_id = int_id + err_id*dt;
    int_iq = int_iq + err_iq*dt;
    % PI control -> voltage references in dq
    vd_ref = Kp_id*err_id + Ki_id*int_id;
    vq_ref = Kp_iq*err_iq + Ki_iq*int_iq;
    % limit v_ref to available magnitude
    Vdq_mag = sqrt(vd_ref^2 + vq_ref^2);
    if Vdq_mag > Vdq_limit
        scale = Vdq_limit / Vdq_mag;
        vd_ref = vd_ref * scale;
        vq_ref = vq_ref * scale;
    end
    vdq_ref(:,k) = [vd_ref; vq_ref];
    
    % --- Inverse Park: get alpha-beta voltages ---
    v_ab = invPark(vd_ref, vq_ref, theta_r(k)); % returns [v_alpha; v_beta]
    % --- SVPWM: map to phase duty cycles (Ta,Tb,Tc) ---
    duty = svpwm_from_vab(v_ab(1), v_ab(2), Vdc, Ts);
    % clamp duties [0,1]
    duty = max(min(duty,1),0);
    
    % --- approximate phase voltages (averaged) from duty & Vdc ---
    % line-to-neutral approximate voltages:
    % v_phase = (2*duty -1) * Vdc/2  (centered modulation)
    va = (2*duty(1)-1)*(Vdc/2);
    vb = (2*duty(2)-1)*(Vdc/2);
    vc = (2*duty(3)-1)*(Vdc/2);
    
    % store voltages for plotting
    % transform phase voltages back to dq to compute electrical derivatives
    % compute alpha-beta from phase voltages (inverse Clarke)
    [valpha, vbeta] = deal( (2/3)*(va - 0.5*(vb+vc)), (2/3)*( (sqrt(3)/2)*(vb - vc) ) );
    % Park transform alpha-beta -> dq
    Tparkm = [ cos(theta_r(k)) sin(theta_r(k)); -sin(theta_r(k)) cos(theta_r(k)) ];
    vd(k) = Tparkm(1,:)*[valpha; vbeta];
    vq(k) = Tparkm(2,:)*[valpha; vbeta];
    
    % --- Electrical dynamics (averaged dq model) ---
    % di_d/dt = (1/Ld) * ( v_d - R i_d + omega_e Lq i_q )
    % di_q/dt = (1/Lq) * ( v_q - R i_q - omega_e (Ld i_d + lambda_m) )
    di_d = (1/Ld) * ( vd(k) - Rs*id(k) + omega_e(k)*Lq*iq(k) );
    di_q = (1/Lq) * ( vq(k) - Rs*iq(k) - omega_e(k)*(Ld*id(k) + lambda_m) );
    % Euler integration
    id(k+1) = id(k) + di_d*dt;
    iq(k+1) = iq(k) + di_q*dt;
    
    % --- Electromagnetic torque (electrical torque) ---
    Te = (3/2)*pole_pairs*( lambda_m*iq(k) + (Ld - Lq)*id(k)*iq(k) ); % N*m
    torque(k) = Te;
    
    % --- Mechanical dynamics (simple rotor) ---
    % d omega/dt = (Te - B*omega - LoadTorque)/J
    % Example load torque - here we set small load or speed-dependent load
    LoadTorque = 0.5*sign(speed_mech(k)); % placeholder constant friction/clutch
    domega = (Te - B*speed_mech(k) - LoadTorque)/J;
    speed_mech(k+1) = speed_mech(k) + domega*dt;
    theta_r(k+1) = theta_r(k) + speed_mech(k)*dt; % mechanical angle (rad)
    
    % --- approximate DC link current (power balance): Idc ~ (3/2Vdc)*(v_a* i_a + v_b*i_b + v_c*i_c)
    % Approximate phase currents from inverse park/clarke: convert id,iq to ia,ib,ic
    % inverse Park: v_alpha_beta -> v_abc etc. We invert transforms for currents:
    % compute alpha-beta currents from dq currents
    idq = [id(k); iq(k)];
    % inverse Park for currents (dq->alpha-beta)
    invP = [ cos(theta_r(k)) -sin(theta_r(k)); sin(theta_r(k)) cos(theta_r(k)) ];
    i_alpha_beta = invP * idq; % [ialpha; ibeta]
    % inverse Clarke -> ia,ib,ic
    ia(k) = i_alpha_beta(1);
    ib(k) = -0.5*i_alpha_beta(1) + (sqrt(3)/2)*i_alpha_beta(2);
    ic(k) = -0.5*i_alpha_beta(1) - (sqrt(3)/2)*i_alpha_beta(2);
    % instantaneous electrical power (approx)
    Pelec = va*ia(k) + vb*ib(k) + vc*ic(k);
    dc_current(k) = Pelec / Vdc;
    
end

% last sample store
theta_r(N) = theta_r(N-1);
omega_e(N) = pole_pairs * speed_mech(N);

%% -------------------- Plots -------------------------------------------
figure('Name','FOC: Currents and References','NumberTitle','off');
subplot(3,1,1)
plot(time, id, 'b', 'LineWidth',1.2); hold on;
plot(time, zeros(size(time)),'k--');
ylabel('i_d (A)'); legend('i_d','i_d^*'); grid on;

subplot(3,1,2)
plot(time, iq, 'r', 'LineWidth',1.2); hold on;
plot(time, iq_ref*ones(size(time)),'k--');
ylabel('i_q (A)'); legend('i_q','i_q^*'); grid on;

subplot(3,1,3)
plot(time, torque,'m','LineWidth',1.2);
ylabel('Torque (N.m)'); xlabel('Time (s)'); grid on;

figure('Name','Speed & DC current','NumberTitle','off');
subplot(2,1,1)
plot(time, speed_mech*60/(2*pi),'LineWidth',1.2); ylabel('Speed (rpm)'); grid on;
subplot(2,1,2)
plot(time, dc_current,'LineWidth',1.2); ylabel('I_{dc} (A)'); xlabel('Time (s)'); grid on;

figure('Name','Phase voltages (approx)','NumberTitle','off');
plot(time, (2*(2/3)*( (2/3) ) )*0 + 0); % placeholder to set figure
hold on;
plot(time, repmat( (2*(0.5)-1)*(Vdc/2), size(time) ), 'k'); % placeholder
plot(time, va*ones(size(time)),'b'); plot(time, vb*ones(size(time)),'r'); plot(time, vc*ones(size(time)),'g');
xlabel('Time (s)'); ylabel('Phase voltage (V)'); legend('va','vb','vc'); grid on;

disp('Simulation complete.');

%% -------------------- Notes / Extensions ------------------------------
% - Sensorless estimation: implement a back-EMF observer or EKF to estimate
%   rotor speed/angle. Replace theta_r(k) with estimated angle for sensorless FOC.
% - Add mechanical load profile or drive cycle for speed_ref changes.
% - Replace average-phase voltage approx with switching-level model (if needed).
% - Tune PI gains for desired bandwidth. Use anti-windup on integrators for real code.
