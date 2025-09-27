clc; clear; close all;

%% Parameters
dt = 0.1;               % Time step (s)
T  = 20;                % Total time (s)
N  = T/dt;              % Number of steps

%% True system (object moving in a circle)
r = 10;                 % Radius
omega = 0.2;            % Angular velocity (rad/s)

%% Linear KF State-space model (constant velocity model)
% State vector: [x; y; vx; vy]
A = [1 0 dt 0;
     0 1 0 dt;
     0 0 1  0;
     0 0 0  1];

H = [1 0 0 0;
     0 1 0 0];

Q = 0.05 * eye(4);   % Process noise covariance
R = 0.5 * eye(2);    % Measurement noise covariance

%% Initialization
x_est_kf = [r; 0; 0; r*omega];  % Initial KF estimate
P_kf = eye(4);

x_est_ekf = [r; 0; 0; r*omega]; % Initial EKF estimate
P_ekf = eye(4);

%% Storage
true_states = zeros(4,N);
meas = zeros(2,N);
estimates_kf = zeros(4,N);
estimates_ekf = zeros(4,N);
K_store_kf = zeros(4,2,N);

%% Helper: Gaussian noise generator
randn_cov = @(cov,n) chol(cov,'lower') * randn(size(cov,1),n);

%% Simulation loop
for k = 1:N
    t = (k-1)*dt;
    
    % --- True circular motion ---
    x_true = [r*cos(omega*t);
              r*sin(omega*t);
             -r*omega*sin(omega*t);
              r*omega*cos(omega*t)];
    true_states(:,k) = x_true;
    
    % --- Measurement (noisy position) ---
    z = H * x_true + randn_cov(R,1);
    meas(:,k) = z;
    
    % === Standard Kalman Filter (Linear Model) ===
    % Prediction
    x_pred = A * x_est_kf;
    P_pred = A * P_kf * A' + Q;
    
    % Update
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est_kf = x_pred + K * (z - H * x_pred);
    P_kf = (eye(4) - K*H) * P_pred;
    
    % Store
    estimates_kf(:,k) = x_est_kf;
    K_store_kf(:,:,k) = K;
    
    % === Extended Kalman Filter (EKF) ===
    % Nonlinear state transition (true dynamics: circular motion)
    f = @(x) [x(1) + dt*x(3);
              x(2) + dt*x(4);
             -omega^2*x(1)*dt + x(3);   % Approx nonlinear vx
             -omega^2*x(2)*dt + x(4)];  % Approx nonlinear vy
    % Jacobian of f wrt x
    F = [1 0 dt 0;
         0 1 0 dt;
        -omega^2*dt 0 1 0;
         0 -omega^2*dt 0 1];
    
    % Prediction
    x_pred = f(x_est_ekf);
    P_pred = F * P_ekf * F' + Q;
    
    % Measurement update
    Hk = H; % Measurement Jacobian (still linear in this case)
    K = P_pred * Hk' / (Hk * P_pred * Hk' + R);
    x_est_ekf = x_pred + K * (z - Hk * x_pred);
    P_ekf = (eye(4) - K*Hk) * P_pred;
    
    % Store
    estimates_ekf(:,k) = x_est_ekf;
end

%% --- Error Analysis ---
t = 0:dt:T-dt;
pos_rmse_kf  = sqrt(mean(sum((true_states(1:2,:) - estimates_kf(1:2,:)).^2,1)));
vel_rmse_kf  = sqrt(mean(sum((true_states(3:4,:) - estimates_kf(3:4,:)).^2,1)));
pos_rmse_ekf = sqrt(mean(sum((true_states(1:2,:) - estimates_ekf(1:2,:)).^2,1)));
vel_rmse_ekf = sqrt(mean(sum((true_states(3:4,:) - estimates_ekf(3:4,:)).^2,1)));

fprintf('KF  -> Position RMSE: %.3f m | Velocity RMSE: %.3f m/s\n', pos_rmse_kf, vel_rmse_kf);
fprintf('EKF -> Position RMSE: %.3f m | Velocity RMSE: %.3f m/s\n', pos_rmse_ekf, vel_rmse_ekf);

%% --- Visualization ---

% 1) Position Tracking
figure; hold on; grid on; axis equal;
plot(true_states(1,:), true_states(2,:), 'b-', 'LineWidth', 2);
plot(meas(1,:), meas(2,:), 'rx', 'MarkerSize', 6);
plot(estimates_kf(1,:), estimates_kf(2,:), 'g-', 'LineWidth', 2);
plot(estimates_ekf(1,:), estimates_ekf(2,:), 'm--', 'LineWidth', 2);
legend('True Path','Measurements','KF Estimate','EKF Estimate');
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('2D Circular Motion Tracking: KF vs EKF');

% 2) Velocity Tracking
figure;
subplot(2,1,1);
plot(t, true_states(3,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates_kf(3,:), 'g--', 'LineWidth', 2);
plot(t, estimates_ekf(3,:), 'm-.', 'LineWidth', 2);
legend('True vx','KF vx','EKF vx');
xlabel('Time (s)'); ylabel('Velocity X (m/s)');

subplot(2,1,2);
plot(t, true_states(4,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates_kf(4,:), 'g--', 'LineWidth', 2);
plot(t, estimates_ekf(4,:), 'm-.', 'LineWidth', 2);
legend('True vy','KF vy','EKF vy');
xlabel('Time (s)'); ylabel('Velocity Y (m/s)');

% 3) Estimation Errors
figure;
subplot(2,1,1);
plot(t, true_states(1,:) - estimates_kf(1,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, true_states(1,:) - estimates_ekf(1,:), 'm', 'LineWidth', 1.2);
legend('KF X error','EKF X error'); grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Error (X)');

subplot(2,1,2);
plot(t, true_states(2,:) - estimates_kf(2,:), 'r', 'LineWidth', 1.2); hold on;
plot(t, true_states(2,:) - estimates_ekf(2,:), 'm', 'LineWidth', 1.2);
legend('KF Y error','EKF Y error'); grid on;
xlabel('Time (s)'); ylabel('Error (m)');
title('Position Error (Y)');

% 4) Kalman Gain Evolution (KF only)
figure;
plot(squeeze(K_store_kf(1,1,:))); hold on;
plot(squeeze(K_store_kf(2,2,:)));
xlabel('Time step'); ylabel('Kalman Gain');
legend('K(1,1)','K(2,2)');
title('Kalman Gain Evolution (KF)');
grid on;
