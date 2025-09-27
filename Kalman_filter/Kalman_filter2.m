clc; clear; close all;

%% Parameters
dt = 0.1;               % Time step (s)
T  = 20;                % Total time (s)
N  = T/dt;              % Number of steps

%% True system (object moving in a circle)
r = 10;                 % Radius
omega = 0.2;            % Angular velocity (rad/s)

%% EKF State-space model
% State vector: [x; y; vx; vy]
Q = 0.05 * eye(4);   % Process noise covariance
R = 0.5 * eye(2);    % Measurement noise covariance
H = [1 0 0 0; 0 1 0 0];  % Measurement matrix (position only)

%% Initialization
x_est = [r; 0; 0; r*omega]; % Initial EKF estimate
P = eye(4);                  % Initial covariance

%% Storage
true_states = zeros(4,N);
meas = zeros(2,N);
estimates = zeros(4,N);

%% Helper function for Gaussian noise
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
    
    % --- Measurement (noisy position only) ---
    z = H * x_true + randn_cov(R,1);
    meas(:,k) = z;
    
    % --- EKF Prediction ---
    % Nonlinear state transition (circular motion approximation)
    f = @(x) [x(1) + dt*x(3);
              x(2) + dt*x(4);
             -omega^2*x(1)*dt + x(3);
             -omega^2*x(2)*dt + x(4)];
    
    % Jacobian of f wrt x
    F = [1 0 dt 0;
         0 1 0 dt;
        -omega^2*dt 0 1 0;
         0 -omega^2*dt 0 1];
    
    % Prediction
    x_pred = f(x_est);
    P_pred = F * P * F' + Q;
    
    % --- Measurement Update ---
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (z - H*x_pred);
    P = (eye(4) - K*H) * P_pred;
    
    % Store estimates
    estimates(:,k) = x_est;
end

%% --- Visualization ---

t = 0:dt:T-dt;

% Position tracking
figure; hold on; grid on; axis equal;
plot(true_states(1,:), true_states(2,:), 'b-', 'LineWidth', 2);
plot(meas(1,:), meas(2,:), 'rx', 'MarkerSize', 6);
plot(estimates(1,:), estimates(2,:), 'g--', 'LineWidth', 2);
legend('True Path','Measurements','EKF Estimate');
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('2D Circular Motion Tracking with EKF');

% Velocity tracking
figure;
subplot(2,1,1);
plot(t, true_states(3,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates(3,:), 'g--', 'LineWidth', 2);
legend('True vx','EKF vx'); xlabel('Time (s)'); ylabel('Velocity X (m/s)');

subplot(2,1,2);
plot(t, true_states(4,:), 'b-', 'LineWidth', 2); hold on; grid on;
plot(t, estimates(4,:), 'g--', 'LineWidth', 2);
legend('True vy','EKF vy'); xlabel('Time (s)'); ylabel('Velocity Y (m/s)');
