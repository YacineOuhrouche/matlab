
%  2-Link Planar Manipulator – Forward Kinematics Simulation


clear; close all; clc;

%%  Link Parameters
L1 = 1.0;      % Length of Link 1 
L2 = 0.7;      % Length of Link 2 

% Define joint angle ranges (in radians)
theta1 = linspace(-pi/2, pi/2, 50);   % Joint 1 rotation
theta2 = linspace(-pi/2, pi/2, 50);   % Joint 2 rotation

%% Forward Kinematics 
% Using Denavit–Hartenberg approach for a 2-link planar arm

% End-effector position arrays
X = zeros(length(theta1), length(theta2));
Y = zeros(length(theta1), length(theta2));

for i = 1:length(theta1)
    for j = 1:length(theta2)
        % Transformation matrices
        T1 = [cos(theta1(i)) -sin(theta1(i)) 0 L1*cos(theta1(i));
              sin(theta1(i))  cos(theta1(i)) 0 L1*sin(theta1(i));
              0                0              1 0;
              0                0              0 1];
        
        T2 = [cos(theta2(j)) -sin(theta2(j)) 0 L2*cos(theta2(j));
              sin(theta2(j))  cos(theta2(j)) 0 L2*sin(theta2(j));
              0                0              1 0;
              0                0              0 1];
        
        % Compute overall transformation
        T = T1 * T2;
        
        % End-effector position
        X(i,j) = T(1,4);
        Y(i,j) = T(2,4);
    end
end

%%  Workspace Visualization 
figure('Color','w');
surf(X, Y, zeros(size(X)),  'FaceAlpha', 0.7, 'EdgeColor', 'none');
hold on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('2-Link Planar Manipulator Workspace');
grid on; axis equal;
colormap jet;

%%  Animation Setup 
figure('Color','w');
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)');
title('2-Link Planar Manipulator – Forward Kinematics Animation');
xlim([-2 2]); ylim([-2 2]);

% Draw base
plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

% Initialize plot handles
link1 = plot([0 0], [0 0], 'LineWidth', 3, 'Color', [0.1 0.5 0.9]);
link2 = plot([0 0], [0 0], 'LineWidth', 3, 'Color', [0.3 0.7 0.3]);
joint_dot = plot(0, 0, 'ko', 'MarkerFaceColor', [0.2 0.2 0.2]);
ee_dot = plot(0, 0, 'ro', 'MarkerFaceColor', 'r');
trace = animatedline('Color', [0.9 0.1 0.1], 'LineWidth', 1.5);

% Create text display for angles and coordinates
info_text = text(-1.9, 1.7, '', 'FontSize', 10, 'FontWeight', 'bold', 'Color', [0 0 0]);

for k = 1:length(theta1)
    % Joint angles
    t1 = theta1(k);
    t2 = theta2(k);

    % Joint positions
    joint1 = [L1*cos(t1); L1*sin(t1)];
    end_eff = [L1*cos(t1) + L2*cos(t1 + t2);
               L1*sin(t1) + L2*sin(t1 + t2)];
    
    % Update links and points
    set(link1, 'XData', [0 joint1(1)], 'YData', [0 joint1(2)]);
    set(link2, 'XData', [joint1(1) end_eff(1)], 'YData', [joint1(2) end_eff(2)]);
    set(joint_dot, 'XData', joint1(1), 'YData', joint1(2));
    set(ee_dot, 'XData', end_eff(1), 'YData', end_eff(2));
    
    % Update trace
    addpoints(trace, end_eff(1), end_eff(2));
    
    % Update info text
    set(info_text, 'String', sprintf('\\theta_1 = %.2f rad\n\\theta_2 = %.2f rad\nX = %.2f m\nY = %.2f m', ...
        t1, t2, end_eff(1), end_eff(2)));
    
    drawnow;
    pause(0.03);
end

disp('Simulation complete.');
