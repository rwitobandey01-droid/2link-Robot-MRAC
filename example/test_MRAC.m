clear; clc; close all;

tspan = [0 35]; 
q0 = robot_ik(0.8, 0.5); % Start at target position
q0 = [q0; 0; 0];
qm0 = q0; 
Kp_init = [10; 0; 0; 10]; Kd_init = [2; 0; 0; 2];   
x0 = [q0; qm0; Kp_init; Kd_init];

disp('Simulating...');
[t, x] = ode45(@mrac_wrapper, tspan, x0);
disp('Done! Starting Real-Time Visuals...');

% --- Data Extraction & Forward Kinematics ---
q1_sim = x(:,1);  q2_sim = x(:,2);
qm1_sim = x(:,5); qm2_sim = x(:,6);
l1 = 1; l2 = 1;

% Actual robot hand trajectory
x1 = l1 * cos(q1_sim);          y1 = l1 * sin(q1_sim);
x2 = x1 + l2 * cos(q1_sim+q2_sim); y2 = y1 + l2 * sin(q1_sim+q2_sim);

% Reference model hand trajectory
xm1 = l1 * cos(qm1_sim);          ym1 = l1 * sin(qm1_sim);
xm2 = xm1 + l2 * cos(qm1_sim + qm2_sim); ym2 = ym1 + l2 * sin(qm1_sim + qm2_sim);

% Re-calculate target circle path for drawing references
cx = 0.5; cy = 0.5; radius = 0.3;
th = linspace(0, 2*pi, 100);
cx_path = cx + radius * cos(th); cy_path = cy + radius * sin(th);

% =========================================================================
% --- INITIALIZE REAL-TIME PLOTS WINDOW ---
% =========================================================================
figure('Position', [100, 100, 1200, 500]); % Wide screen layout

% Subplot 1: The Live 2-Link Robot Animation
subplot(1, 2, 1);
plot(cx_path, cy_path, 'k--', 'LineWidth', 1); hold on;
h_target = plot(0, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
h_link1  = plot([0, 0], [0, 0], 'r-o', 'LineWidth', 4, 'MarkerSize', 6);
h_link2  = plot([0, 0], [0, 0], 'b-o', 'LineWidth', 4, 'MarkerSize', 6);
xlim([-0.5, 2.2]); ylim([-0.5, 2.2]); grid on;
xlabel('X Position (m)'); ylabel('Y Position (m)');
legend('Target Path', 'Moving Target', 'Link 1', 'Link 2', 'Location', 'northwest');
h_title = title('MRAC Visual Tracker - Time: 0.00s');

% Subplot 2: Real-time Cartesian Tracking Error
subplot(1, 2, 2);
h_err1 = plot(t(1), x2(1)-xm2(1), 'r', 'LineWidth', 1.5); hold on;
h_err2 = plot(t(1), y2(1)-ym2(1), 'b', 'LineWidth', 1.5);
xlim([0, tspan(end)]); ylim([-0.1, 0.1]); grid on;
xlabel('Time (s)'); ylabel('Cartesian Error (m)');
legend('X Position Error', 'Y Position Error');
title('Real-Time Tracking Error');

% =========================================================================
% --- ANIMATION & DATA UPDATE LOOP ---
% =========================================================================
for k = 1:5:length(t)
    % 1. Compute current target position
    tar_x = cx + radius * cos(t(k)); 
    tar_y = cy + radius * sin(t(k));
    
    % 2. Update graphic elements for the robot configuration
    set(h_target, 'XData', tar_x, 'YData', tar_y);
    set(h_link1,  'XData', [0, x1(k)], 'YData', [0, y1(k)]);
    set(h_link2,  'XData', [x1(k), x2(k)], 'YData', [y1(k), y2(k)]);
    set(h_title,  'String', ['MRAC Visual Tracker - Time: ' num2str(t(k), '%.2f') 's']);
    
    % 3. Update data arrays for error lines dynamically up to current time step k
    set(h_err1, 'XData', t(1:k), 'YData', x2(1:k) - xm2(1:k));
    set(h_err2, 'XData', t(1:k), 'YData', y2(1:k) - ym2(1:k));
    
    % Refresh the frame
    drawnow;
end