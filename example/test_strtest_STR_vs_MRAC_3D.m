clear; clc; close all;
addpath('../src');

% --- 1. Simulation Time Profile ---
t_total = 14;
dt = 0.04;
time_steps = 0:dt:t_total;

% --- 2. Pre-allocated Storage Buffers for Plotting ---
mrac_err_history = zeros(length(time_steps), 1);
str_err_history  = zeros(length(time_steps), 1);
m2_est_history   = zeros(length(time_steps), 1);

% Initialize Control States
q0 = robot_ik(1.2, 0.2); % Starting pose
x_mrac = [q0; 0; 0; q0; 0; 0; 20;0;0;20; 5;0;0;5]; 
x_str  = [q0; 0; 0; q0; 0; 0; 1.0; 1.0];

% --- 3. Setup Layout Figures ---
fig_perf = figure('Name', 'Adaptive Control Benchmark Performance', 'Position', [100, 100, 1100, 500], 'Color', 'w');

% Subplot 1: Real-Time Cartesian Error Comparisons
ax_err = subplot(1, 2, 1); hold on; grid on;
h_line_mrac_err = plot(ax_err, 0, 0, 'r', 'LineWidth', 1.5);
h_line_str_err  = plot(ax_err, 0, 0, 'm', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Total Position Error Magnitude (m)');
legend('Direct MRAC Error', 'Indirect STR Error');
title('Transient Tracking Error Profile');

% Subplot 2: STR Mass Identification Tracking
ax_param = subplot(1, 2, 2); hold on; grid on;
h_line_true_m = plot(ax_param, [0 t_total], [1.0 1.0], 'R--', 'LineWidth', 2);
h_line_est_m  = plot(ax_param, 0, 0, 'y', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Link 2 Mass Parameters (kg)');
legend('True Physical Mass Matrix Value', 'STR Online Mass Estimator (\theta_2)');
title('Indirect Parameter Convergence Profile');

disp('Running benchmark comparisons...');

% --- 4. Execution Sequence Loop ---
for k = 1:length(time_steps)
    t = time_steps(k);
    
    % Generate a unified, abrupt target step path profile based on time steps
    if t < 3
        mx = 1.2; my = 0.2;
    elseif t < 7
        mx = 0.5; my = 1.0;   % Sudden spatial jump!
    elseif t < 11
        mx = 0.7; my = -0.5;  % Sudden spatial jump!
    else
        mx = 1.4; my = 0.0;   % Sudden spatial jump!
    end
    
    % Step 1: Run Direct MRAC Physics Step
    [~, xs_mrac] = ode45(@(t, x) mrac_wrapper(t, x, mx, my), [t, t+dt], x_mrac);
    x_mrac = xs_mrac(end, :)';
    
    % Step 2: Run Indirect STR Physics Step
    [~, xs_str] = ode45(@(t, x) str_wrapper(t, x, mx, my), [t, t+dt], x_str);
    x_str = xs_str(end, :)';
    
    % --- Step 3: Performance Extraction & Analysis Math ---
    l1 = 1; l2 = 1;
    % MRAC Cartesian Positions
    xm_x = l1*cos(x_mrac(1)) + l2*cos(x_mrac(1)+x_mrac(2));
    xm_y = l1*sin(x_mrac(1)) + l2*sin(x_mrac(1)+x_mrac(2));
    
    % STR Cartesian Positions
    xs_x = l1*cos(x_str(1)) + l2*cos(x_str(1)+x_str(2));
    xs_y = l1*sin(x_str(1)) + l2*sin(x_str(1)+x_str(2));
    
    % Calculated Combined Error Vectors Magnitudes against target location
    mrac_err_history(k) = sqrt((mx - xm_x)^2 + (my - xm_y)^2);
    str_err_history(k)  = sqrt((mx - xs_x)^2 + (my - xs_y)^2);
    m2_est_history(k)   = x_str(10); % Extract estimated m2 parameter
    
    % --- Step 4: Dynamically Update Plots Data ---
    set(h_line_mrac_err, 'XData', time_steps(1:k), 'YData', mrac_err_history(1:k));
    set(h_line_str_err,  'XData', time_steps(1:k), 'YData', str_err_history(1:k));
    xlim(ax_err, [0, t_total]); ylim(ax_err, [0, 1.5]);
    
    % Handle the dynamic step jump representation of the true mass timeline
    if t > 7
        set(h_line_true_m, 'XData', [0, 7, 7, t_total], 'YData', [1.0, 1.0, 2.5, 2.5]);
    end
    set(h_line_est_m, 'XData', time_steps(1:k), 'YData', m2_est_history(1:k));
    xlim(ax_param, [0, t_total]); ylim(ax_param, [0, 3.5]);
    
    drawnow limitrate;
end
disp('Comparison test complete!');