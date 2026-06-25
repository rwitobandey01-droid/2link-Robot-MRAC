clear; clc; close all;
addpath('../src'); % Automatically link core math files

% --- 1. Global Variables for Mouse Tracking ---
global target_x target_y
target_x = 1.0; 
target_y = 0.5;

% --- 2. Setup Figure Layout ---
fig = figure('Name', '3D Interactive Self-Tuning Regulator', 'Position', [150, 150, 900, 750]);
set(fig, 'WindowButtonMotionFcn', @mouseMoveCallback); 
set(fig, 'Color', 'w');

ax3d = axes('Parent', fig);
view(ax3d, -30, 30); grid on; hold on;
axis([-2.5 2.5 -2.5 2.5 -0.5 1.5]);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% Draw a grid floor
[Xf, Yf] = meshgrid(-2.5:0.5:2.5, -2.5:0.5:2.5);
Zf = zeros(size(Xf)) - 0.05;
surf(ax3d, Xf, Yf, Zf, 'FaceColor', [0.95 0.95 0.95], 'EdgeColor', [0.85 0.85 0.85]);

% --- Initialize Graphic Objects ---
h_target = plot3(ax3d, target_x, target_y, 0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
h_path_str = plot3(ax3d, 0, 0, 0, 'm-', 'LineWidth', 1.5); % STR path trail

% STR ROBOT (Solid Magenta/Black)
h_link1  = plot3(ax3d, [0,0], [0,0], [0,0], 'm', 'LineWidth', 8);
h_link2  = plot3(ax3d, [0,0], [0,0], [0,0], 'k', 'LineWidth', 8);
h_joints = plot3(ax3d, [0,0,0], [0,0,0], [0,0,0], 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

legend(ax3d, [h_target, h_path_str], {'Mouse Target', 'STR Trajectory Trail'}, 'Location', 'northwest');
h_title = title(ax3d, 'Interactive 3D STR Tracker');

% --- 3. Initialize STR Physics States ---
% States: [q1; q2; dq1; dq2; qm1; qm2; dqm1; dqm2; m1_est; m2_est]
q0 = robot_ik(target_x, target_y);
x_str = [q0; 0; 0; q0; 0; 0; 1.0; 1.0]; % Initial mass guesses set to 1.0kg
t = 0; dt = 0.04;

xstr_history = []; ystr_history = [];

disp('STR Simulation running! Move your mouse inside the figure window.');

% --- 4. Real-Time Loop ---
while ishandle(fig)
    % Step STR physics forward using str_wrapper
    [ts, xs] = ode45(@(t, x) str_wrapper(t, x, target_x, target_y), [t, t+dt], x_str);
    x_str = xs(end, :)';
    t = t + dt;
    
    % Actual STR Kinematics
    q1 = x_str(1); q2 = x_str(2);
    l1 = 1; l2 = 1;
    x1_act = l1 * cos(q1);          y1_act = l1 * sin(q1);
    x2_act = x1_act + l2 * cos(q1+q2);  y2_act = y1_act + l2 * sin(q1+q2);
    
    % Append History for Path Trail
    xstr_history = [xstr_history, x2_act]; ystr_history = [ystr_history, y2_act];
    if length(xstr_history) > 100
        xstr_history(1) = []; ystr_history(1) = [];
    end
    
    % --- Real-Time Command Window Printout ---
    m1_est = x_str(9); m2_est = x_str(10);
    fprintf('Time: %5.2fs | Estimated Masses -> m1: %4.2f kg, m2: %4.2f kg\n', t, m1_est, m2_est);
    
    % --- Update Graphics ---
    set(h_target, 'XData', target_x, 'YData', target_y);
    set(h_path_str, 'XData', xstr_history, 'YData', ystr_history, 'ZData', zeros(size(xstr_history)));
    set(h_link1,  'XData', [0, x1_act], 'YData', [0, y1_act]);
    set(h_link2,  'XData', [x1_act, x2_act], 'YData', [y1_act, y2_act]);
    set(h_joints, 'XData', [0, x1_act, x2_act], 'YData', [0, y1_act, y2_act]);
    
    if t > 7
        set(h_title, 'String', ['STR Running [PAYLOAD ADDED] - Time: ' num2str(t, '%.2f') 's']);
    else
        set(h_title, 'String', ['STR Running [Normal Mass] - Time: ' num2str(t, '%.2f') 's']);
    end
    
    drawnow limitrate;
end

function mouseMoveCallback(~, ~)
    global target_x target_y
    ax = gca; C = get(ax, 'CurrentPoint');
    target_x = C(1, 1); target_y = C(1, 2);
end