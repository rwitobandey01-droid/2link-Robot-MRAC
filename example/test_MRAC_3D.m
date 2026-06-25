clear; clc; close all;

% --- 1. Global Variables for Mouse Tracking ---
global target_x target_y
target_x = 1.0; 
target_y = 0.5;

% --- 2. Setup Figure Layout ---
fig = figure('Name', '3D Interactive MRAC Tracker', 'Position', [150, 150, 900, 750]);
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

% Real-Time Trajectory Trails (The paths drawn on the floor)
h_path_ref = plot3(ax3d, 0, 0, 0, 'k--', 'LineWidth', 1.5);
h_path_act = plot3(ax3d, 0, 0, 0, 'r-', 'LineWidth', 1.5);

% REFERENCE MODEL ROBOT (Ghostly Gray)
h_ref_link1  = plot3(ax3d, [0,0], [0,0], [0,0], 'Color', [0.6 0.6 0.6 0.3], 'LineWidth', 6);
h_ref_link2  = plot3(ax3d, [0,0], [0,0], [0,0], 'Color', [0.6 0.6 0.6 0.3], 'LineWidth', 6);

% ACTUAL ADAPTIVE ROBOT (Solid Red/Blue)
h_act_link1  = plot3(ax3d, [0,0], [0,0], [0,0], 'r', 'LineWidth', 8);
h_act_link2  = plot3(ax3d, [0,0], [0,0], [0,0], 'b', 'LineWidth', 8);
h_joints     = plot3(ax3d, [0,0,0], [0,0,0], [0,0,0], 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);

legend(ax3d, [h_target, h_path_ref, h_path_act], ...
    {'Mouse Target', 'Model Path (Ideal)', 'Robot Path (Adaptive)'}, 'Location', 'northwest');
h_title = title(ax3d, 'Interactive 3D MRAC Tracker');

% --- 3. Initialize Physics ---
q0 = robot_ik(target_x, target_y);
x_current = [q0; 0; 0; q0; 0; 0; 20;0;0;20; 5;0;0;5]; 
t = 0; dt = 0.04;

% Path history buffers
xref_history = []; yref_history = [];
xact_history = []; yact_history = [];

disp('Simulation running! Move your mouse inside the figure window.');

% --- 4. The Real-Time Loop ---
while ishandle(fig)
    % Run physics step
    [ts, xs] = ode45(@(t, x) mrac_wrapper(t, x, target_x, target_y), [t, t+dt], x_current);
    x_current = xs(end, :)';
    t = t + dt;
    
    % --- Actual Robot Forward Kinematics ---
    q1 = x_current(1); q2 = x_current(2);
    l1 = 1; l2 = 1;
    x1_act = l1 * cos(q1);          y1_act = l1 * sin(q1);
    x2_act = x1_act + l2 * cos(q1+q2);  y2_act = y1_act + l2 * sin(q1+q2);
    
    % --- Reference Model Forward Kinematics ---
    qm1 = x_current(5); qm2 = x_current(6);
    x1_ref = l1 * cos(qm1);          y1_ref = l1 * sin(qm1);
    x2_ref = x1_ref + l2 * cos(qm1+qm2);  y2_ref = y1_ref + l2 * sin(qm1+qm2);

    % --- Extract and Print Adaptive Gains in Real-Time ---
    % Reconstruct the 2x2 matrices from the state vector
    Kp_current = reshape(x_current(9:12), [2, 2]);
    Kd_current = reshape(x_current(13:16), [2, 2]);
    
    % Print the diagonal main gains (Joint 1 and Joint 2 values)
    fprintf('Time: %5.2fs | Kp1: %6.1f, Kp2: %6.1f | Kd1: %5.1f, Kd2: %5.1f\n', ...
        t, Kp_current(1,1), Kp_current(2,2), Kd_current(1,1), Kd_current(2,2));
    
    % --- Append History for Trails ---
    xref_history = [xref_history, x2_ref]; yref_history = [yref_history, y2_ref];
    xact_history = [xact_history, x2_act]; yact_history = [yact_history, y2_act];
    
    % Keep the trail limited to the last 100 steps so the screen stays clean
    if length(xref_history) > 100
        xref_history(1) = []; yref_history(1) = [];
        xact_history(1) = []; yact_history(1) = [];
    end
    
    % --- Update 3D Graphics Elements ---
    set(h_target, 'XData', target_x, 'YData', target_y);
    
    % Update Path Trails on the floor plane (Z = 0)
    set(h_path_ref, 'XData', xref_history, 'YData', yref_history, 'ZData', zeros(size(xref_history)));
    set(h_path_act, 'XData', xact_history, 'YData', yact_history, 'ZData', zeros(size(xact_history)));
    
    % Update Ghost Model Linkage
    set(h_ref_link1, 'XData', [0, x1_ref], 'YData', [0, y1_ref]);
    set(h_ref_link2, 'XData', [x1_ref, x2_ref], 'YData', [y1_ref, y2_ref]);
    
    % Update Actual Robot Linkage
    set(h_act_link1, 'XData', [0, x1_act], 'YData', [0, y1_act]);
    set(h_act_link2, 'XData', [x1_act, x2_act], 'YData', [y1_act, y2_act]);
    set(h_joints,    'XData', [0, x1_act, x2_act], 'YData', [0, y1_act, y2_act]);
    
    set(h_title, 'String', ['Interactive 3D MRAC - Time: ' num2str(t, '%.2f') 's']);
    
    drawnow limitrate;
end

% --- Callback Function to Read Mouse ---
function mouseMoveCallback(~, ~)
    global target_x target_y
    ax = gca;
    C = get(ax, 'CurrentPoint');
    target_x = C(1, 1);
    target_y = C(1, 2);
end