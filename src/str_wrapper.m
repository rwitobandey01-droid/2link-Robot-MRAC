%Self Tunning Regulator:
function dx = str_wrapper(t, x, mouse_x, mouse_y)
    % Extract Robot States
    q   = x(1:2); dq  = x(3:4);
    qm  = x(5:6); dqm = x(7:8);
    theta_est = x(9:10); % Estimated masses [m1_est; m2_est]
    
    % Workspace target clamp
    max_reach = 1.99;
    dist = sqrt(mouse_x^2 + mouse_y^2);
    if dist > max_reach
        mouse_x = mouse_x * (max_reach / dist); mouse_y = mouse_y * (max_reach / dist);
    end
    
    % Inverse Kinematics for target
    r = robot_ik(mouse_x, mouse_y); 
    
    % Reference Model (Ideal trajectory generation)
    Wn = 5; zeta = 1;
    ddqm = Wn^2 * (r - qm) - 2 * zeta * Wn * dqm;
    
    % Core Tracking PD Errors
    e = q - qm; de = dq - dqm;
    
    % --- Self-Tuning Control Law Computation ---
    % Using the currently ESTIMATED masses to compute feedback cancellations
    m1_e = max(0.1, theta_est(1)); % Prevent negative mass estimations
    m2_e = max(0.1, theta_est(2));
    
    l1 = 1; l2 = 1; lc1 = l1/2; lc2 = l2/2; I1 = 0.1; I2 = 0.1; g = 9.81;
    a = I1 + I2 + m1_e*lc1^2 + m2_e*(l1^2 + lc2^2);
    b = m2_e*l1*lc2;
    d = I2 + m2_e*lc2^2;
    
    M_est = [a + 2*b*cos(q(2)), d + b*cos(q(2)); d + b*cos(q(2)), d];
    C_est = [-b*sin(q(2))*dq(2), -b*sin(q(2))*(dq(1) + dq(2)); b*sin(q(2))*dq(1), 0];
    G_est = [(m1_e*lc1 + m2_e*l1)*g*cos(q(1)) + m2_e*lc2*g*cos(q(1) + q(2)); m2_e*lc2*g*cos(q(1) + q(2))];
    
    % Outer loop aux input (PD tracking block)
    Kp_nominal = diag([40, 60]); Kd_nominal = diag([12, 18]);
    v = ddqm - Kp_nominal*e - Kd_nominal*de;
    
    % Compute Model-Based Self-Tuning Torque
    tau = M_est * v + C_est * dq + G_est;
    max_torque = 80;
    tau = max(-max_torque, min(max_torque, tau));
    
    % Run True Robot Dynamics Plant
    dq_actual = two_link_dynamics(t, [q; dq], tau);
    ddq = dq_actual(3:4);
    
    % --- Update Online Parameter Estimator ---
    Gamma_est = diag([2.5, 5.0]); % Parameter learning speed
    dtheta = parameter_estimator(q, dq, ddq, tau, theta_est, Gamma_est);
    
    dx = [dq; ddq; dqm; ddqm; dtheta];
end