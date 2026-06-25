function dx = mrac_wrapper(t, x, mouse_x, mouse_y)
    q   = x(1:2); dq  = x(3:4);
    qm  = x(5:6); dqm = x(7:8);
    Kp = reshape(x(9:12), [2, 2]);
    Kd = reshape(x(13:16), [2, 2]);
    
    % --- WORKSPACE SAFETY CLAMP ---
    % Prevent the mouse target from exceeding the 2m arm reach!
    max_reach = 1.99; % slightly less than l1 + l2 to prevent singularity
    dist = sqrt(mouse_x^2 + mouse_y^2);
    if dist > max_reach
        mouse_x = mouse_x * (max_reach / dist);
        mouse_y = mouse_y * (max_reach / dist);
    end
    
    % Inverse Kinematics
    r = robot_ik(mouse_x, mouse_y); 
    
    % Reference Model 
    Wn = 5; zeta = 1; % Speed up the model slightly for responsive mouse tracking
    ddqm = Wn^2 * (r - qm) - 2 * zeta * Wn * dqm;
    
    % Errors and Adaptation
    e = q - qm; de = dq - dqm;
    P_pos = diag([20, 40]); P_vel = diag([4, 8]);   
    filtered_error = P_vel * de + P_pos * e; 
    
    Gamma_p = diag([150, 500]); Gamma_d = diag([30, 200]);   
    dKp = Gamma_p * (filtered_error * e');
    dKd = Gamma_d * (filtered_error * de');
    
    % Control & Saturation
    tau = - Kp * e - Kd * de; 
    max_torque = 80; % Increased torque to handle fast mouse swipes
    tau = max(-max_torque, min(max_torque, tau)); 
    
    % True Dynamics
    dq_actual = two_link_dynamics(t, [q; dq], tau);
    ddq = dq_actual(3:4);
    
    dx = [dq; ddq; dqm; ddqm; dKp(:); dKd(:)];
end