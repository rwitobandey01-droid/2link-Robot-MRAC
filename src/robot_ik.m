%robot inverse kinametics

function q = robot_ik(x, y)
    % Link lengths (must match your dynamics)
    l1 = 1; l2 = 1;
    
    % Cosine theorem for q2
    D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
    % Bound D to avoid complex numbers due to rounding
    D = max(-1, min(1, D)); 
    
    q2 = acos(D); % Elbow up configuration
    q1 = atan2(y, x) - atan2(l2 * sin(q2), l1 + l2 * cos(q2));
    
    q = [q1; q2];
end