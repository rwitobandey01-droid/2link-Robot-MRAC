function dq = two_link_dynamics(t, q, tau)
% q = [q1; q2; dq1; dq2]
q1  = q(1); q2  = q(2);
dq1 = q(3); dq2 = q(4);

% --- Robot parameters ---
m1 = 1; m2 = 1;

% SIMULATE PAYLOAD: After 7 seconds, the robot picks up a heavy 1.5kg object!
if t > 7
    m_load = 1.5; 
else
    m_load = 0;
end
m2_eff = m2 + m_load; % Effective mass of link 2

l1 = 1; l2 = 1;
lc1 = l1/2; lc2 = l2/2;
I1 = 0.1; I2 = 0.1;
g  = 9.81;

% --- Precompute constants using effective mass ---
a = I1 + I2 + m1*lc1^2 + m2_eff*(l1^2 + lc2^2);
b = m2_eff*l1*lc2;
d = I2 + m2_eff*lc2^2;

M = [a + 2*b*cos(q2), d + b*cos(q2); d + b*cos(q2), d];
C = [-b*sin(q2)*dq2, -b*sin(q2)*(dq1 + dq2); b*sin(q2)*dq1, 0];
G = [(m1*lc1 + m2_eff*l1)*g*cos(q1) + m2_eff*lc2*g*cos(q1 + q2); m2_eff*lc2*g*cos(q1 + q2)];

ddq = M \ (tau - C*[dq1; dq2] - G);
dq = [dq1; dq2; ddq];
end