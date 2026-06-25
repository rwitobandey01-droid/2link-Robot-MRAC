function dtheta = parameter_estimator(q, dq, ddq, tau, theta, Gamma_est);

%Extract states::
q2 = q(2);
dq1 = dq(1); dq2 = dq(2);

%known kinemetic constants:
l1 = 1; l2 = 1;
lc1 = l1/2 ; lc2 = l2/2;
g = 9.81;

% In adaptive control, a regression function translates complex, unknown 
% system dynamics into a structured format where unknown physical parameters 
% (like mass or friction) are linearly separated from measurable signals (like states or velocities

Y11 =lc1^2*ddq(1) + l1*g*cos(q(1)); 
% Y_{11}$ (Multiplier for $m_1$): Tracks how much the mass of Link 1
% affects the trq at Joint 1 through its acceleration and gravity.
Y12 = (l1^2 + lc2^2 + 2*l1*lc2*cos(q2))*ddq(1) + (lc2^2 + l1*lc2*cos(q2))*ddq(2) ...
          - l1*lc2*sin(q2)*dq2*dq1 - l1*lc2*sin(q2)*(dq1+dq2)*dq2 + l1*g*cos(q(1)) + lc2*g*cos(q(1)+q2);
% Y_{12}$ (Multiplier for $m_2$): A complex term capturing how the mass of Link 2
% exerts trq on Joint 1 via centrifugal forces, Coriolis forces (dot{q}_1,dot{q}_2),and gravity.
Y21 = 0; %This is 0. Why? Because moving Link 2 has absolutely no physical
         % effect on the first link's mass behind it.
Y22 = (lc2^2 + l1*lc2*cos(q2))*ddq(1) + lc2^2*ddq(2) + l1*lc2*sin(q2)*dq1^2 + lc2*g*cos(q(1)+q2);
%Y22 : Tracks how link 2's won mass effects its own joint trq.


Y = [Y11, Y12 ; Y21, Y22];

%prediction error :: e_param = act trq - predicted trq

tau_pred = Y*theta;
e_param = tau - tau_pred;
e_param = e_param(:); % FORCE e_param to be a 2x1 column vector to avoid dimension mixing
% I don't care how this array is currently oriented, 
% flatten it out and force it into a strict column vector

%Gradient descnet param update law::
dtheta = Gamma_est*Y' * e_param;
% Force dtheta to be a column vectors for ode45 compatibility 
dtheta = dtheta(:);

end