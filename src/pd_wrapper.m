%PD wrapper

function dq = pd_wrapper(t, q, q_des, Kp, Kd)

q1 = q(1:2);
dq1 = q(3:4);

%PD torque
tau = Kp*(q_des -q1) - Kd*dq1;

%call the dynamics

dq = two_link_dynamics(t, q, tau);

end
