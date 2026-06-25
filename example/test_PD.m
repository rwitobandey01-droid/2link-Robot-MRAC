clear; clc;

%Intial state [q1, q2, dq1, dq2]

q0 = [0;0;0;0];

%desired joint angles 
q_des = [pi/4; pi/6];

%PD gains
Kp = diag([50 50]);
Kd = diag([10 10]);

%simulate time
tspan = [0 10];

%run the ODE 
[t, q] = ode45(@(t,q) pd_wrapper(t,q,q_des,Kp,Kd), tspan, q0);


%plot 
figure;
plot(t, q(:,1), 'r', t, q(:,2), 'b');
hold on;
yline(q_des(1), 'r--', 'Target q1');
yline(q_des(2), 'b--', 'Target q2');
legend('q1', 'q2', 'Target q1', 'Target q2');
title('Joint Angles under PD Control');
grid on;
