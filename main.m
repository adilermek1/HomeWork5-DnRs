clc
clear all
close all

fprintf('Forward Kinematics')
L =0.2;


% initial angles 
q = 30*pi/180;
s = 1 ;
% Forward kinematics
T = FK_hw5(q,s,L)
% robot figure 
draw_robot(q,s,L);

fprintf('Findind the Jacobians')
syms q1 q2 dq1 dq2 ddq1 ddq2 l1 l2 d1 d2 m1 m2 I1 I2 g
T_analitycal = FK_hw5(q,s,L)

T1 = T1_RRR(q1);
T2 = T2_RRR(q1,q2);
T3 = T3_RRR(q1,q2,L);

z0 = [T1(1,3);T1(2,3);T1(3,3)];
z1 = [T2(1,3);T2(2,3);T2(3,3)];

Oc = [T_analitycal(1,4);T_analitycal(2,4);T_analitycal(3,4)];
Oc1 = [0;0;0];
Oc2 = [(q2+L)*cos(q1);(q2+L)*sin(q1);0];
O0 = [T1(1,4);T1(2,4);T1(3,4)];
O1 = [T2(1,4);T2(2,4);T2(3,4)];
Zer = [0;0;0];

Jv1 = [cross(z0,Oc1-O0) Zer]
Jv2 = [cross(z0,Oc2-O0) cross(z1,Oc2-O1)]

Jw1 = [z0 Zer]
Jw2 = [z0 z1]

R1 = T1(1:3,1:3);
R2 = T2(1:3,1:3);

q = [q1;q2];

D1 = m1*((Jv1).')*(Jv1)+(Jw1.')*R1*I1*(R1.')*Jw1;
D2 = m2*(Jv2.')*(Jv2)+(Jw2')*R2*I2*(R2')*Jw2;

D = D1+D2

P1 = m1*g*0;
P2 = m2*g*(q2+L)*sin(q1);

P = P1+P2;

G1 = diff(P,q1);
G2 = diff(P,q2);
G = G1+G2

dq = [dq1;dq2];
ddq = [ddq1;ddq2];

C = Coriolis(D,q,dq,2)

tor = D*ddq+C*dq+G
D(q1,q2) = subs(D,{m1,m2,I1,I2,l1,l2,d1,d2},{2,2,1,2,0,1,0,1.2});
C(q1,q2,dq1,dq2) = subs(C*dq,{m1,m2,I1,I2,l1,l2,d1,d2},{2,2,1,2,0,1,0,1.2});
G(q1,q2) = subs(G,{m1,m2,I1,I2,l1,l2,d1,d2,g},{2,2,1,2,0,1,0,1.2,9.81});

D(0,0);
C(0,0,0,0);
G(0,0);

q1_0 = pi/6;
q2_0 = 0;
dq1_0 = 0;
dq2_0 = 0;
dt = 0.01;

U = [1;1];
n=300;

for i=1:n
    q1p(i) = q1_0;
    q2p(i) = q2_0;
    dq1p(i) = dq1_0;
    dq2p(i) = dq2_0;
    ddq = inv(D(q1_0,q2_0))*(U-C(q1_0,q2_0,dq1_0,dq2_0)-G(q1_0,q2_0));
    
    
    dq1_0 = dq1p(i)+double(ddq(1)*dt);
    dq2_0 = dq2p(i)+double(ddq(2)*dt);
    
    q1_0 = q1p(i) + dq1_0*dt;
    q2_0 = q2p(i) + dq2_0*dt;
    
end
t = 0:0.1:(0.1*(n-1));
figure(2)
plot(t,q1p)
title('q1 vs time,with torque')
xlabel('time')
ylabel('q1') 
grid on

figure(3) 
plot(t,q2p)
title('s vs time,with torque')
xlabel('time')
ylabel('s')
grid on 

figure(4)
plot(t,dq1p)
title('dq1 vs time,with torque')
xlabel('time')
ylabel('dq1')
grid on 

figure(5)
plot(t,dq2p)
title('ds vs time,with torque')
xlabel('ds')
ylabel('time')
grid on 

