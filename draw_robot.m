function draw_robot(q,s,L)

figure;
hold on
view(200,25)
grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title('Robot')
T1= Rz(q(1));
plot3(0,0,0,'ro','MarkerSize',10,'LineWidth', 10);
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-b','LineWidth', 5);
plot3(T1(1,4),T1(2,4),T1(3,4),'ro','MarkerSize',10,'LineWidth', 10);
T2=Rz(q(1))*Tx(s);
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-b','LineWidth', 5);
plot3(T2(1,4),T2(2,4),T2(3,4),'ro','MarkerSize',10,'LineWidth', 10);
T3=Rz(q(1))*Tx(s)*Tx(2*L);
plot3([T2(1,4) T3(1,4)],[T2(2,4) T3(2,4)],[T2(3,4) T3(3,4)],'-b','LineWidth', 5);
plot3(T3(1,4),T3(2,4),T3(3,4),'ro','MarkerSize',10,'LineWidth', 10);
 