%*********************************************************************
% figures.m
% A file to plot euler angles, body rates, regulated torque,
% reaction wheel torque, and reaction wheel speed. Addidtionaly, the
% overall power consumption for the specified maneuver is printed
%*********************************************************************

hfig1 = figure(1);
plot(euler.Time(:,1),euler.Data(:,3),'k-','LineWidth',1)
hold on 
plot(euler.Time(:,1),euler.Data(:,2),'r-','LineWidth',1)
plot(euler.Time(:,1),euler.Data(:,1),'b-','LineWidth',1) 
plot(trajectory.Time(:,1),trajectory.Data(:,1),'g:','LineWidth',1.5)
xl = xlabel('time (sec)');
yl = ylabel('Euler angles (deg)');
l1 = legend('$\phi$','$\theta$','$\psi$','Location','eastoutside');
set([xl yl l1],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig1, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

hfig2 = figure(2);
plot(wb.Time(:,1),wb.Data(:,1),'k-','LineWidth',1) 
hold on
plot(wb.Time(:,1),wb.Data(:,2),'r-','LineWidth',1)
plot(wb.Time(:,1),wb.Data(:,3),'b-','LineWidth',1)
x3 = xlabel('time (sec)');
y3 = ylabel('body rates (rad/sec)');
l3 = legend('$\omega_{x}$','$\omega_{y}$','$\omega_{z}$','Location','eastoutside');
ax = gca;
ax.YRuler.Exponent = 0;
set([x3 y3 l3],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig2, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

hfig3 = figure(3);
plot(tau_regulator.Time(:,1),tau_regulator.Data(:,1)*1000,'k-','LineWidth',1) 
hold on
plot(tau_regulator.Time(:,1),tau_regulator.Data(:,2)*1000,'r-','LineWidth',1)
plot(tau_regulator.Time(:,1),tau_regulator.Data(:,3)*1000,'b-','LineWidth',1)
x3 = xlabel('time (sec)');
y3 = ylabel('regulated torque (mN$\cdot$m)');
l3 = legend('$T_{c1}$','$T_{c2}$','$T_{c3}$','Location','eastoutside');
set([x3 y3 l3],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig3, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

hfig4 = figure(4);
plot(tau_rxnwheel.Time(:,1),tau_rxnwheel.Data(:,1)*1000,'k-','LineWidth',1) 
hold on
plot(tau_rxnwheel.Time(:,1),tau_rxnwheel.Data(:,2)*1000,'r-','LineWidth',1)
plot(tau_rxnwheel.Time(:,1),tau_rxnwheel.Data(:,3)*1000,'b-','LineWidth',1)
plot(tau_rxnwheel.Time(:,1),tau_rxnwheel.Data(:,4)*1000,'g-','LineWidth',1)
x4 = xlabel('time (sec)');
y4 = ylabel('wheel torque (mN$\cdot$m)');
l4 = legend('$\tau_{1}$','$\tau_{2}$','$\tau_{3}$','$\tau_{4}$','Location','eastoutside');
set([x4 y4 l4],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig4, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

hfig5 = figure(5);
plot(rxnwheel_speed.Time(:,1),rxnwheel_speed.Data(:,1)*(30/pi),'k-','LineWidth',1) 
hold on
plot(rxnwheel_speed.Time(:,1),rxnwheel_speed.Data(:,2)*(30/pi),'r-','LineWidth',1)
plot(rxnwheel_speed.Time(:,1),rxnwheel_speed.Data(:,3)*(30/pi),'b-','LineWidth',1)
plot(rxnwheel_speed.Time(:,1),rxnwheel_speed.Data(:,4)*(30/pi),'g-','LineWidth',1)
x5 = xlabel('time (sec)');
y5 = ylabel('wheel speed (rpm)');
l5 = legend('$\omega_{1}$','$\omega_{2}$','$\omega_{3}$','$\omega_{4}$','Location','eastoutside');
set([x5 y5 l5],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig5, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

hfig6 = figure(6);
plot(current.Time(:,1),abs(current.Data(:,1))*1000,'k-','LineWidth',1) 
hold on
plot(current.Time(:,1),abs(current.Data(:,2))*1000,'r-','LineWidth',1)
plot(current.Time(:,1),abs(current.Data(:,3))*1000,'b-','LineWidth',1)
plot(current.Time(:,1),abs(current.Data(:,4))*1000,'g-','LineWidth',1)
x6 = xlabel('time (sec)');
y6 = ylabel('current (mA)');
l6 = legend('wheel 1','wheel 2','wheel 3','wheel 4','Location','eastoutside');
set([x6 y6 l6],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig6, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

hfig7 = figure(7);
plot(voltage.Time(:,1),abs(voltage.Data(:,1)),'k-','LineWidth',1) 
hold on
plot(voltage.Time(:,1),abs(voltage.Data(:,2)),'r-','LineWidth',1)
plot(voltage.Time(:,1),abs(voltage.Data(:,3)),'b-','LineWidth',1)
plot(voltage.Time(:,1),abs(voltage.Data(:,4)),'g-','LineWidth',1)
x7 = xlabel('time (sec)');
y7 = ylabel('voltage (V)');
l7 = legend('wheel 1','wheel 2','wheel 3','wheel 4','Location','eastoutside');
set([x7 y7 l7],'Interpreter','Latex','FontSize',16)
set(gca,'TickLabelInterpreter','Latex','FontSize',14)
set(hfig7, 'Position', [250 75 1100 400])
legend('boxoff')
axis tight
grid on
hold off

%*********************************************************************
% Total Power Consumption
%*********************************************************************
P_w1  = sum(abs(power.Data(:,1)));
P_w2  = sum(abs(power.Data(:,2)));
P_w3  = sum(abs(power.Data(:,3)));
P_w4  = sum(abs(power.Data(:,4)));
P_tot = P_w1 + P_w2 + P_w3 + P_w4;

print = 'Power consumption for specified manuever is %4.2f Watts \n';
fprintf(print,P_tot)

%*********************************************************************
% Run Animation Script
%*********************************************************************
% animate.m