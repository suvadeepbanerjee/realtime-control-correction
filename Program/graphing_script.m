figure(1)
subplot(121)
load MARS_10.mat
plot(t(1:100:end-1),5*error(1:100:end),'rv-','linewidth',2)
hold on
load MARS_50.mat
plot(t(1:100:end-1),10*error(1:100:end),'ko-','linewidth',2)
load nominal_MARS.mat
plot(t(1:end-1),error,'b','linewidth',2);
xlabel('Time (seconds)', 'fontsize',30);
ylabel('Error Signal', 'fontsize',30);
legend('10% Fault','50% Fault','Fault-free');
axis([0 20 -1 5])
title('Actuator Failure - MARS','fontsize',30);
subplot(122)
load ART2_10.mat
plot(t(1:100:end-1),10*error(1:100:end),'rv-','linewidth',2)
hold on
load ART2_50.mat
plot(t(1:100:end-1),20*error(1:100:end),'ko-','linewidth',2)
load nominal_ART.mat
plot(t(1:end-1),error,'b','linewidth',2);
xlabel('Time (seconds)', 'fontsize',30);
ylabel('Error Signal', 'fontsize',30);
legend('10% Fault','50% Fault','Fault-free');
title('Actuator Failure - ART2','fontsize',30);

figure(2)
subplot(121)
load MARS_50s.mat
plot(t(1:100:end-1),100*error(1:100:end),'rv-','linewidth',2)
hold on
load MARS_100.mat
plot(t(1:100:end-1),1*error(1:100:end),'ko-','linewidth',2)
load nominal_MARS.mat
plot(t(1:end-1),error,'b','linewidth',2);
xlabel('Time (seconds)', 'fontsize',30);
ylabel('Error Signal', 'fontsize',30);
legend('10% Fault','50% Fault','Fault-free','Location','NorthWest');
axis([0 20 -1.5 5])
title('Sensor Failure - MARS','fontsize',30);
subplot(122)
load ART2_50s.mat
plot(t(1:100:end-1),10*error(1:100:end),'rv-','linewidth',2)
hold on
load ART2_100.mat
plot(t(1:100:end-1),1*error(1:100:end),'ko-','linewidth',2)
load nominal_ART.mat
plot(t(1:end-1),error,'b','linewidth',2);
xlabel('Time (seconds)', 'fontsize',30);
ylabel('Error Signal', 'fontsize',30);
legend('10% Fault','50% Fault','Fault-free');
axis([0 20 -1.5 1.2])
title('Sensor Failure - ART2','fontsize',30);
