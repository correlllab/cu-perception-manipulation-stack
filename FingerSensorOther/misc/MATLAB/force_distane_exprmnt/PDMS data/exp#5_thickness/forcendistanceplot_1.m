%use this to plot color, current and thickness

figure(1);
hold on
Dx = [6 5 4 3 2 1 0 -1 -2 -3 -4 -5];
load('8:1_6mm.mat');
cv = 1; %{40,80,120,160,200} change row(current value)
Dy = data_gray(cv,1:12);
plot(Dx,Dy)
hold on
cv = 2; %{40,80,120,160,200} change row(current value)
Dy = data_gray(cv,1:12);
plot(Dx,Dy)

legend('6mm','12mm');
 set(xlabel('-Force [N] / Distance [cm] '),'FontSize',12)
 set(ylabel('Sensor reading'),'FontSize',12)
 set(title('PDMS'),'FontSize',15)
 axis([-5 6 0 65535]);
 %figure('Color',[0.8 0.8 0.8]);
 plot([0 0],[0 65535],'r--')
 %print('force_distance','-dpng')
