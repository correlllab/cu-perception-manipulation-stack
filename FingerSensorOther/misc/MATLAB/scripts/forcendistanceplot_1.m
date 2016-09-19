%use this to plot color, current and thickness
%figure 5a, 5b and 6a

figure(1);
hold on
Dx = [6 5 4 3 2 1 0 -1 -2 -3 -4 -5];
load('8:1_6mm.mat');
j = {'white','gray','black','red','yellow'};
color = {'w','g','k','r','y'};
cv = 1; %{40,80,120,160,200} change row(current value)
    
Dy = data_white(cv,1:12);
plot(Dx,Dy,'g')
hold on
Dy = data_gray(cv,:);
plot(Dx,Dy,'color', [0.5 0.5 0.5])
Dy = data_black(cv,:);
plot(Dx,Dy,'k')
Dy = data_red(cv,1:12);
plot(Dx,Dy,'r')
hold on
Dy = data_yellow(cv,:);
plot(Dx,Dy,'y')


legend('White','Gray', 'Black', 'Red', 'Yellow');
 set(xlabel('-Force [N] / Distance [cm] '),'FontSize',12)
 set(ylabel('Sensor reading'),'FontSize',12)
 set(title('PDMS'),'FontSize',15)
 axis([-5 6 0 65535]);
 %figure('Color',[0.8 0.8 0.8]);
 plot([0 0],[0 65535],'r--')
 %print('force_distance','-dpng')
