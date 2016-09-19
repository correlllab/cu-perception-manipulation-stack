%use this to plot color, current and thickness
%figure 5a, 5b and 6a

h=figure;
hold on
Dx = [6 5 4 3 2 1 0 1 2 3 4 5];
load('8:1_6mm.mat');
j = {'white','gray','black','red','yellow'};
color = {'w','g','k','r','y'};
cv = 1; %{40,80,120,160,200} change row(current value)
    
subplot(1,2,1)
Dy = data_white(cv,7:12);
plot(Dx(7:12),flip(Dy),'g')
hold on
Dy = data_gray(cv,7:12);
plot(Dx(7:12),flip(Dy),'color', [0.5 0.5 0.5])
Dy = data_black(cv,7:12);
plot(Dx(7:12),flip(Dy),'k')
Dy = data_red(cv,7:12);
plot(Dx(7:12),flip(Dy),'r')
Dy = data_yellow(cv,7:12);
plot(Dx(7:12),flip(Dy),'y')

subplot(1,2,2)
Dy = data_white(cv,1:7);
plot(Dx(1:7),Dy,'g')
hold on
Dy = data_gray(cv,1:7);
plot(Dx(1:7),Dy,'color', [0.5 0.5 0.5])
Dy = data_black(cv,1:7);
plot(Dx(1:7),Dy,'k')
Dy = data_red(cv,1:7);
plot(Dx(1:7),Dy,'r')
Dy = data_yellow(cv,1:7);
plot(Dx(1:7),Dy,'y')


subplot(1,2,1)
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
set(xlabel('Force [N]'),'FontSize',20)
set(ylabel('16-bit ADC value'),'FontSize',20)
axis([0 5 0 65535]);

subplot(1,2,2)
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
set(legend('White','Gray', 'Black', 'Red', 'Yellow'),'FontSize',18);
set(xlabel('Distance [cm] '),'FontSize',20)
%set(title('PDMS|8:1, 40mA'),'FontSize',14)
axis([0 6 0 65535]);

set(gca,'pos',get(gca,'pos') + [-0.05 0 0 0])
saveas(h,'Figure6a','png')