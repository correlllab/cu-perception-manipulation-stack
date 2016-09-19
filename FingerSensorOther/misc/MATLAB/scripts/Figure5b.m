h=figure(1);
hold on
Dx = [6 5 4 3 2 1 0 1 2 3 4 5];


subplot(1,2,1);
load('8:1_6mm.mat');
cv = 2; %{40,80,120,160,200} change row(current value)
Dy = data_gray(cv,7:12);
plot(Dx(7:12),flip(Dy))
hold on
load('8:1_12mm.mat');
cv = 2; %{40,80,120,160,200} change row(current value)
Dy = data_gray(cv,7:12);
plot(Dx(7:12),flip(Dy))
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
set(xlabel('Force [N]'),'FontSize',18)
set(ylabel('16-bit ADC Value'),'FontSize',18)
set(gca,'XTickLabel',num2str(flip(Dx(7:12))','%d'))

axis([0 5 0 65535]);


subplot(1,2,2);
load('8:1_6mm.mat');
cv = 2; %{40,80,120,160,200} change row(current value)
Dy = data_gray(cv,1:7);
plot(Dx(1:7),Dy)
hold on
load('8:1_12mm.mat');
cv = 2; %{40,80,120,160,200} change row(current value)
Dy = data_gray(cv,1:7);
plot(Dx(1:7),Dy)
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
set(legend('6mm','12mm'),'FontSize',16);
set(xlabel('Distance [cm] '),'FontSize',18)
%set(ylabel('16-bit ADC Value'),'FontSize',18)
%set(title('PDMS|80mA, 8:1'),'FontSize',14)
set(gca,'pos',get(gca,'pos') + [-0.05 0 0 0])

axis([0 6 0 65535]);

saveas(h,'Figure5b','png')
