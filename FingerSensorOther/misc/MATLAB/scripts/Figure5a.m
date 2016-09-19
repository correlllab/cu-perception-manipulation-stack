h=figure(1);
hold on
Dx = [6 5 4 3 2 1 0 1 2 3 4 5];
load('8:1_6mm.mat');
j = {'white','gray','black','red','yellow'};
color = {'w','g','k','r','y'};
cv = 1; %{40,80,120,160,200} change row(current value)

subplot(1,2,1)
for cv=1:1:5 
 Dy = data_red(cv,7:12);
 plot(Dx(7:12),flip(Dy))
 hold on
end

set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
set(xlabel('Force [N]'),'FontSize',18)
set(ylabel('16-bit ADC value'),'FontSize',18)
set(gca,'XTickLabel',num2str(flip(Dx(7:12))','%d'))
axis([0 5 0 65535]);
l=legend('40mA','80mA', '120mA', '160mA', '200mA');
delete(l);

subplot(1,2,2)
for cv=1:1:5 
 Dy = data_red(cv,1:7);
 plot(Dx(1:7),Dy)
 hold on
end
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12)
set(xlabel('Distance [cm]'),'FontSize',18)
%set(ylabel('16-bit ADC value'),'FontSize',18)
axis([0 6 0 65535]);

set(gca,'pos',get(gca,'pos') + [-0.05 0 0 0])

set(legend('40mA','80mA', '120mA', '160mA', '200mA'),'fontsize',16);
%title('PDMS|8:1,6mm')
%axis([-5 6 0 65535]);
%plot([0 0],[0 65535],'r--')



saveas(h,'Figure5a','png')

