figure(1);
hold on
Dx = [6 5 4 3 2 1 0 -1 -2 -3 -4 -5];
load('8:1_6mm.mat');
j = {'white','gray','black','red','yellow'};
color = {'w','g','k','r','y'};
cv = 1; %{40,80,120,160,200} change row(current value)
    
for cv=1:1:5 
 Dy = data_red(cv,1:12);
 plot(Dx,Dy)
 hold on
end

legend('40mA','80mA', '120mA', '160mA', '200mA');
%title('PDMS|8:1,6mm')
set(xlabel('-Force [N] / Distance [cm] '),'FontSize',12)
set(ylabel('16-bit ADC value'),'FontSize',12)
set(title('PDMS|8:1, 6mm'),'FontSize',14)
axis([-5 6 0 65535]);
%figure('Color',[0.8 0.8 0.8]);
plot([0 0],[0 65535],'r--')
%print('force_distance','-dpng')


legend('40mA','80mA', '120mA', '160mA', '200mA');
%title('PDMS|8:1,6mm')
set(xlabel('-Force [N] / Distance [cm] '),'FontSize',12)
set(ylabel('16-bit ADC value'),'FontSize',12)
set(title('PDMS|8:1, 6mm'),'FontSize',14)
axis([-5 6 0 65535]);
%figure('Color',[0.8 0.8 0.8]);
plot([0 0],[0 65535],'r--')
%print('force_distance','-dpng')
