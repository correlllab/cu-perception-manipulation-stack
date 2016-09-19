h=figure;
hold on

% %Forces and distances at 40mA (PDMS)
 Dpdmsx = [1 2 3 4 5];
 Fpdmsx = [0 1 2 3 4 5];
 load('exp#4.mat');
 Dpdmsy = distance81(2,[2,4,6,7,8]);
 Fpdmsy = pressure81(2,[1,6,7,8]);
 %plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy]);
 subplot(1,2,1)
 plot([(Fpdmsx)],[flip(Fpdmsy)])
 hold on;

 subplot(1,2,2)
 plot([Fpdmsx(1) Dpdmsx],[Fpdmsy(1) (Dpdmsy)])
 hold on
 
 Dpdmsy = distance101(2,[2,4,6,7,8]);
 Fpdmsy = pressure101(2,[1,6,7,8]);
 %plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy])
 subplot(1,2,1)
 plot([(Fpdmsx)],[flip(Fpdmsy)])
 subplot(1,2,2)
 plot([Fpdmsx(1) Dpdmsx],[Fpdmsy(1) (Dpdmsy)])
 
 
 hold on 
 
 Dpdmsy = distance121(2,[2,4,6,7,8]);
 Fpdmsy = pressure121(2,[1,6,7,8]);
 %plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy])
 subplot(1,2,1)
 plot([(Fpdmsx)],[flip(Fpdmsy)])
 subplot(1,2,2)
 plot([Fpdmsx(1) Dpdmsx],[Fpdmsy(1) (Dpdmsy)])

 subplot(1,2,1); 
 axis([0 5 0 65535]); 
 set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
set(xlabel('Force [N]'),'FontSize',18)
set(ylabel('16-bit ADC value'),'FontSize',18)
%set(title('PDMS|40mA, 6mm'),'FontSize',14)
subplot(1,2,2); 
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12);
axis([0 6 0 65535]);
set(legend('8:1','10:1', '12:1'),'FontSize',16);
set(xlabel('Distance [cm] '),'FontSize',18)
set(gca,'pos',get(gca,'pos') + [-0.05 0 0 0])
saveas(h,'Figure5c','png')
