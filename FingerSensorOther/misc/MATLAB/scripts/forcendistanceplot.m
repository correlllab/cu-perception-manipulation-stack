figure(6);
hold on
% % Forces and distances at 70mA (Vytaflex)
% Dvytax = [0 1 2 3 4 5 6 7];
% Fvytax = [0 5 10 20 50 100 200 500];
% Dvytay = mean(load('distance_vytaflex_20mA.txt'));
% Dvytay = Dvytay(1,1:8); %removing the values for infinty 
% Fvytay = mean(load('force_vytaflex_20mA.txt'));
% plot([-flip(Fvytax)/1000*9.81 Dvytax],[flip(Fvytay) Dvytay],'r')

% %Forces and distances at 80mA (Ecoflex)
%Decox = [0 1 2 3 4 5 6 7];
%Fecox = [0 5 10 20 50 100 200 500];
%Decoy = mean(load('distance_ecoflex_120mA.txt'));
%Decoy = Decoy(1,1:8);
%Fecoy = mean(load('force_ecoflex_120mA.txt'));
%plot([-flip(Fecox)/1000*9.81 Decox],[flip(Fecoy) Decoy])


% %Forces and distances at 40mA (PDMS)
 Dpdmsx = [1 2 3 4 5];
 Fpdmsx = [0 1 2 3 4 5];
 load('exp#4.mat');
 Dpdmsy = distance121(2,[2,4,6,7,8]);
 Fpdmsy = pressure121(2,[1,6,7,8]);
 plot([-flip(Fpdmsx) Dpdmsx],[flip(Fpdmsy) Dpdmsy])

%legend('Vytaflex@40mA','Ecoflex @20mA', 'PDMS@120mA');
set(xlabel('-Force [N] / Distance [cm] '),'FontSize',14)
set(ylabel('Sensor reading'),'FontSize',14)
set(title('PDMS'),'FontSize',14)
axis([-5 8 0 65535]);
plot([0 0],[0 65535],'r--')
print('force_distance','-dpng')
