%load 8_1_6mm
load('8:1_6mm.mat');
h=figure();
Y=data_white(1,7:12);
X=0:5;
[f,gof]=fit(Y',X','power1','Robust','Bisquare')
p=plot(f,Y,X); hold on;
p(1).Color='Yellow';
p(2).Color='Yellow';


Y=data_red(1,7:12);
X=0:5;
[f,gof]=fit(Y',X','power1','Robust','Bisquare')
p=plot(f,Y,X); hold on;
p(1).Color='Red';
p(2).Color='Red';

Y=data_black(1,7:12);
X=0:5;
[f,gof]=fit(Y',X','power1','Robust','Bisquare')
p=plot(f,Y,X); hold on;
p(2).Color='Black';
p(1).Color='Black';

set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',12)
set(legend('White data','White fit','Red data','Red fit','Black data','Black fit'),'FontSize',18);
%set(title('Calibration of force PDMS|8:1, 6mm'),'FontSize',14);
set(xlabel('Raw sensor data (x10^4)'),'FontSize',20);
set(ylabel('Force [N]'),'FontSize',20);

axis([0 55550 0 6])

saveas(h','Figure6d','png')

