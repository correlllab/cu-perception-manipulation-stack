h=figure(1);
data = load('../../objectscan_data/contactpoint_pan.txt');

subplot(1,2,1)
Z = [1:7];
Z = repmat(Z,size(data,1),1);
time = [1:1:size(data,1)];
time = repmat(time',1,7);
mesh(Z,time,data(:,1:7));
%set(title('Left finger'),'FontSize',14);
hold on
pointA = [1,49,6707];
pointB = [7,49,6946];
pointC = [5,49,18860];
points=[pointA' pointB' pointC'];
%fill3(points(1,:),points(2,:),points(3,:),'r')
%alpha(0.3)
set(xlabel('Sensor (left)'),'FontSize',18,'Rotation',40);
set(ylabel('Time [0.1s]'),'FontSize',18,'Rotation',-50);
set(zlabel('Raw value'),'FontSize',18);
set(gca,'XTick',1:7)
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',16)

subplot(1,2,2)
mesh(Z,time,data(:,9:15));
hold on 
pointA = [1,49,6707];%you might wanna change this values
pointB = [7,49,6946];%you might wanna change this values
pointC = [5,49,18860];%you might wanna change this values
points=[pointA' pointB' pointC'];
%fill3(points(1,:),points(2,:),points(3,:),'r')
%alpha(0.3)
%set(title('Right finger'),'FontSize',14);
set(xlabel('Sensor (right)'),'FontSize',18,'Rotation',40);
%set(ylabel('Time [0.1s]'),'FontSize',18);
set(gca,'XTick',1:7)
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',16)
saveas(h,'Figure9a','png')

h=figure(2);
X=time(:,1);
Y=data(:,5);
f=fit(X,Y,'smoothingspline','SmoothingParam',0.5);
fdev=fnder(f.p,1);
y_prime=ppval(fdev,X);
plot(f,X,Y); hold on;
plot(X,y_prime);
set(legend('Raw data','Spline interpolation','First derivative'),'Location','NorthWest','FontSize',18);
set(xlabel('Time [0.1s]'),'FontSize',18);
set(ylabel('Raw value'),'FontSize',18);

ext=X(y_prime==max(y_prime));
plot([ext ext],[0 Y(ext)],'r--')
plot([0 ext],[Y(ext) Y(ext)],'r--')
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',18)
saveas(h,'Figure9b','png')


h=figure(3)
load 8_1_6mm.mat
%plot(data_black(3,:)'), hold on
%set(gca,'XTick',1:12)
%set(gca,'XTickLabel',{'6cm','5cm','4cm','3cm','2cm','1cm','0cm','1N ','2N ','3N ','4N ','5N '})
%plot([7 7],[0 data_black(3,7)],'r--')
%plot([0 7],[data_black(3,7) data_black(3,7)],'r--');
%set(title('Black Paper @120mA, 6mm PDMS|8:1'),'FontSize',14);

subplot(1,2,1);
plot(data_black(3,1:7)'), hold on
set(gca,'XTick',1:7)
set(gca,'XTickLabel',{'6cm','5cm','4cm','3cm','2cm','1cm','0cm'})
axis([1 7 0 20000])
set(xlabel('Distance'),'FontSize',18);
set(ylabel('Raw value'),'FontSize',18);
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',16)
subplot(1,2,2)
plot(data_black(3,7:12)'), hold on
set(gca,'XTick',1:6)
set(gca,'XTickLabel',{'ON','1N ','2N ','3N ','4N ','5N '})
axis([1 6 0 20000])
set(xlabel('Force'),'FontSize',18);
set(gca,'XTickLabel',get(gca,'XTickLabel'),'fontsize',16)

saveas(h,'Figure9c','png')