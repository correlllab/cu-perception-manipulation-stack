D=load('calib_whitepaper.txt');
S=D(:,1:7);
R=reshape(S',size(S,1)*size(S,2),1);

%Xvalues=[0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 9]';
Xvalues=[0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10]'
X=repmat(Xvalues,1,size(S,2)*50);
X=reshape(X',size(S,1)*size(S,2),1);


%% Fit data to y=a/x^b+c
[f,gof]=fit(X,R,'power2','Robust', 'Bisquare','Lower',[-Inf -Inf -Inf],'Upper',[Inf Inf Inf])
% The data is of the form y = a*x^b+c, which is in line with
% physical intuition. We fix b=-2 to facilitate the calculation of the 
% inverse. The inverse is
% x=((y-c)/a)^(1/b), which can be used as a look-up function to
% assign a distance to a specific sensor reading. We set the minimum value ever measured 
% as the minimum for to avoid the square root yielding complex values. 

%% Plot fitted curve and data
h=figure(1)
plot(f,X,R)
set(xlabel('Distance (cm)'),'FontSize',14);
set(ylabel('16-bit value'),'FontSize',14);
set(title('No PDMS, white paper'),'FontSize',14);
disp(sprintf('R-Square Value %f',gof.rsquare))
saveas(h,'fit_whitepaper','png');

%% Plot calculated distances from sensor readings
h=figure(2)
Xest=((R-f.c)/f.a).^(1/f.b);
Xerror=reshape(Xest-X,length(Xest)/length(Xvalues),length(Xvalues));
%errorbar(Xvalues,mean(Xerror),min(Xerror),max(Xerror)), hold on;
errorbar(Xvalues,mean(Xerror),std(Xerror),'k')

set(title('Average error (No PDMS/white paper)'),'FontSize',14);
set(xlabel('distance (cm)'),'FontSize',14);
set(ylabel('Average error (cm)'),'FontSize',14);
saveas(h,'error_whitepaper','png');

%% Analyze different colors
h=figure(3), hold on;
% 1. Yellow paper
D=load('calib_yellowpaper.txt');
S=D(:,1:7);
R=reshape(S',size(S,1)*size(S,2),1);
Xvalues=[0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10]'
X=repmat(Xvalues,1,350);
X=reshape(X',size(S,1)*size(S,2),1);
%[f,gof]=fit(X,R,'power2','Robust', 'Bisquare','Lower',[-Inf -Inf -Inf],'Upper',[Inf Inf Inf])
Xest=((R-f.c)/f.a).^(1/f.b);
Xerror=reshape(Xest-X,length(Xest)/length(Xvalues),length(Xvalues));
errorbar(Xvalues,mean(Xerror),std(Xerror),'y')
% 2. Red paper
D=load('calib_redpaper.txt');
S=D(:,1:7);
R=reshape(S',size(S,1)*size(S,2),1);
Xvalues=[0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10]'
X=repmat(Xvalues,1,350);
X=reshape(X',size(S,1)*size(S,2),1);
%[f,gof]=fit(X,R,'power2','Robust', 'Bisquare','Lower',[-Inf -Inf -Inf],'Upper',[Inf Inf Inf])
Xest=((R-f.c)/f.a).^(1/f.b);
Xerror=reshape(Xest-X,length(Xest)/length(Xvalues),length(Xvalues));
errorbar(Xvalues,mean(Xerror),std(Xerror),'r')
% 3. Gray paper
D=load('calib_graypaper.txt');
S=D(:,1:7);
R=reshape(S',size(S,1)*size(S,2),1);
Xvalues=[0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10]'
X=repmat(Xvalues,1,350);
X=reshape(X',size(S,1)*size(S,2),1);
%[f,gof]=fit(X,R,'power2','Robust', 'Bisquare','Lower',[-Inf -Inf -Inf],'Upper',[Inf Inf Inf])
Xest=((R-f.c)/f.a).^(1/f.b);
Xerror=reshape(Xest-X,length(Xest)/length(Xvalues),length(Xvalues));
errorbar(Xvalues,mean(Xerror),std(Xerror),'g')
% 4. Black paper
D=load('calib_blackpaper.txt');
S=D(:,1:7);
R=reshape(S',size(S,1)*size(S,2),1);
Xvalues=[0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10]'
X=repmat(Xvalues,1,350);
X=reshape(X',size(S,1)*size(S,2),1);
%[f,gof]=fit(X,R,'power2','Robust', 'Bisquare','Lower',[-Inf -Inf -Inf],'Upper',[Inf Inf Inf])
Xest=(((R-f.c)/f.a).^(1/f.b));
Xerror=reshape(Xest-X,length(Xest)/length(Xvalues),length(Xvalues));
errorbar(Xvalues,mean(Xerror),std(Xerror),'k')

set(legend('Yellow paper','Red paper','Gray paper','Black Paper'),'FontSize',14);
set(title('Average error (No PDMS/colored paper)'),'FontSize',14);
set(xlabel('distance (cm)'),'FontSize',14);
set(ylabel('Average error (cm)'),'FontSize',14);
saveas(h,'error_colorpapers','png');

