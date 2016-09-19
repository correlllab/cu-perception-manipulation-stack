D=load('wineglass.txt');
%D=load('ikea_cup');
% The first colum contains the angle, 2-8 the first sensor strip and 9-15
% the second strip's data.

w=12; % distance between sensors (cm)
%load model;
%f=fit(X',Y','exp2');

% Convert into meters
%R=f([D(:,2:8) D(:,10:16)]);
%R=reshape(R,size(R,1)/14,14);
load calib
% Read measurements
R=[D(:,2:8) D(:,10:16)];

% Read base values
B=load('baseValues_14sensors.txt');
B=[B(:,2:8) B(:,10:16)];
B=repmat(mean(B),size(R,1),1);

% Convert into decibles
Rdb=10*log10(R./B);

%Rm=((Rdb-fpdms.c)/fpdms.a).^(1/(fpdms.b)); 
Rm=((Rdb-fpdms.c)/fpdms.a).^(-1); 

% Filter sensor values
Fvalues=abs(Rm)<w/2;
Z=repmat([7 6 5 4 3 2 1 7 6 5 4 3 2 1],size(D,1),1);

X=(w/2-Rm).*[repmat(cos(D(:,1)),1,7) repmat(cos(D(:,1)+pi),1,7)];
Y=(w/2-Rm).*[repmat(sin(D(:,1)),1,7) repmat(sin(D(:,1)+pi),1,7)];

%subplot(1,2,1)
%surf(X,Y,Z); axis([-10 10 -10 10 0 10])

hold on
for I=1:14, 
 for J=1:size(Rm,1),
    if(Fvalues(J,I)),
     if(I<=7) 
         plot3(X(J,I),Y(J,I),Z(J,I),'g.','markersize',9); 
     else
         plot3(X(J,I),Y(J,I),Z(J,I),'r.','markersize',9);
     end;
    end;
 end;    
end;
drawnow; 


axis([-10 10 -10 10 0 10])
hold off