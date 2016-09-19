serial_ = serial('/dev/ttyACM1');
fopen(serial_);
if(serial_.BytesAvailable)
  fscanf(serial_);
end;
%figure
%set(gcf,'DoubleBuffer','On');

left=[];
right=[];
for j=1:6
for i=1:10
    fprintf(serial_,'m');
    R=fscanf(serial_,'%d %d %d %d %d\n')
    left=[left; R'];
    %R=fscanf(serial_,'%d %d %d %d %d\n')
    %right=[right; R'];
    %bar(R)
    %drawnow;
    %axis([1 5 0 65535])        
end;
input('move the baxter finger and press ENTER')
end

fclose(serial_);