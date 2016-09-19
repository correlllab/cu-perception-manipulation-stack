serial_ = serial('/dev/ttyACM1');
fopen(serial_);
if(serial_.BytesAvailable)
  fscanf(serial_);
end

left=[];
R = [];

for k=12:12 %board positions
for j=1:5 %current values
for i=1:20 %number of values
    fprintf(serial_,'m');
    R=fscanf(serial_,'%d \n');
    if isempty(R)==0 
    left =  [left;R];
    end
    
end
left = mean(left);
data_red(j,k)=left;
left=[]; %reinitialise an empty array
R = [];

fclose(serial_);
input('Change the current and press ENTER')

fopen(serial_);
if(serial_.BytesAvailable)
  fscanf(serial_);
end

end
input('Move the board and press ENTER')
end
fclose(serial_);
