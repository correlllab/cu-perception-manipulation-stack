serial_ = serial('/dev/ttyACM0');
fopen(serial_);
if(serial_.BytesAvailable)
  fscanf(serial_);
end

left=[];
R = [];

for k=1:10  %weights
for j=1:1 %current values
for i=1:20 %number of values
    fprintf(serial_,'m');
    R=fscanf(serial_,'%d \n');
    if isempty(R)==0 
    left =  [left;R];
    end
    
end
left = mean(left);
data(j,k)=left;
left=[]; %reinitialise an empty array
R = [];

fclose(serial_);
input('Change the current and press ENTER')

fopen(serial_);
if(serial_.BytesAvailable)
  fscanf(serial_);
end

end
input('Change the weight and press ENTER')
end
fclose(serial_);
