% load data file
file_path = fileparts(which('process_min_max_data')); % start in ../ir_sensor/matlab/data
[file_name, path] = uigetfile('*.mat','Select data file');
data = load([path file_name]);
data1 = data.data_;
data2=[];
bb=[];
offset=[];
maxlength=95;

for i= 3:2:11
    bb(:,i) = (maxlength/(max(data1(51:75,i)-min(data1(51:75,i)))))*data1(51:75,i); %scaling
    offset(1,i) = maxlength - ((maxlength/(max(data1(51:75,i)-min(data1(51:75,i)))))*(max(data1(51:75,i))));
    data2(:,i) = bb(:,i) + offset(1,i); %calibrated data
end

figure(1)
 x=[44:-1:20];
 ax = gca;
 ax.XDir = 'reverse';
 plot(x',data2(:,3),'Xr');
 hold on
 plot(x',data2(:,5),'Xy');
 hold on
 plot(x',data2(:,7),'Xb');
 hold on
 plot(x',data2(:,9),'Xg');
 hold on
 plot(x',data2(:,11),'Xk');
