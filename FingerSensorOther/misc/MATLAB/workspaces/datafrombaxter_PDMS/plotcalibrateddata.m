% load data file
file_path = fileparts(which('process_min_max_data')); % start in ../ir_sensor/matlab/data
[file_name, path] = uigetfile('*.mat','Select data file');
data = load([path file_name]);
data1 = data.data_;
data2=[];
for i= 1:1:5
    bb = (137/(max(data1(:,3)-min(data1(:,3)))))*data1; %scaling
    offset = (137/(max(data1(:,3)-min(data1(:,3)))))*(max(data1(:,3)));
    data2(:,i) = bb - offset; %calibrated data
end

figure(1)
 plot(data1(:,3),'k');
 hold on
 plot(data1(:,5),'y');
 hold on
 plot(data1(:,7),'b');
 hold on
 plot(data1(:,9),'g');
 hold on
 plot(data1(:,11),'k');
 hold on
 plot(data1(:,3),'r');
 hold on

