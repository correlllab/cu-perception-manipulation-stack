% load data file
file_path = fileparts(which('nikolausexpsetup.mat')); % start in ../ir_sensor/matlab/data
[file_name, path] = uigetfile('*.mat','Select data file');
data = load([path file_name]);
data1 = data.left;
figure(1)
 plot(data1(:,2),'r');
 hold on
 plot(data1(:,3),'k');
 hold on
 plot(data1(:,4),'b');
 hold on
 plot(data1(:,5),'g');

