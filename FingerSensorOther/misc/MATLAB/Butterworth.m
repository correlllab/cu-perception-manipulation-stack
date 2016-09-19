[b,a] = butter(1,0.48,'high');
dataIn = load('towercollapse.txt');

% normalize [0 1]
%for j=1:size(dataOut,2),for i=1:size(dataOut,1), dataOut3(i,j) = (dataOut(i,j)-min(dataOut(:,j)))/((dataOut(:,j))-min(dataOut(:,j)));end,end

% subtract base values from each sensor
dataIn2 = []; for j=1:size(dataIn,2),for i=1:size(dataIn,1), dataIn2(i,j) = dataIn(i,j)-min(dataIn(:,j)); end,end

for j=1:1:16, dataOut2(:,j) = filter(b,a,dataIn2(:,j)); end
% sum every sensor value at every epoch
 dataOut2Sum = sum(dataOut2,2);