aa = load('bluejug.txt');
bb = [aa(:,1:7) aa(:,9:15)];
for j=1:size(bb,2),for i=1:size(bb,1), dd(i,j) = (bb(i,j)-min(bb(:,j)))/(max(bb(:,j))-min(bb(:,j)));end,end
cc = normc(bb);