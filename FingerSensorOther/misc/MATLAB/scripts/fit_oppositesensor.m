D=load('../../misc/14_sensorwhiteppr_PDMS.txt');
B=load('../../misc/baseValues_14sensors.txt');
B=repmat(mean(B),size(D,1),1);
D=10*log10(D./B);

S=D(:,[1:7 9:15]);

Xvalues=[0.5 1 1.5 2 2.5 3 3.5 4 4.5 5:19]';
X=repmat(Xvalues,1,2*50);
X=reshape(X',size(S,1)*2,1);
S1 = S';
R=[];
for i=1:7
    S2([2*i-1 2*i],:) = S1([i i+7],:);
    S3=S2([2*i-1 2*i],:);
    R(:,i) = reshape(S3,size(S,1)*2,1);
    f{i}=fit(X,R(:,i),'power2','Lower',[-Inf -Inf -Inf],'Upper',[Inf Inf Inf]);
end    
