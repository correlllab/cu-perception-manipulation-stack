%load data%
FAI = load('FAI.txt'); 
FAII = load('FAII.txt');
Fgl = load('SAI_Fgl.txt');
%Fgl = Fgl./sum(Fgl);
Fgr = load('SAI_Fgr.txt');
%Fgr = Fgr./sum(Fgr);
gripperAperture = load('gripperAperture.txt');

figure(1)
subplot(4,1,1);
xgripper = linspace(0,length(gripperAperture)/20, length(gripperAperture));
plot(xgripper,gripperAperture,'c', 'linewidth', .5)
axis([0 23 0 110])
set(gca, 'box','off')
title('Grasp Event Detection', 'fontsize', 14)
ylabel('% Gripper Aperture', 'FontSize', 10);

subplot(4,1,2)
xFgl = linspace(0,length(Fgl)/18.78,length(Fgl));
plot(xFgl,Fgl, 'linewidth', .5)
set(gca, 'box','off')
axis([0 23 30000 70000])
hold on
xFgr = linspace(0,length(Fgl)/18.78, length(Fgr));
plot(xFgr,Fgr,'linewidth', 1)
ylabel('SAI channel', 'FontSize', 10);
hold off

subplot(4,1,3)
xFAI = linspace(0,length(FAI)/18.78,length(FAI));
ylabel('FAI channel')
plot(xFAI, FAI,'m','linewidth', .5)
set(gca, 'box','off')
axis([0 23 -15000 15000])
ylabel('FAI channel', 'FontSize', 10);

subplot(4,1,4)
xFAII = linspace(0,length(FAII)/100,length(FAII));
xlabel('Time')
ylabel('FAII channel')
plot(xFAII, FAII,'b','linewidth', .5)
set(gca, 'box','off')
axis([0 23 0 4])
ylabel('FAII channel', 'FontSize', 10);
xlabel('Time(s)', 'fontsize', 12)



%print('graspevent', '-dpng')