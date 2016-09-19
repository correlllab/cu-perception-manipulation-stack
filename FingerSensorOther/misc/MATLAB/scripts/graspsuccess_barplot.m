y = [20 20; 20 18; 20 12; 20 11; 20 6; 18 5; 17 4];
gca = bar(y);
title('Grasp Success Rate', 'fontsize', 14)
xlabel('Error*random[-0.01, 0.01]', 'fontsize', 14)
ylabel ('No. of success grasp', 'fontsize', 14)
axis([0 8 0 23])
grid on 
legend('Controller', 'No controller')
%print('graspsuccess_barplot','-dpng')