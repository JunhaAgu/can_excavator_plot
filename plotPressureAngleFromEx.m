function [] = plotPressureAngleFromEx(data, time, plotColor)

subplot(9,2,1);
plot(time,data(1,:),plotColor,'LineWidth',2); hold on;
title('Arm Large Cyl [bar]');
subplot(9,2,2);
plot(time,data(2,:),plotColor,'LineWidth',2); hold on;
title('Arm Small Cyl [bar]');
subplot(9,2,3);
plot(time,data(3,:),plotColor,'LineWidth',2); hold on;
title('Swing Left [bar]');
subplot(9,2,4);
plot(time,data(4,:),plotColor,'LineWidth',2); hold on;
title('Swing Right [bar]');
subplot(9,2,5);
plot(time,data(5,:),plotColor,'LineWidth',2); hold on;
title('Boom Large Cyl [bar]');
subplot(9,2,6);
plot(time,data(6,:),plotColor,'LineWidth',2); hold on;
title('Boom Small Cyl [bar]');
subplot(9,2,7);
plot(time,data(7,:),plotColor,'LineWidth',2); hold on;
title('Bucket Large Cyl [bar]');
subplot(9,2,8);
plot(time,data(8,:),plotColor,'LineWidth',2); hold on;
title('Bucket Small Cyl [bar]');

subplot(9,2,9);
plot(time,data(9,:),plotColor,'LineWidth',2); hold on;
title('Body Pitch Angle [rad]');
subplot(9,2,10);
plot(time,data(10,:),plotColor,'LineWidth',2); hold on;
title('Body Roll Angle [rad]');
subplot(9,2,11);
plot(time,data(11,:),plotColor,'LineWidth',2); hold on;
title('Boom Pitch Angle [rad]');
subplot(9,2,12);
plot(time,data(12,:),plotColor,'LineWidth',2); hold on;
title('Arm Pitch Angle [rad]');
subplot(9,2,13);
plot(time,data(13,:),plotColor,'LineWidth',2); hold on;
title('Bucket Pitch Angle [rad]');
subplot(9,2,14);
plot(time,data(14,:),plotColor,'LineWidth',2); hold on;
title('Swing Angle [rad]');

subplot(9,2,15);
plot(time,data(15,:),plotColor,'LineWidth',2); hold on;
ylim([-0.2 0.2]);
title('Boom Rate [rad]');
subplot(9,2,16);
plot(time,data(16,:),plotColor,'LineWidth',2); hold on;
ylim([-0.2 0.2]);
title('Arm Rate [rad]');
subplot(9,2,17);
plot(time,data(17,:),plotColor,'LineWidth',2); hold on;
ylim([-0.2 0.2]);
title('Bucket Rate [rad]');
subplot(9,2,18);
plot(time,data(18,:),plotColor,'LineWidth',2); hold on;
ylim([-0.2 0.2]);
title('Swing Rate [rad]');

sgtitle('Pressure(8) Angle(6) Rat(4) -- from ex');

end