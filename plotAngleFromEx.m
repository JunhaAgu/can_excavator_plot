function [] = plotAngleFromEx(data, time, plotColor)

subplot(3,2,1);
plot(time,data(13,:),plotColor,'LineWidth',2); hold on;
title('Body Pitch Angle [rad]');
subplot(3,2,2);
plot(time,data(14,:),plotColor,'LineWidth',2); hold on;
title('Body Roll Angle [rad]');
subplot(3,2,3);
plot(time,data(10,:),plotColor,'LineWidth',2); hold on;
title('Boom Pitch Angle [rad]');
subplot(3,2,4);
plot(time,data(11,:),plotColor,'LineWidth',2); hold on;
title('Arm Pitch Angle [rad]');
subplot(3,2,5);
plot(time,data(12,:),plotColor,'LineWidth',2); hold on;
title('Bucket Pitch Angle [rad]');
subplot(3,2,6);
plot(time,data(9,:),plotColor,'LineWidth',2); hold on;
title('Swing Angle [rad]');

sgtitle('Angle(from ex)');

end