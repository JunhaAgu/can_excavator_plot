figure();
subplot(2,2,1);
plot(time,data(16,:),'r','LineWidth',2); hold on;
title('Boom Joint Rate [rad/s]');
subplot(2,2,2);
plot(time,data(17,:),'r','LineWidth',2); hold on;
title('Arm Joint Rate [rad/s]');
subplot(2,2,3);
plot(time,data(18,:),'r','LineWidth',2); hold on;
title('Bucket Joint Rate [rad/s]');
subplot(2,2,4);
plot(time,data(15,:),'r','LineWidth',2); hold on;
title('Swing Rate [rad/s]');

sgtitle('Rate(from ex)');