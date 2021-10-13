function [] = plotFromEx(data, time, plotColor)

subplot(9,2,1);
plot(time,data(1,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(1,:))-0.1, max(data(1,:))+0.1]);
title('Arm Large Cyl [bar]');
subplot(9,2,2);
plot(time,data(2,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(2,:))-0.1, max(data(2,:))+0.1]);
title('Arm Small Cyl [bar]');
subplot(9,2,3);
plot(time,data(3,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(3,:))-0.1, max(data(3,:))+0.1]);
title('Swing Left [bar]');
subplot(9,2,4);
plot(time,data(4,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(4,:))-0.1, max(data(4,:))+0.1]);
title('Swing Right [bar]');
subplot(9,2,5);
plot(time,data(5,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(5,:))-0.1, max(data(5,:))+0.1]);
title('Boom Large Cyl [bar]');
subplot(9,2,6);
plot(time,data(6,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(6,:))-0.1, max(data(6,:))+0.1]);
title('Boom Small Cyl [bar]');
subplot(9,2,7);
plot(time,data(7,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(7,:))-0.1, max(data(7,:))+0.1]);
title('Bucket Large Cyl [bar]');
subplot(9,2,8);
plot(time,data(8,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(8,:))-0.1, max(data(8,:))+0.1]);
title('Bucket Small Cyl [bar]');

subplot(9,2,9);
plot(time,data(9,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(9,:))-0.0001, max(data(9,:))+0.0001]);
title('Body Pitch Angle [deg]');
subplot(9,2,10);
plot(time,data(10,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(10,:))-0.0001, max(data(10,:))+0.0001]);
title('Body Roll Angle [deg]');
subplot(9,2,11);
plot(time,data(11,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(11,:))-0.0001, max(data(11,:))+0.0001]);
title('Boom Pitch Angle [deg]');
subplot(9,2,12);
plot(time,data(12,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(12,:))-0.0001, max(data(12,:))+0.0001]);
title('Arm Pitch Angle [deg]');
subplot(9,2,13);
plot(time,data(13,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(13,:))-0.0001, max(data(13,:))+0.0001]);
title('Bucket Pitch Angle [deg]');
subplot(9,2,14);
plot(time,data(14,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(14,:))-0.0001, max(data(14,:))+0.0001]);
title('Swing Angle [deg]');

subplot(9,2,15);
plot(time,data(15,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(15,:)), max(data(15,:))]);
title('Boom Rate [deg/s]');
subplot(9,2,16);
plot(time,data(16,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(16,:)), max(data(16,:))]);
title('Arm Rate [deg/s]');
subplot(9,2,17);
plot(time,data(17,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(17,:)), max(data(17,:))]);
title('Bucket Rate [deg/s]');
subplot(9,2,18);
plot(time,data(18,:),plotColor,'LineWidth',2); hold on;
ylim([min(data(18,:)), max(data(18,:))]);
title('Swing Rate [deg/s]');

sgtitle('Pressure(8) Angle(6) Rate(4) -- from ex');

%%
figure('Position',[1100 1 900 973]);

subplot(3,2,1);
plot(time,data(19,:),'b','LineWidth',2); hold on;
% ylim([min(data(19,:)), max(data(19,:))]);
title('GPS Easting [m]');
subplot(3,2,3);
plot(time,data(20,:),'b','LineWidth',2); hold on;
% ylim([min(data(20,:)), max(data(20,:))]);
title('GPS Northing [m]');
subplot(3,2,5);
plot(time,data(21,:),'b','LineWidth',2); hold on;
% ylim([min(data(21,:)), max(data(21,:))]);
title('GPS Elevation [m]');
subplot(3,2,2);
plot(time,data(22,:),'r','LineWidth',2); hold on;
% ylim([min(data(22,:)), max(data(22,:))]);
title('GPS Latitude [deg]');
subplot(3,2,4);
plot(time,data(23,:),'r','LineWidth',2); hold on;
% ylim([min(data(23,:)), max(data(23,:))]);
title('GPS Longitude [deg]');
subplot(3,2,6);
plot(time,data(24,:),'r','LineWidth',2); hold on;
% ylim([min(data(24,:)), max(data(24,:))]);
title('GPS Sea Level [m]');

sgtitle('GPS1(col1) GPS2(col2) -- from ex');

end