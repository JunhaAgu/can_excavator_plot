function [] = plotToEx(data, time, plotColor, a)

if a == 0.2
    start_id = 0;
elseif a == 0.5
    start_id = 12;
elseif a == 1.0
    start_id = 24;
end

%% 0.2s
subplot(6,2,1);
plot(time,data(1+start_id,:),plotColor,'LineWidth',2); hold on;
title('Swing  Angle');
subplot(6,2,2);
plot(time,data(2+start_id,:),plotColor,'LineWidth',2); hold on;
title('Boom Length');
subplot(6,2,3);
plot(time,data(3+start_id,:),plotColor,'LineWidth',2); hold on;
title('Arm Length');
subplot(6,2,4);
plot(time,data(4+start_id,:),plotColor,'LineWidth',2); hold on;
title('Bucket Length');

subplot(6,2,5);
plot(time,data(5+start_id,:),plotColor,'LineWidth',2); hold on;
title('Swing  Rate');
subplot(6,2,6);
plot(time,data(6+start_id,:),plotColor,'LineWidth',2); hold on;
title('Boom Rate');
subplot(6,2,7);
plot(time,data(7+start_id,:),plotColor,'LineWidth',2); hold on;
title('Arm Rate');
subplot(6,2,8);
plot(time,data(8+start_id,:),plotColor,'LineWidth',2); hold on;
title('Bucket Rate');

subplot(6,2,9);
plot(time,data(9+start_id,:),plotColor,'LineWidth',2); hold on;
title('Swing  Torque');
subplot(6,2,10);
plot(time,data(10+start_id,:),plotColor,'LineWidth',2); hold on;
title('Boom Force');
subplot(6,2,11);
plot(time,data(11+start_id,:),plotColor,'LineWidth',2); hold on;
title('Arm Force');
subplot(6,2,12);
plot(time,data(12+start_id,:),plotColor,'LineWidth',2); hold on;
title('Bucket Force');

if a == 0.2
    sgtitle('0.2s state(8) & input(4)');
elseif a == 0.5
    sgtitle('0.5s state(8) & input(4)');
elseif a == 1.0
    sgtitle('1.0s state(8) & input(4)');
end
%% 0.5s
% figure();
% subplot(6,2,1);
% plot(time,data(13,:),plotColor,'LineWidth',2); hold on;
% title('Swing  Angle');
% subplot(6,2,2);
% plot(time,data(14,:),plotColor,'LineWidth',2); hold on;
% title('Boom Length');
% subplot(6,2,3);
% plot(time,data(15,:),plotColor,'LineWidth',2); hold on;
% title('Arm Length');
% subplot(6,2,4);
% plot(time,data(16,:),plotColor,'LineWidth',2); hold on;
% title('Bucket Length');
% sgtitle('Angle & Lenght (0.5s)');
% 
% figure();
% subplot(6,2,1);
% plot(time,data(17,:),plotColor,'LineWidth',2); hold on;
% title('Swing  Rate');
% subplot(6,2,2);
% plot(time,data(18,:),plotColor,'LineWidth',2); hold on;
% title('Boom Rate');
% subplot(6,2,3);
% plot(time,data(19,:),plotColor,'LineWidth',2); hold on;
% title('Arm Rate');
% subplot(6,2,4);
% plot(time,data(60,:),plotColor,'LineWidth',2); hold on;
% title('Bucket Rate');
% sgtitle('Angle rate & velocity (0.5s)');
% 
% figure();
% subplot(6,2,1);
% plot(time,data(61,:),plotColor,'LineWidth',2); hold on;
% title('Swing  Torque');
% subplot(6,2,2);
% plot(time,data(62,:),plotColor,'LineWidth',2); hold on;
% title('Boom Force');
% subplot(6,2,3);
% plot(time,data(63,:),plotColor,'LineWidth',2); hold on;
% title('Arm Force');
% subplot(6,2,4);
% plot(time,data(64,:),plotColor,'LineWidth',2); hold on;
% title('Bucket Force');
% sgtitle('Torque & Force (0.5s)');
% 
% %% 1.0s
% figure();
% subplot(6,2,1);
% plot(time,data(65,:),plotColor,'LineWidth',2); hold on;
% title('Swing  Angle');
% subplot(6,2,2);
% plot(time,data(66,:),plotColor,'LineWidth',2); hold on;
% title('Boom Length');
% subplot(6,2,3);
% plot(time,data(67,:),plotColor,'LineWidth',2); hold on;
% title('Arm Length');
% subplot(6,2,4);
% plot(time,data(68,:),plotColor,'LineWidth',2); hold on;
% title('Bucket Length');
% sgtitle('Angle & Lenght (1.0s)');
% 
% figure();
% subplot(6,2,1);
% plot(time,data(69,:),plotColor,'LineWidth',2); hold on;
% title('Swing  Rate');
% subplot(6,2,2);
% plot(time,data(30,:),plotColor,'LineWidth',2); hold on;
% title('Boom Rate');
% subplot(6,2,3);
% plot(time,data(31,:),plotColor,'LineWidth',2); hold on;
% title('Arm Rate');
% subplot(6,2,4);
% plot(time,data(32,:),plotColor,'LineWidth',2); hold on;
% title('Bucket Rate');
% sgtitle('Angle rate & velocity (1.0s)');
% 
% figure();
% subplot(6,2,1);
% plot(time,data(33,:),plotColor,'LineWidth',2); hold on;
% title('Swing  Torque');
% subplot(6,2,2);
% plot(time,data(34,:),plotColor,'LineWidth',2); hold on;
% title('Boom Force');
% subplot(6,2,3);
% plot(time,data(35,:),plotColor,'LineWidth',2); hold on;
% title('Arm Force');
% subplot(6,2,4);
% plot(time,data(36,:),plotColor,'LineWidth',2); hold on;
% title('Bucket Force');
% sgtitle('Torque & Force (1.0s)');
end