clc; clear all; close all;

%% data directory
dir = '/home/junhakim/hce_plot_data';
dataname = '/2021_10_14/14test_5';

%% 2021-07-30, Junha Test
bagfile_name = 'bagfile.bag';

%% plot flag
%flag_from_ex_to_gcs
%flag_from_gcs_to_local
%flag_from_local_to_gsc
%flag_from_gcs_to_ex 
flag_plot = [ 1 1 1 1 ];

%% load rosbag file

bagfile = [dir,dataname, '/' , bagfile_name];
bag = rosbag(bagfile);
bag.AvailableTopics;

bag_canpackets_fromexc = select(bag,'Topic','/canpackets/FromExcavator');
bag_canpackets_toexc = select(bag,'Topic','/canpackets/ToExcavator');

bag_state = select(bag,'Topic','/to_local_planning');
bag_MPC_sp_HCE = select(bag,'Topic','/local_planning_setpoints'); % 0.2, 0.5, 1.0

msg_canpackets_fromexc = readMessages(bag_canpackets_fromexc,'DataFormat','struct');
msg_canpackets_toexc = readMessages(bag_canpackets_toexc,'DataFormat','struct');

msg_state = readMessages(bag_state,'DataFormat','struct');
msg_MPC_sp_HCE = readMessages(bag_MPC_sp_HCE,'DataFormat','struct');

%%
% data_num = '1';
% load(['0119_exp_data',data_num,'.mat']);

%% min time
%time1_0 = 
%time2_0 = 
time3_0 = double(msg_canpackets_toexc{1, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_canpackets_toexc{1, 1}.Header.Stamp.Nsec);
time4_0 = double(msg_MPC_sp_HCE{1, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_MPC_sp_HCE{1, 1}.Header.Stamp.Nsec) ;
min_time = min(time3_0,time4_0);

%% message from excavator to gcs

n_canpackets_fromexc = size(msg_canpackets_fromexc,1);
byte1 = zeros(68,n_canpackets_fromexc);
data1 = zeros(24,n_canpackets_fromexc);
time1 = zeros(1, n_canpackets_fromexc);
angle_scale = 3.0517578125e-5;
angle_offset = -250;
for i=1: n_canpackets_fromexc
    byte1(:,i) = msg_canpackets_fromexc{i, 1}.Bytes;
    time1(1,i) = double(msg_canpackets_fromexc{i, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_canpackets_fromexc{i, 1}.Header.Stamp.Nsec) ...
        - double(msg_canpackets_fromexc{1, 1}.Header.Stamp.Sec) ...
        - 1e-9*double(msg_canpackets_fromexc{1, 1}.Header.Stamp.Nsec);
end
data1 = decodeFromEx(byte1, data1, angle_scale, angle_offset);

if flag_plot(1,1)==1
    f1 =  figure('Position',[66 1 928 973]);  %figure('Position',[993 1 928 973]);
    plotPressureAngleFromEx(data1, time1,'r'); hold on;
end

%% message from gcs to local_planner

n_msg_state_tompc = size(msg_state,1);
data2 = zeros(18,n_msg_state_tompc);
time2 = zeros(1, n_msg_state_tompc);

for i=1: n_msg_state_tompc
    data2(1,i) = msg_state{i, 1}.Measure.Pressure.ACylLC;
    data2(2,i) = msg_state{i, 1}.Measure.Pressure.ACylSC;
    data2(3,i) = msg_state{i, 1}.Measure.Pressure.SwingL;
    data2(4,i) = msg_state{i, 1}.Measure.Pressure.SwingR;
    data2(5,i) = msg_state{i, 1}.Measure.Pressure.BCylLC;
    data2(6,i) = msg_state{i, 1}.Measure.Pressure.BCylSC;
    data2(7,i) = msg_state{i, 1}.Measure.Pressure.KCylLC;
    data2(8,i) = msg_state{i, 1}.Measure.Pressure.KCylSC;
    
    % [Degree]
    data2(9,i) = 180/pi()*msg_state{i, 1}.Measure.Angle.ThetaBody;
    data2(10,i) = 180/pi()*msg_state{i, 1}.Measure.Angle.PhiBody;
    data2(11,i) = 180/pi()*msg_state{i, 1}.Measure.Angle.ThetaB;
    data2(12,i) = 180/pi()*msg_state{i, 1}.Measure.Angle.ThetaA;
    data2(13,i) = 180/pi()*msg_state{i, 1}.Measure.Angle.ThetaK;
    data2(14,i) = 180/pi()*msg_state{i, 1}.Measure.Angle.PsiU;
    data2(15,i) = 180/pi()*msg_state{i, 1}.Measure.Velocity.ThetaBDot;
    data2(16,i) = 180/pi()*msg_state{i, 1}.Measure.Velocity.ThetaADot;
    data2(17,i) = 180/pi()*msg_state{i, 1}.Measure.Velocity.ThetaKDot;
    data2(18,i) = 180/pi()*msg_state{i, 1}.Measure.Velocity.PsiUDot;
%     data2(9,i) = msg_state{i, 1}.Measure.Angle.ThetaBody;
%     data2(10,i) = msg_state{i, 1}.Measure.Angle.PhiBody;
%     data2(11,i) = msg_state{i, 1}.Measure.Angle.ThetaB;
%     data2(12,i) = msg_state{i, 1}.Measure.Angle.ThetaA;
%     data2(13,i) = msg_state{i, 1}.Measure.Angle.ThetaK;
%     data2(14,i) = msg_state{i, 1}.Measure.Angle.PsiU;
%     data2(15,i) = msg_state{i, 1}.Measure.Velocity.ThetaBDot;
%     data2(16,i) = msg_state{i, 1}.Measure.Velocity.ThetaADot;
%     data2(17,i) = msg_state{i, 1}.Measure.Velocity.ThetaKDot;
%     data2(18,i) = msg_state{i, 1}.Measure.Velocity.PsiUDot;
    
    time2(1,i) = double(msg_state{i, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_state{i, 1}.Header.Stamp.Nsec) ...
        - double(msg_state{1, 1}.Header.Stamp.Sec) ...
        - 1e-9*double(msg_state{1, 1}.Header.Stamp.Nsec);
end

if flag_plot(1,2)==1
    figure(f1); plotPressureAngleFromEx(data2, time2,'b');
    legend('from ex to gcs', 'from gcs to local');
end

%% ==================================================================== %%

%% ==================================================================== %%

%% message from gcs to excavator
n_canpackets_toexc = size(msg_canpackets_toexc,1);
byte3 = zeros(72,n_canpackets_toexc);
data3 = zeros(36,n_canpackets_toexc);
time3 = zeros(1, n_canpackets_toexc);
for i=1: n_canpackets_toexc
    byte3(:,i) = msg_canpackets_toexc{i, 1}.Bytes;
    time3(1,i) = double(msg_canpackets_toexc{i, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_canpackets_toexc{i, 1}.Header.Stamp.Nsec) ...
        - min_time;
        %- double(msg_canpackets_toexc{1, 1}.Header.Stamp.Sec) ...
        %- 1e-9*double(msg_canpackets_toexc{1, 1}.Header.Stamp.Nsec);
end
data3 = calRealToEx(byte3, data3);

if flag_plot(1,4)==1
    f2 =  figure('Position',[993 1 928 973]);
    plotToEx(data3, time3,'r',0.2); hold on;
    f3 =  figure('Position',[993 1 928 973]);
    plotToEx(data3, time3,'r',0.5); hold on;
    f4 =  figure('Position',[993 1 928 973]);
    plotToEx(data3, time3,'r',1.0); hold on;
end
%% message from local_planner to gcs
n_msg_MPC_sp_HCE = size(msg_MPC_sp_HCE,1);
data4 = zeros(18,n_msg_MPC_sp_HCE);
time4 = zeros(1, n_msg_MPC_sp_HCE);

for i=1: n_msg_MPC_sp_HCE
    data4(1,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(1,1);
    data4(2,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(2,1);
    data4(3,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(3,1);
    data4(4,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(4,1);
    data4(5,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(5,1);
    data4(6,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(6,1);
    data4(7,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(7,1);
    data4(8,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).State(8,1);
    
    data4(9,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).Input(1,1);
    data4(10,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).Input(2,1);
    data4(11,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).Input(3,1);
    data4(12,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,1).Input(4,1);
    
    data4(13,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(1,1);
    data4(14,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(2,1);
    data4(15,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(3,1);
    data4(16,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(4,1);
    data4(17,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(5,1);
    data4(18,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(6,1);
    data4(19,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(7,1);
    data4(20,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).State(8,1);
    
    data4(21,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).Input(1,1);
    data4(22,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).Input(2,1);
    data4(23,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).Input(3,1);
    data4(24,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,2).Input(4,1);
    
    data4(25,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(1,1);
    data4(26,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(2,1);
    data4(27,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(3,1);
    data4(28,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(4,1);
    data4(29,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(5,1);
    data4(30,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(6,1);
    data4(31,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(7,1);
    data4(32,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).State(8,1);
    
    data4(33,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).Input(1,1);
    data4(34,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).Input(2,1);
    data4(35,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).Input(3,1);
    data4(36,i) = msg_MPC_sp_HCE{i, 1}.Predictions_(1,3).Input(4,1);
    
    
    time4(1,i) = double(msg_MPC_sp_HCE{i, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_MPC_sp_HCE{i, 1}.Header.Stamp.Nsec) ...
        - min_time;
        %- double(msg_MPC_sp_HCE{1, 1}.Header.Stamp.Sec) ...
        %- 1e-9*double(msg_MPC_sp_HCE{1, 1}.Header.Stamp.Nsec) ;
end

if flag_plot(1,4)==1
    figure(f2);
    plotToEx(data4, time4,'b--',0.2);
    legend('from gcs to ex', 'from local to gcs');
    figure(f3); hold on;
    plotToEx(data4, time4,'b--',0.5);
    legend('from gcs to ex', 'from local to gcs');
    figure(f4); hold on;
    plotToEx(data4, time4,'b--',1.0);
    legend('from gcs to ex', 'from local to gcs');
end

%% CAN comunication dt plot
% ex -> gcs
time1_inter = zeros(1,size(time1,2)-1);
for i=1:size(time1,2)-1
    time1_inter(1,i) = time1(1,i+1) - time1(1,i);
end
m = mean(time1_inter(1,1:end));
s = std(time1_inter(1,1:end));
figure(); plot(time1_inter,'c');
m_ = ['Mean = ',num2str(m),' [s]'];
s_ = ['std = ',num2str(s),' [s]'];
yline(m,'-',m_,'LineWidth',1,'Color','b');
yline(m,'-',s_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
xlabel('# of data'); ylabel('dt [s]');
title('exc -> gcs');
grid on;
xlim([1, length(time1_inter(1,1:end))]);
ylim([0.09 0.11]);

% gcs -> local
time2_inter = zeros(1,size(time2,2)-1);
for i=1:size(time2,2)-1
    time2_inter(1,i) = time2(1,i+1) - time2(1,i);
end
m = mean(time2_inter(1,1:end));
s = std(time2_inter(1,1:end));
figure(); plot(time2_inter,'c');
m_ = ['Mean = ',num2str(m),' [s]'];
s_ = ['std = ',num2str(s),' [s]'];
yline(m,'-',m_,'LineWidth',1,'Color','b');
yline(m,'-',s_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
xlabel('# of data'); ylabel('dt [s]');
title('gcs -> local');
grid on;
xlim([1, length(time2_inter(1,1:end))]);
ylim([0.09 0.11]);

% gcs -> ex
time3_inter = zeros(1,size(time3,2)-1);
for i=1:size(time3,2)-1
    time3_inter(1,i) = time3(1,i+1) - time3(1,i);
end
m = mean(time3_inter(1,1:end));
s = std(time3_inter(1,1:end));
figure(); plot(time3_inter,'c');
m_ = ['Mean = ',num2str(m),' [s]'];
s_ = ['std = ',num2str(s),' [s]'];
yline(m,'-',m_,'LineWidth',1,'Color','b');
yline(m,'-',s_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
xlabel('# of data'); ylabel('dt [s]');
title('gcs -> ex');
grid on;
xlim([1, length(time3_inter(1,1:end))]);
ylim([0.09 0.11]);

% local -> gcs
time4_inter = zeros(1,size(time4,2)-1);
for i=1:size(time4,2)-1
    time4_inter(1,i) = time4(1,i+1) - time4(1,i);
end
m = mean(time4_inter(1,1:end));
s = std(time4_inter(1,1:end));
figure(); plot(time4_inter,'c');
m_ = ['Mean = ',num2str(m),' [s]'];
s_ = ['std = ',num2str(s),' [s]'];
yline(m,'-',m_,'LineWidth',1,'Color','b');
yline(m,'-',s_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
xlabel('# of data'); ylabel('dt [s]');
title('local -> gcs');
grid on;
xlim([1, length(time4_inter(1,1:end))]);
ylim([0.09 0.11]);
