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
    plotFromEx(data1, time1,'r'); hold on;
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
% ylim([0.09 0.11]);