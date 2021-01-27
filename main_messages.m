clc; clear all; close all;

%% 2021-01-19 experiment
bagfile1 = 'exp_communication_2021-01-19-14-38-10.bag';
bagfile2 = 'exp_communication_2021-01-19-14-43-22.bag';
bagfile3 = 'exp_communication_2021-01-19-14-55-00.bag';

%% 2021-01-25, 26 test
bagfile_test = 'exp_test_2021-01-25-17-23-31.bag';
bagfile_test2 = 'exp_test_2021-01-25-19-53-11.bag';
bagfile_test3 = 'exp_test_2021-01-26-16-31-29.bag';

%% 2021-01-27 experiment
bagfile_s1 = 'exp_test_2021-01-27-10-31-11.bag';
bagfile_s2 = 'exp_test_2021-01-27-10-45-29.bag';
bagfile_s3 = 'exp_test_2021-01-27-10-50-53.bag';
bagfile_s4 = 'exp_test_2021-01-27-10-57-10.bag';
bagfile_s5 = 'exp_test_2021-01-27-10-57-45.bag';

%% load rosbag file

bag = rosbag(bagfile_s3);
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
byte1 = zeros(34,n_canpackets_fromexc);
data1 = zeros(18,n_canpackets_fromexc);
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
f1 =  figure('Position',[66 1 928 973]);  %figure('Position',[993 1 928 973]);
plotPressureAngleFromEx(data1, time1,'r'); hold on;
% figure(2); plotAngleFromEx(data1, time1,'r'); hold on;

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
    data2(9,i) = msg_state{i, 1}.Measure.Angle.ThetaBody;
    data2(10,i) = msg_state{i, 1}.Measure.Angle.PhiBody;
    data2(11,i) = msg_state{i, 1}.Measure.Angle.ThetaB;
    data2(12,i) = msg_state{i, 1}.Measure.Angle.ThetaA;
    data2(13,i) = msg_state{i, 1}.Measure.Angle.ThetaK;
    data2(14,i) = msg_state{i, 1}.Measure.Angle.PsiU;
    data2(15,i) = msg_state{i, 1}.Measure.Velocity.ThetaBDot;
    data2(16,i) = msg_state{i, 1}.Measure.Velocity.ThetaADot;
    data2(17,i) = msg_state{i, 1}.Measure.Velocity.ThetaKDot;
    data2(18,i) = msg_state{i, 1}.Measure.Velocity.PsiUDot;
    
    time2(1,i) = double(msg_state{i, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_state{i, 1}.Header.Stamp.Nsec) ...
        - double(msg_state{1, 1}.Header.Stamp.Sec) ...
        - 1e-9*double(msg_state{1, 1}.Header.Stamp.Nsec);
end
figure(f1); plotPressureAngleFromEx(data2, time2,'b');
legend('from ex to gcs', 'from gcs to local');

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

f2 =  figure('Position',[993 1 928 973]);
plotToEx(data3, time3,'r',0.2); hold on;
f3 =  figure('Position',[993 1 928 973]);
plotToEx(data3, time3,'r',0.5); hold on;
f4 =  figure('Position',[993 1 928 973]);
plotToEx(data3, time3,'r',1.0); hold on;

%% message from local_planner to gcs
n_msg_MPC_sp_HCE = size(msg_MPC_sp_HCE,1);
data4 = zeros(18,n_msg_MPC_sp_HCE);
time4 = zeros(1, n_msg_MPC_sp_HCE);

for i=1: n_msg_MPC_sp_HCE
    data4(1,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(1,1);
    data4(2,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(2,1);
    data4(3,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(3,1);
    data4(4,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(4,1);
    data4(5,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(5,1);
    data4(6,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(6,1);
    data4(7,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(7,1);
    data4(8,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).State(8,1);
    
    data4(9,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).Input(1,1);
    data4(10,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).Input(2,1);
    data4(11,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).Input(3,1);
    data4(12,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,1).Input(4,1);
    
    data4(13,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(1,1);
    data4(14,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(2,1);
    data4(15,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(3,1);
    data4(16,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(4,1);
    data4(17,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(5,1);
    data4(18,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(6,1);
    data4(19,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(7,1);
    data4(20,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).State(8,1);
    
    data4(21,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).Input(1,1);
    data4(22,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).Input(2,1);
    data4(23,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).Input(3,1);
    data4(24,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,2).Input(4,1);
    
    data4(25,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(1,1);
    data4(26,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(2,1);
    data4(27,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(3,1);
    data4(28,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(4,1);
    data4(29,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(5,1);
    data4(30,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(6,1);
    data4(31,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(7,1);
    data4(32,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).State(8,1);
    
    data4(33,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).Input(1,1);
    data4(34,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).Input(2,1);
    data4(35,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).Input(3,1);
    data4(36,i) = msg_MPC_sp_HCE{i, 1}.Predictions(1,3).Input(4,1);
    
    
    time4(1,i) = double(msg_MPC_sp_HCE{i, 1}.Header.Stamp.Sec) ...
        + 1e-9*double(msg_MPC_sp_HCE{i, 1}.Header.Stamp.Nsec) ...
        - min_time;
        %- double(msg_MPC_sp_HCE{1, 1}.Header.Stamp.Sec) ...
        %- 1e-9*double(msg_MPC_sp_HCE{1, 1}.Header.Stamp.Nsec) ;
end

figure(f2);
plotToEx(data4, time4,'b',0.2);
legend('from gcs to ex', 'from local to gcsc');
figure(f3); hold on;
plotToEx(data4, time4,'b',0.5);
legend('from gcs to ex', 'from local to gcsc');
figure(f4); hold on;
plotToEx(data4, time4,'b',1.0); 
legend('from gcs to ex', 'from local to gcsc');