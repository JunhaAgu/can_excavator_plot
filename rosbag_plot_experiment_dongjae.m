
clc;
clear;
close all;

%% NOTE
%{
1. 01/19 experiemnt data
    - Swing pressure not measured
    - Boom pressure wrongly published from communication node (Junha)
        -> ignore Swing, Boom direction disturbance estimator
    - MPC blows up at data 3
        -> MPC setpoint tracking controller was not implemented
    - Swing angle reference = current value
        -> not desirable (shift), should be changed to a constant value
        -> result: non-converging behavior in Swing
    - problem in "to_local_planning" pressure data
        -> compared to the 0921 exp. data, 1/10 scale
            - 200921: pressure 10^2 [bar] scale (Arm)
            - 210119: pressure 10^1 [bar] scale (Arm)
        -> result 1: impossible to validate disturbance estimation module
        -> result 2: impossible to validate MPC + disturbance estimation
%}
%% matlabPlot for experiments

% cd '~/ros_bagfiles/HCE_excavator/simul/0119/'
cd '~/ros_bagfiles/HCE_excavator/exp/0119/'
% no_plot = true;

%% rosbag
cd '~/ros_bagfiles/HCE_excavator'

bag_data_num = 3; % 1,2,3
switch bag_data_num
    case 1
        msg_save_name = '0119_exp_data1.mat';
        bag = rosbag('./exp/0119/exp_communication_2021-01-19-14-38-10.bag');
    case 2
        msg_save_name = '0119_exp_data2.mat';
        bag = rosbag('./exp/0119/exp_communication_2021-01-19-14-43-22.bag');
    case 3
        msg_save_name = '0119_exp_data3.mat';
        bag = rosbag('./exp/0119/exp_communication_2021-01-19-14-55-00.bag');
end

input_type = "Force"; % L, Ldot, Force
fig_save_flag = false;
FL_flag = true;
cd '~/excavator_ws/src/excavator_larr/local_planning/matlab_data'
load('param_dyn.mat');              % param_dyn

%% check available topics
bag.AvailableTopics

%% topics to be used

bag_canpackets_fromexc = select(bag,'Topic','/canpackets/FromExcavator');
bag_canpackets_toexc = select(bag,'Topic','/canpackets/ToExcavator');

bag_MPC_sp = select(bag,'Topic','/excavator_MPC_FL/excavator_MPC_sp');
bag_ilqr = select(bag,'Topic','/excavator_MPC_FL/ilqr/predictions');
bag_ref_x = select(bag,'Topic','/excavator_MPC_FL/excavator_reference_x');
bag_cpt_time = select(bag,'Topic','/excavator_MPC_FL/excavator_compute_time');

bag_state = select(bag,'Topic','/to_local_planning');
bag_MPC_sp_HCE = select(bag,'Topic','/local_planning_setpoints'); % 0.2, 0.5, 1.0

bag_dist_estm = select(bag,'Topic','/estimated_uncertainty');
bag_dist_estm_L = select(bag,'Topic','/estimated_uncertaintyL');

%% readMessages

msg_canpackets_fromexc = readMessages(bag_canpackets_fromexc,'DataFormat','struct');
msg_canpackets_toexc = readMessages(bag_canpackets_toexc,'DataFormat','struct');

msg_MPC_sp = readMessages(bag_MPC_sp,'DataFormat','struct');
msg_ilqr = readMessages(bag_ilqr,'DataFormat','struct');
msg_ref_x = readMessages(bag_ref_x,'DataFormat','struct');
msg_cpt_time = readMessages(bag_cpt_time,'DataFormat','struct');

msg_state = readMessages(bag_state,'DataFormat','struct');
msg_MPC_sp_HCE = readMessages(bag_MPC_sp_HCE,'DataFormat','struct');

msg_dist_estm = readMessages(bag_dist_estm,'DataFormat','struct');
msg_dist_estm_L = readMessages(bag_dist_estm_L,'DataFormat','struct');

% save(msg_save_name);

%% constraint check
% 1. force/torque limit - unit: [Nm], [N]
T_U_min = -126309;
T_U_max = 126309;
F_B_min = -855617;
F_B_max = 1746885;
F_A_min = -463459;
F_A_max = 1002676;
F_K_min = -384181;
F_K_max = 812168;

u_min = [T_U_min;F_B_min;F_A_min;F_K_min];
u_max = [T_U_max;F_B_max;F_A_max;F_K_max];

% 2. power limit - unit: [W = Nm/s]
p_min = -65000;
p_max = 65000;

% 3. pump flow rate limit
    % pump flow rate - unit: [mm^3/s / 1000]
fBar_min_1 = -266*1e3/60;
fBar_min_2 = -266*1e3/60;
fBar_max_1 = 266*1e3/60;
fBar_max_2 = 266*1e3/60;

    % cross-section area - unit: [cc/rad], [mm^2]
ABar_U = 169.4*133.85454/(2*3.1415926535);
ABar_B_H = 15394*2;    % head - extension
ABar_B_R = 7854*2;     % rod  - retraction
ABar_A_H = 17671;      % head - extension
ABar_A_R = 9503;       % rod  - retraction
ABar_K_H = 14314;      % head - extension
ABar_K_R = 7543;       % rod  - retraction

% 4. excavator cylinder length - unit: [m]
L_B_min = 2.080;
L_B_max = 3.545;
L_A_min = 2.378;
L_A_max = 4.143;
L_K_min = 1.795;
L_K_max = 2.980;

L_min = [L_B_min;L_A_min;L_K_min];
L_max = [L_B_max;L_A_max;L_K_max];

%% desired points

MPC_sp_size = size(msg_MPC_sp,1);
ilqr_size = size(msg_ilqr,1);
ref_x_size = size(msg_ref_x,1);
cpt_time_size = size(msg_cpt_time,1);

state_size = size(msg_state,1);
MPC_sp_HCE_size = size(msg_MPC_sp_HCE,1);

dist_estm_size = size(msg_dist_estm,1);
dist_estm_L_size = size(msg_dist_estm_L,1);

idx = 1;
if(MPC_sp_size > 0)
    time0_can(idx) = getTime(msg_MPC_sp{1});
    idx = idx+1;
end
if(ilqr_size > 0)
time0_can(idx) = getTime(msg_ilqr{1});
    idx = idx+1;
end
if(ref_x_size > 0)
time0_can(idx) = getTime(msg_ref_x{1});
    idx = idx+1;
end
if(cpt_time_size > 0)
time0_can(idx) = getTime(msg_cpt_time{1});
    idx = idx+1;
end
% if(state_size > 0)
% time0_can(idx) = getTime(msg_state{1});
%     idx = idx+1;
% end
if(MPC_sp_HCE_size > 0)
time0_can(idx) = getTime(msg_MPC_sp_HCE{1});
    idx = idx+1;
end
if(dist_estm_size > 0)
time0_can(idx) = getTime(msg_dist_estm{1});
    idx = idx+1;
end
if(dist_estm_L_size > 0)
time0_can(idx) = getTime(msg_dist_estm_L{1});
    idx = idx+1;
end

time_0 = min(min(time0_can));
% MPC setpoint
for i=1:MPC_sp_size
    MPC_sp_time(1,i) = getTime(msg_MPC_sp{i}) - time_0;
    MPC_sp_L(1,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Configuration.PsiU);
    MPC_sp_L(2,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Configuration.LB);
    MPC_sp_L(3,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Configuration.LA);
    MPC_sp_L(4,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Configuration.LK);
    MPC_sp_Ldot(1,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Velocity.PsiUDot);
    MPC_sp_Ldot(2,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Velocity.LBDot);
    MPC_sp_Ldot(3,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Velocity.LADot);
    MPC_sp_Ldot(4,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Velocity.LKDot);
    MPC_sp_F(1,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Force.TauU);
    MPC_sp_F(2,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Force.FB);
    MPC_sp_F(3,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Force.FA);
    MPC_sp_F(4,i) = IfcomplexThenNaN(msg_MPC_sp{i}.State.Force.FK);
end
% ilqr prediction & initial condition
if(ilqr_size > 0)
    ilqr_dt = msg_ilqr{1}.Dt;
    ilqr_N = length(msg_ilqr{1}.Predictions);   
end
for i=1:ilqr_size
    ilqr_time(1,i) = getTime(msg_ilqr{i}) - time_0;
    ilqr_x0(:,i) = IfcomplexThenNaN(msg_ilqr{i}.Initial);
    ilqr_xN(:,1,i) = ilqr_x0(:,i);
    for j=1:ilqr_N
        ilqr_xN(:,j+1,i) = IfcomplexThenNaN(msg_ilqr{i}.Predictions(j).State);
        ilqr_uN(:,j,i) = IfcomplexThenNaN(msg_ilqr{i}.Predictions(j).Input);
    end
end

% reference x
if(ref_x_size > 0)
    ref_x_dt = msg_ref_x{1}.Dt;
    ref_x_N = length(msg_ref_x{1}.StateSeq); % ilqr_N + 1
end
for i=1:ref_x_size
    ref_x_time(1,i) = getTime(msg_ref_x{i}) - time_0;
    for j=1:ref_x_N
        ref_x(:,j,i) = msg_ref_x{i}.StateSeq(j).State;
    end
end

% computation time
for i=1:cpt_time_size
    cpt_time_time(1,i) = getTime(msg_cpt_time{i}) - time_0;
    cpt_time(1,i) = msg_cpt_time{i}.Point.X;
end

% state
for i=1:state_size
%     state_time(1,i) = getTime(msg_state{i}) - time_0;
    state_time(1,i) = getTime(msg_state{i});
    % bar --> N/mm^2 (*0.1)
    input_F(1,i) = (ABar_U*(msg_state{i}.Measure.Pressure.SwingL - ...
                           msg_state{i}.Measure.Pressure.SwingR))/10;
    input_F(2,i) = (ABar_B_H*msg_state{i}.Measure.Pressure.BCylLC - ...
                   ABar_B_R*msg_state{i}.Measure.Pressure.BCylSC)/10;
    input_F(3,i) = (ABar_A_H*msg_state{i}.Measure.Pressure.ACylLC - ...
                   ABar_A_R*msg_state{i}.Measure.Pressure.ACylSC)/10;
    input_F(4,i) = (ABar_K_H*msg_state{i}.Measure.Pressure.KCylLC - ...
                   ABar_K_R*msg_state{i}.Measure.Pressure.KCylSC)/10;
    
    phi_L(1,i) = msg_state{i}.Measure.Angle.PhiBody;
    theta_L(1,i) = msg_state{i}.Measure.Angle.ThetaBody;
    
    state_th(1,i) = msg_state{i}.Measure.Angle.PsiU;
    state_th(2,i) = msg_state{i}.Measure.Angle.ThetaB;
    state_th(3,i) = msg_state{i}.Measure.Angle.ThetaA;
    state_th(4,i) = msg_state{i}.Measure.Angle.ThetaK;
    state_th(5,i) = msg_state{i}.Measure.Velocity.PsiUDot;
    state_th(6,i) = msg_state{i}.Measure.Velocity.ThetaBDot;
    state_th(7,i) = msg_state{i}.Measure.Velocity.ThetaADot;
    state_th(8,i) = msg_state{i}.Measure.Velocity.ThetaKDot;
    
    state_L(1,i) = state_th(1,i); % psi_U
    state_L(5,i) = state_th(5,i); % psi_Udot
    [state_L(2:4,i), Jacobian] = Theta2L_new(state_th(2:4,i),param_dyn);
    state_L(6:8,i) = Jacobian*state_th(6:8,i);
end
state_time = state_time - min(state_time); % temporary sol.

% MPC_sp_HCE
for i=1:MPC_sp_HCE_size
    MPC_sp_HCE_time(1,i) = getTime(msg_MPC_sp_HCE{i}) - time_0;
    for j=1:3
        MPC_sp_HCE_x3(:,j,i) = IfcomplexThenNaN(msg_MPC_sp_HCE{i}.Predictions(j).State);
        MPC_sp_HCE_u3(:,j,i) = IfcomplexThenNaN(msg_MPC_sp_HCE{i}.Predictions(j).Input);
        
    end
end

% estimated disturbance
for i=1:dist_estm_size
    dist_estm_time(1,i) = getTime(msg_dist_estm{i}) - time_0;
    dist_estm(1,i) = msg_dist_estm{i}.PsiU;
    dist_estm(2,i) = msg_dist_estm{i}.ThetaB;
    dist_estm(3,i) = msg_dist_estm{i}.ThetaA;
    dist_estm(4,i) = msg_dist_estm{i}.ThetaK;
end
for i=1:dist_estm_L_size
    dist_estm_L_time(1,i) = getTime(msg_dist_estm_L{i}) - time_0;
    dist_estm_L(1,i) = msg_dist_estm_L{i}.PsiU;
    dist_estm_L(2,i) = msg_dist_estm_L{i}.LB;
    dist_estm_L(3,i) = msg_dist_estm_L{i}.LA;
    dist_estm_L(4,i) = msg_dist_estm_L{i}.LK;
end

% XXX - add estimated disturbance (computed in MPC_sp_HCE)
if(FL_flag && ilqr_size > 0)
     for i=1:1 % size(ilqr_uN,2) % = 50
         for j=1:size(ilqr_uN,3)
            DeltaHat(:,1) = IfcomplexThenNaN(MPC_sp_HCE_u3(:,1,i) - input_woDist_FL(ilqr_xN(:,11,j),ilqr_uN(:,10,j),param_dyn));
            ilqr_uN(:,i,j) = IfcomplexThenNaN(input_woDist_FL(ilqr_xN(:,i+1,j),ilqr_uN(:,i,j),param_dyn) + DeltaHat);
         end
     end
end

%% disturbance
figure('Name','estimated disturbance')
for i=1:4
subplot(2,2,i)
hold on; grid on;
    pl1 = plot(dist_estm_L_time,dist_estm_L(i,:),'-k','linewidth',2);
%     pl2 = plot(state_time,input_F(i,:),'-m','linewidth',2);
%     plot(ext_force_time,ext_force(i,:),'-r','linewidth',1);
    switch i
        case 1
            legend([pl1],{'$\tilde{\Delta}_{U}$'},'interpreter','latex');
%             legend([pl1,pl2],{'$\tilde{\Delta}_{U}$','$T_U$'},'interpreter','latex');
%             legend('$\tilde{\Delta}_{U}$','fontsize',12,'interpreter','latex');
%             legend('$\tilde{\Delta}_{U}$','${\Delta}_{U}$','fontsize',12,'interpreter','latex');
            ylabel('torque [Nm]');
        case 2
            legend([pl1],{'$\tilde{\Delta}_{B}$'},'interpreter','latex');
%             legend([pl1,pl2],{'$\tilde{\Delta}_{B}$','$F_B$'},'interpreter','latex');
%             legend('$\tilde{\Delta}_{B}$','fontsize',12,'interpreter','latex');
%             legend('$\tilde{\Delta}_{B}$','${\Delta}_{B}$','fontsize',12,'interpreter','latex');
            ylabel('force [N]');
        case 3
            legend([pl1],{'$\tilde{\Delta}_{A}$'},'interpreter','latex');
%             legend([pl1,pl2],{'$\tilde{\Delta}_{A}$','$F_A$'},'interpreter','latex');
%             legend('$\tilde{\Delta}_{A}$','fontsize',12,'interpreter','latex');
%             legend('$\tilde{\Delta}_{A}$','${\Delta}_{A}$','fontsize',12,'interpreter','latex');
            ylabel('force [N]');
        case 4
            legend([pl1],{'$\tilde{\Delta}_{K}$'},'interpreter','latex');
%             legend([pl1,pl2],{'$\tilde{\Delta}_{K}$','$F_K$'},'interpreter','latex');
%             legend('$\tilde{\Delta}_{K}$','fontsize',12,'interpreter','latex');
%             legend('$\tilde{\Delta}_{K}$','${\Delta}_{K}$','fontsize',12,'interpreter','latex');
            ylabel('force [N]');
    end
    xlabel('time [s]');
end

if(fig_save_flag)
    print(join("dist_estm","_"),'-dpng','-r1000')
end

%% ROS-Time based plot - reference (MPC) vs. state/input (simulator)
%% computation time plot
figure('Name','computation time')
plot(cpt_time_time,cpt_time,'k','linewidth',2); grid on; hold on;
plot(cpt_time_time,0.1*ones(length(cpt_time_time),1),'r','linewidth',2);
xlabel('time [s]');
ylabel('computation time [s]');

if(fig_save_flag)
    print(join("compute_time","_"),'-dpng','-r1000')
end
%% figure plot
figure('Name','position')
for i=1:4
subplot(2,2,i)
hold on; grid on;
if i > 1
    plot(ilqr_time,L_min(i-1)*ones(1,ilqr_size),'r','linewidth',2);
    plot(ilqr_time,L_max(i-1)*ones(1,ilqr_size),'r','linewidth',2);
end
% pl1 = plot(ilqr_time,reshape(ilqr_xN(i,2,:),1,[]),'m','linewidth',2);
pl2 = plot(ilqr_time,ilqr_x0(i,:),'k','linewidth',2); % real value (initial value subscribed by MPC)
for j=1:3
    pl_(j) = plot(MPC_sp_HCE_time,reshape(MPC_sp_HCE_x3(i,j,:),1,[]),'--','linewidth',2);
end
% pl2 = plot(state_time,state_L(i,:),'b','linewidth',2);
switch i
    case 1
%         legend([pl1],{'\psi_U MPC'});
        title('\psi_U');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'\psi_U MPC','\psi_U real'});
        ylabel('angle [rad]');
    case 2
%         legend([pl1],{'L_B MPC'});
        title('L_B');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'L_B MPC','L_B real'});
        ylabel('cyl. length [m]');
    case 3
%         legend([pl1],{'L_A MPC'});
        title('L_A');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'L_A MPC','L_A real'});
        ylabel('cyl. length [m]');
    case 4
%         legend([pl1],{'L_K MPC'});
        title('L_K');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'L_K MPC','L_K real'});
        ylabel('cyl. length [m]');
end
xlabel('time [s]');
end

if(fig_save_flag)
    print(join("position","_"),'-dpng','-r1000')
end

figure('Name','velocity')
for i=1:4
subplot(2,2,i)
hold on; grid on;
% for j=1:ilqr_size
%     ilqr_time_MPC = ilqr_time(j):ilqr_dt:ilqr_time(j)+ilqr_dt*(ilqr_N-1);
%     plot(ilqr_time_MPC,reshape(ilqr_xN(i+4,1:end-1,j),1,[]),'k--','linewidth',0.5);
% end
% pl1 = plot(ilqr_time,reshape(ilqr_xN(i+4,2,:),1,[]),'m','linewidth',2); % first estimated state
pl2 = plot(ilqr_time,ilqr_x0(i+4,:),'k','linewidth',2); % real value (initial value subscribed by MPC)
for j=1:3
    pl_(j) = plot(MPC_sp_HCE_time,reshape(MPC_sp_HCE_x3(i+4,j,:),1,[]),'--','linewidth',2);
end
% pl2 = plot(state_L_time,state_L(i+4,:),'b','linewidth',2);
switch i
    case 1
%         legend([pl1],{'\psi_U dot MPC'});
        title('\psi_U dot');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'\psi_U dot MPC','\psi_U dot real'});
        ylabel('ang. vel. [rad/s]');
    case 2
%         legend([pl1],{'L_B dot MPC'});
        title('L_B dot');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'L_B dot MPC','L_B dot real'});
        ylabel('lin. vel. [m/s]');
    case 3
%         legend([pl1],{'L_A dot MPC'});
        title('L_A dot');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'L_A dot MPC','L_A dot real'});
        ylabel('lin. vel. [m/s]');
    case 4
%         legend([pl1],{'L_K dot MPC'});
        title('L_K dot');
        legend([pl2,pl_(1),pl_(2),pl_(3)],{'measured','MPC 0.2','MPC 0.5','MPC 1.0'});
%         legend([pl1,pl2],{'L_K dot MPC','L_K dot real'});
        ylabel('lin. vel. [m/s]');
end
xlabel('time [s]');
end

if(fig_save_flag)
    print(join("velocity","_"),'-dpng','-r1000')
end
%%
figure('Name','1. MPC input check')
% sgtitle('1. MPC input constraint')
for i=1:4
subplot(2,2,i)
hold on; grid on;
plot(ilqr_time,u_min(i)*ones(1,ilqr_size),'r','linewidth',2);
plot(ilqr_time,u_max(i)*ones(1,ilqr_size),'r','linewidth',2);
% for j=1:ilqr_size
%     ilqr_time_MPC = ilqr_time(j):ilqr_dt:ilqr_time(j)+ilqr_dt*(ilqr_N-1);
%     plot(ilqr_time_MPC,reshape(ilqr_uN(i,:,j),1,[]),'k--','linewidth',0.5);
% end
% pl2 = plot(state_time,input_F(i,:),'b','linewidth',2);
% pl1 = plot(ilqr_time,reshape(ilqr_uN(i,1,:),1,[]),'m','linewidth',2);
for j=1:3
    pl_(j) = plot(MPC_sp_HCE_time,reshape(MPC_sp_HCE_u3(i,j,:),1,[]),'--','linewidth',2);
end
switch i
    case 1
%         legend([pl1],{'T_U MPC'});
        title('T_U');
%         legend([pl1,pl2],{'T_U MPC','T_U real'});
        ylabel('torque [Nm]');
    case 2
%         legend([pl1],{'F_B MPC'});
        title('F_B');
%         legend([pl1,pl2],{'F_B MPC','F_B real'});
        ylabel('force [N]');
    case 3
%         legend([pl1],{'F_A MPC'});
        title('F_A');
%         legend([pl1,pl2],{'F_A MPC','F_A real'});
        ylabel('force [N]');
    case 4
%         legend([pl1],{'F_K MPC'});
        title('F_K');
%         legend([pl1,pl2],{'F_K MPC','F_K real'});
        ylabel('force [N]');
end
legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
xlabel('time [s]');
end

if(fig_save_flag)
    print(join("const1","_"),'-dpng','-r1000')
end
%% new time scale - for MPC constraint check
ilqr_time_new = 0:ilqr_dt:ilqr_dt*(ilqr_size-1); 
MPC_sp_HCE_time_new = 0:ilqr_dt:ilqr_dt*(MPC_sp_HCE_size-1); 
    %% MPC - velocity
    figure('Name','MPC velocity')
    % sgtitle('3. MPC cyl. length constraint')
    for i=1:4
    subplot(2,2,i)
    hold on; grid on;
    t0_ref_x = 0;
    for j=1:ref_x_size
        ref_x_time_MPC = linspace(t0_ref_x,t0_ref_x+ref_x_dt*(ref_x_N-1),ref_x_N);
        t0_ref_x = t0_ref_x + ref_x_dt;
        plot(ref_x_time_MPC,reshape(ref_x(i+4,:,j),1,[]),'--','linewidth',1);
    end
%     pl1 = plot(ilqr_time_new,reshape(ilqr_xN(i+4,2,:),1,[]),'m','linewidth',2);
    for j=1:3
        pl_(j) = plot(MPC_sp_HCE_time_new,reshape(MPC_sp_HCE_x3(i+4,j,:),1,[]),'-','linewidth',2);
    end
    switch i
        case 1
%             legend(pl1,'\psi_U dot MPC');
            title('\psi_U dot');
            ylabel('ang. vel. [rad/s]');
        case 2
%             legend(pl1,'L_B dot MPC');
            title('L_B dot');
            ylabel('lin. vel. [m/s]');
        case 3
%             legend(pl1,'L_A dot MPC');
            title('L_A dot');
            ylabel('lin. vel. [m/s]');
        case 4
%             legend(pl1,'L_K dot MPC');
            title('L_K dot');
            ylabel('lin. vel. [m/s]');
    end
    legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
    xlabel('time [s]');
    end
    
    %% MPC - length
    figure('Name','3. MPC length check')
    % sgtitle('3. MPC cyl. length constraint')
    for i=1:4
    subplot(2,2,i)
    hold on; grid on;
    if i > 1
        plot(ilqr_time_new,L_min(i-1)*ones(1,ilqr_size),'r','linewidth',2);
        plot(ilqr_time_new,L_max(i-1)*ones(1,ilqr_size),'r','linewidth',2);
    end
%     for j=1:ilqr_size
%         ilqr_time_MPC = ilqr_time_new(j):ilqr_dt:ilqr_time_new(j)+ilqr_dt*(ilqr_N-1);
%         plot(ilqr_time_MPC,reshape(ilqr_xN(i,1:end-1,j),1,[]),'k--','linewidth',0.5);
%     end
    t0_ref_x = 0;
    for j=1:ref_x_size
        ref_x_time_MPC = linspace(t0_ref_x,t0_ref_x+ref_x_dt*(ref_x_N-1),ref_x_N);
        t0_ref_x = t0_ref_x + ref_x_dt;
        plot(ref_x_time_MPC,reshape(ref_x(i,:,j),1,[]),'--','linewidth',1);
    end
%     pl1 = plot(ilqr_time_new,reshape(ilqr_xN(i,2,:),1,[]),'m','linewidth',2);
    for j=1:3
        pl_(j) = plot(MPC_sp_HCE_time_new,reshape(MPC_sp_HCE_x3(i,j,:),1,[]),'-','linewidth',2);
    end
    switch i
        case 1
%             legend(pl1,'\psi_U MPC');
            title('\psi_U');
            ylabel('angle [rad]');
        case 2
%             legend(pl1,'L_B MPC');
            title('L_B');
            ylabel('length [m]');
        case 3
%             legend(pl1,'L_A MPC');
            title('L_A');
            ylabel('length [m]');
        case 4
%             legend(pl1,'L_K MPC');
            title('L_K');
            ylabel('length [m]');
    end
    legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
    xlabel('time [s]');
    end

    if(fig_save_flag)
        print(join("const3","_"),'-dpng','-r1000')
    end
    %% MPC - input
    figure('Name','1. MPC input check')
    % sgtitle('1. MPC input constraint')
    for i=1:4
    subplot(2,2,i)
    hold on; grid on;
    plot(ilqr_time_new,u_min(i)*ones(1,ilqr_size),'r','linewidth',2);
    plot(ilqr_time_new,u_max(i)*ones(1,ilqr_size),'r','linewidth',2);
    % for j=1:ilqr_size
    %     ilqr_time_MPC = ilqr_time_new(j):ilqr_dt:ilqr_time_new(j)+ilqr_dt*(ilqr_N-1);
    %     plot(ilqr_time_MPC,reshape(ilqr_uN(i,:,j),1,[]),'k--','linewidth',0.5);
    % end
%     pl1 = plot(ilqr_time_new,reshape(ilqr_uN(i,1,:),1,[]),'m','linewidth',2);
    for j=1:3
        pl_(j) = plot(MPC_sp_HCE_time_new,reshape(MPC_sp_HCE_u3(i,j,:),1,[]),'--','linewidth',2);
    end
    switch i
        case 1
%             legend(pl1,'T_U MPC');
            title('T_U');
            ylabel('torque [Nm]');
        case 2
%             legend(pl1,'F_B MPC');
            title('F_B');
            ylabel('force [N]');
        case 3
%             legend(pl1,'F_A MPC');
            title('F_A');
            ylabel('force [N]');
        case 4
%             legend(pl1,'F_K MPC');
            title('F_K');
            ylabel('force [N]');
    end
    legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
    xlabel('time [s]');
    end

    if(fig_save_flag)
        print(join("const1","_"),'-dpng','-r1000')
    end
    %% MPC - power
    figure('Name','2. MPC power check')
    % sgtitle('2. MPC power constraint')
    hold on; grid on;
    plot(ilqr_time_new,p_max*ones(1,ilqr_size),'r','linewidth',2);
    % for j=1:ilqr_size
    %     ilqr_time_MPC = ilqr_time_new(j):ilqr_dt:ilqr_time_new(j)+ilqr_dt*(ilqr_N-1);
    %     plot(ilqr_time_MPC,sum(ilqr_uN(:,:,j).*reshape(ilqr_xN(5:8,1:end-1,j),4,[]),1),'k--','linewidth',1);
    % end
%     pl = plot(ilqr_time_new,sum(reshape(ilqr_uN(:,1,:),4,[]).*reshape(ilqr_xN(5:8,2,:),4,[]),1),'b','linewidth',2);
    for j=1:3
        pl_(j) = plot(MPC_sp_HCE_time_new,sum(reshape(MPC_sp_HCE_u3(:,j,:),4,[]).*reshape(MPC_sp_HCE_x3(5:8,j,:),4,[]),1),'--','linewidth',2);
    end
%     legend(pl,'power');
    legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
    title('power');
    ylabel('power [Nm/s]');
    xlabel('time [s]');

    if(fig_save_flag)
        print(join("const2","_"),'-dpng','-r1000')
    end
    
    %% MPC - flow rate
    figure('Name','4. MPC pump flow rate check')
    % sgtitle('4. MPC flow rate constraint')

    pump1_flowRate = zeros(MPC_sp_HCE_size,ilqr_N);
    pump2_flowRate = zeros(MPC_sp_HCE_size,ilqr_N);
    % pump1 = Swing + Boom up 1/2 + Boom down + Arm in/out 1/2
    % pump2 =         Boom up 1/2 +             Arm in/out 1/2 + Bucket inj/out
    for i=1:MPC_sp_HCE_size
        FlowRate_max_MPC_pump1(i,1) = fBar_max_1;
        FlowRate_max_MPC_pump2(i,1) = fBar_max_2;
        for j=1:3
            % UpperAssy
            pump1_flowRate(i,j) = pump1_flowRate(i,j) + abs(MPC_sp_HCE_x3(5,j,i))*ABar_U;
            % Boom
            if(MPC_sp_HCE_x3(6,j,i) > 0)
                pump1_flowRate(i,j) = pump1_flowRate(i,j) + abs(MPC_sp_HCE_x3(6,j,i))*ABar_B_H/2;
                pump2_flowRate(i,j) = pump2_flowRate(i,j) + abs(MPC_sp_HCE_x3(6,j,i))*ABar_B_H/2;
            else
                pump1_flowRate(i,j) = pump1_flowRate(i,j) + abs(MPC_sp_HCE_x3(6,j,i))*ABar_B_R;
            end
            % Arm
            if(MPC_sp_HCE_x3(7,j,i) > 0)
                pump1_flowRate(i,j) = pump1_flowRate(i,j) + abs(MPC_sp_HCE_x3(7,j,i))*ABar_A_H/2;
                pump2_flowRate(i,j) = pump2_flowRate(i,j) + abs(MPC_sp_HCE_x3(7,j,i))*ABar_A_H/2;
            else
                pump1_flowRate(i,j) = pump1_flowRate(i,j) + abs(MPC_sp_HCE_x3(7,j,i))*ABar_A_R/2;
                pump2_flowRate(i,j) = pump2_flowRate(i,j) + abs(MPC_sp_HCE_x3(7,j,i))*ABar_A_R/2;
            end
            % Bucket
            if(MPC_sp_HCE_x3(8,j,i) > 0)
                pump2_flowRate(i,j) = pump2_flowRate(i,j) + abs(MPC_sp_HCE_x3(8,j,i))*ABar_K_H;
            else
                pump2_flowRate(i,j) = pump2_flowRate(i,j) + abs(MPC_sp_HCE_x3(8,j,i))*ABar_K_R;
            end    
        end
    end   
    
    hold on; grid on;
%     t0_ilqr = 0;
    subplot(2,1,1)
    hold on; grid on; 
%     plot(MPC_sp_HCE_time_new,FlowRate_max_MPC_pump1(:,1),'r','linewidth',2);
    plot(MPC_sp_HCE_time,FlowRate_max_MPC_pump1(:,1),'r','linewidth',2);
    for j=1:3
%         pl_(j) = plot(MPC_sp_HCE_time_new,pump1_flowRate(:,j),'--','linewidth',2);
        pl_(j) = plot(MPC_sp_HCE_time,pump1_flowRate(:,j),'--','linewidth',2);
    end
%     for j=1:ilqr_size
%         ilqr_time_MPC = linspace(t0_ilqr,t0_ilqr+ilqr_dt*(ilqr_N-1),ilqr_N);
%         t0_ilqr = t0_ilqr + ilqr_dt;
%         plot(ilqr_time_MPC,pump1_flowRate(j,:),'k--','linewidth',1);
%     end
%     legend(pl,'pump 1');
    title('pump 1');
    legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
%     t0_ilqr = 0;
    subplot(2,1,2)
    hold on; grid on;
%     plot(MPC_sp_HCE_time_new,FlowRate_max_MPC_pump2(:,1),'r','linewidth',2);
    plot(MPC_sp_HCE_time,FlowRate_max_MPC_pump2(:,1),'r','linewidth',2);
    for j=1:3
%         pl_(j) = plot(MPC_sp_HCE_time_new,pump2_flowRate(:,j),'--','linewidth',2);
        pl_(j) = plot(MPC_sp_HCE_time,pump2_flowRate(:,j),'--','linewidth',2);
    end
%     for j=1:ilqr_size
%         ilqr_time_MPC = linspace(t0_ilqr,t0_ilqr+ilqr_dt*(ilqr_N-1),ilqr_N);
%         t0_ilqr = t0_ilqr + ilqr_dt;
%         plot(ilqr_time_MPC,pump2_flowRate(j,:),'k--','linewidth',1);
%     end
%     legend(pl,'pump 2');
    title('pump 1');
    legend([pl_(1),pl_(2),pl_(3)],{'MPC 0.2','MPC 0.5','MPC 1.0'});
    ylabel('flow rate [m*mm^2/s]');
    xlabel('time [s]'); 

    if(fig_save_flag)
        print(join("const4","_"),'-dpng','-r1000')
    end

%% functions
function output = getTime(msg)
    output = (double(msg.Header.Stamp.Sec) + 1e-9*double(msg.Header.Stamp.Nsec));
end
function output = IfcomplexThenNaN(data)
    if(imag(data) ~= 0)
        warning('complex number detected');
        output = zeros(size(data))*nan;
    else
        output = data;
    end
end