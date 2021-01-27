clc; clear all; close all;

folder_name = 'D:\hce_datasets\20210119_exp_planning\';
data_name = 'concrete2';
% 2021-01-19_14_38_09_concrete1
% 2021-01-19_14_43_20_concrete2
% 2021-01-19_14_54_56_up1
f = fopen([folder_name, data_name, '\to_excavator.txt']);
a = fgets(f);

time=[];
byte=[];

iter=1;
while(1)
    line = fgets(f);
    if(line==-1)
        break;
    else
        temp = strsplit(line, ', ');
        time = [time, str2num(temp{1,1})];
        for i=2:73
            byte(i-1,iter) = str2num(temp{1,i});
        end
    end
    iter=iter+1;
end

calRealToEx;

plotToEx;

%% time_check
time2 = zeros(1, length(time));
for i=1:size(time2,2)
   time2(1,i) = time(1,i) - time(1,1); 
end

diff_time = zeros(1, length(time)-1);
for i=1:size(time2,2)-1
   diff_time(1,i) = time2(1,i+1) - time2(1,i); 
end

m = mean(diff_time);
s = std(diff_time);

figure();
plot(diff_time,'r');
m_ = ['Mean = ',num2str(m),' [s]'];
s_plus = ['Mean + 3sigma = ',num2str(m+3*s),' [s]'];
s_minus = ['Mean - 3sigma = ',num2str(m-3*s),' [s]'];
yline(m,'-',m_,'LineWidth',3,'Color','b');
yline(m+3*s,'--',s_plus,'LineWidth',3,'Color','k');
yline(m-3*s,'--',s_minus,'LineWidth',3,'Color','k');
title('CAN transmition time check');
xlabel('# of data'); ylabel('time interval[s]');
 grid on;
