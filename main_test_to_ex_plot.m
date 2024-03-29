clc; clear all;
% close all;

f = fopen('to_excavator.txt');
fgets(f);

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

% calRealToEx;

% plotToEx;

%% time_check
time2 = zeros(1, length(time));
for i=1:size(time2,2)
    time2(1,i) = time(1,i) - time(1,1);
end

diff_time = zeros(1, length(time)-1);
for i=1:size(time2,2)-1
    diff_time(1,i) = time2(1,i+1) - time2(1,i);
end

m = mean(diff_time(1,50:end));
s = std(diff_time(1,50:end));

m_hz = mean(1./diff_time(1,50:end));
s_hz = std(1./diff_time(1,50:end));

figure();
plot(diff_time(1,50:end),'c');
m_ = ['Mean = ',num2str(m),' [s]'];
% s_plus = ['Mean + 3sigma = ',num2str(m+3*s),' [s]'];
% s_minus = ['Mean - 3sigma = ',num2str(m-3*s),' [s]'];
s_ = ['std = ',num2str(s),' [s]'];
yline(m,'-',m_,'LineWidth',1,'Color','b');
yline(m,'-',s_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
% yline(m+3*s,'--',s_plus,'LineWidth',3,'Color','k');
% yline(m-3*s,'--',s_minus,'LineWidth',3,'Color','k');
title('CAN transmition time check dt [s]');
xlabel('# of data'); ylabel('time interval [s]');
grid on;
xlim([1, length(diff_time(1,50:end))]);
ylim([0.09 0.11]);

figure();
plot(1./diff_time(1,50:end),'g');
m_hz_ = ['Mean = ',num2str(m_hz),' [Hz]'];
% s_plus = ['Mean + 3sigma = ',num2str(m_hz+3*s_hz),' [Hz]'];
% s_minus = ['Mean - 3sigma = ',num2str(m_hz-3*s_hz),' [Hz]'];
s_hz_ = ['std = ',num2str(s_hz),' [hz]'];
yline(m_hz,'-',m_hz_,'LineWidth',1,'Color','b');
yline(m_hz,'-',s_hz_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
% yline(m_hz+3*s_hz,'--',s_plus,'LineWidth',3,'Color','k');
% yline(m_hz-3*s_hz,'--',s_minus,'LineWidth',3,'Color','k');
title('CAN transmition time check freq [Hz]');
xlabel('# of data'); ylabel('frequency [Hz]');
grid on;
xlim([1, length(diff_time(1,50:end))]);
ylim([9 11]);