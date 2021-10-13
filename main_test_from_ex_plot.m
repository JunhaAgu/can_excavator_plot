clc; clear all;
close all;
dir = '/home/larr/Downloads/matlab/code/can_excavator_plot';
dataname = '/datafolder';

D2R = pi/180;
R2D = 1/D2R;

img_data    = struct();
img_data.n_cols = 1032; 
img_data.n_rows = 772;

sensor_data = struct();

data = importdata([dir,dataname,'/from_excavator.txt']);

fileID = fopen([dir,dataname,'/association.txt'],'r');
tline  = fgets(fileID);
n_data = 0;
img_data.time = [];
img_data.image_name.boom.upper  = cell(1,1);
img_data.image_name.boom.lower  = cell(1,1);
img_data.image_name.cabin.upper = cell(1,1);
img_data.image_name.cabin.lower = cell(1,1);

while(1)
   tline = fgets(fileID);
   
   if(tline == -1)
      img_data.n_data = n_data;
      break;
   end
      n_data = n_data + 1;

   chars = strsplit(tline,' ');
   img_data.time = [img_data.time;str2num(chars{1,1})];
   img_data.image_name.boom.upper{1,n_data}  = [dir,dataname,chars{1,2}];
   img_data.image_name.boom.lower{1,n_data}  = [dir,dataname,chars{1,3}];
   img_data.image_name.cabin.upper{1,n_data} = [dir,dataname,chars{1,4}];
   img_data.image_name.cabin.lower{1,n_data} = [dir,dataname,chars{1,5}];
   
end


f = fopen([dir,dataname,'/from_excavator.txt']);
fgets(f);

time=[];
data=[];

iter=1;
while(1)
    line = fgets(f);
    if(line==-1)
        break;
    else
        temp = strsplit(line, ', ');
        time = [time, str2num(temp{1,1})];
        for i=2:25
            data(i-1,iter) = str2num(temp{1,i});
        end
    end
    iter=iter+1;
end

%% integer plot
% plotPressureFromEx;
% plotAngleFromEx;
% plotRateFromEx;

%% time_check
time2 = zeros(1, length(time));
for i=1:size(time2,2)
    time2(1,i) = time(1,i) - time(1,1);
end

diff_time = zeros(1, length(time)-1);
for i=1:size(time2,2)-1
    diff_time(1,i) = time2(1,i+1) - time2(1,i);
end


%% plot ex to gcs
for i=9:18
   data(i,:) = 180/pi*data(i,:);
end

f1 =  figure('Position',[66 1 928 973]);  %figure('Position',[993 1 928 973]);
plotFromEx(data, time2,'r'); hold on;


m = mean(diff_time(1,1:end));
s = std(diff_time(1,1:end));
figure(); plot(diff_time,'c');
m_ = ['Mean = ',num2str(m),' [s]'];
s_ = ['std = ',num2str(s),' [s]'];
yline(m,'-',m_,'LineWidth',1,'Color','b');
yline(m,'-',s_,'LineWidth',1,'Color','b','LabelVerticalAlignment','bottom');
xlabel('# of data'); ylabel('dt [s]');
title('exc -> gcs');
grid on;
xlim([1, length(diff_time(1,1:end))]);
% ylim([0.09 0.11]);

figure();
plot(time,time,'.k'); hold on;
plot(img_data.time,img_data.time,'rs');
legend('sensor time','image time');

