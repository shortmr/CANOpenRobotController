function [pos_rmse, tor_rmse, trial_data] = m1_post_process(csv_file,varargin)
%%% Process .csv file from CORC/multi_robot_interaction
%
% inputs:
%%%% (string) csv_file: path to csv file containing data from M1 log
%%%% (bool)   flag: bool for suppressing individual trial plot
% outputs:
%%%% (mat)  pos_rmse: RMSE for position tracking performance
%%%% (mat)  tor_rmse: RMSE for interaction torque tracking performance
%%%% (cell) trial_data: continuous position/torque data for each trial
%
% example: [pos, tor, dat] = m1_post_process('example.csv',0) 
%
%% Initialize variables
s = 2; % time (seconds) to subtract from the start of each trial
ss = get(0, 'ScreenSize');
fig_pos = [0 0 ss(3)*0.7 ss(4)*0.5];

if length(varargin) < 1
    flag = 1; % default is enabled trial plot
else
    flag = varargin{1};
end

if exist(csv_file, 'file') == 2
%% load data and assign variables (if file exists)
[~,fn,~]= fileparts(csv_file);
data = readtable(csv_file);

time = data.time;
fs = 1./mean(diff(time)); % sampling frequency

act_pos = data.JointPositions_1;
des_pos = data.MM1_DesiredJointPositions_1;
act_tor = data.SensorTorques_1;
des_tor = data.MM1_DesiredInteractionTorques_1;
%% find start and end time (index) of each trial
events = struct('start',[],'end',[],'interaction',[]);
pos_switch = diff(des_pos); % derivative of desired position

% check for non-zero bias term
shift_idx = find(abs(pos_switch)>0, 1, 'first')+1+1;
bias = mode(des_pos(shift_idx:end)); % zero point
split_idx = shift_idx;

% look for first start index (bias to trajectory)
temp_switch = pos_switch(split_idx:end);
start_idx = find(abs(temp_switch)>0, 1, 'first')+1+(split_idx-1);  

ii = 0;
cut_flag = false; % flag to check if last trial was cut short
while ~isempty(start_idx) && ~cut_flag
    ii = ii + 1;
    
    % record start index
    events.start(ii).index = start_idx;
    events.start(ii).time = time(start_idx);
    
    % look for end index (trajectory to bias)
    temp_start_idx = start_idx;
    idx_search = true;
    while idx_search
        temp_pos = des_pos(temp_start_idx+2:end);
        temp_end_idx = find(temp_pos==bias, 1, 'first');
        
        % check if end index is missing
        if isempty(temp_end_idx)
            end_idx = length(pos_switch)-1;
            idx_search = false;
            cut_flag = true;
        end
        
        % make sure end index is followed by bias
        if temp_pos(temp_end_idx+1) == bias 
            end_idx = temp_end_idx + temp_start_idx;
            idx_search = false;
        else
            temp_start_idx = temp_end_idx + temp_start_idx;
        end
    end
    
    % record end index
    events.end(ii).index = end_idx;
    events.end(ii).time = time(end_idx);
    
    % check if spring interaction was enabled/disabled
    % (non zero desired interaction torque)
    mid_idx = ceil(mean([start_idx, end_idx]));
    events.interaction(ii,1) = any(abs(des_tor(mid_idx-100:mid_idx+100))>0);
    
    % update switch vector to check for next start index
    split_idx = end_idx+1;
    temp_switch = pos_switch(split_idx:end);
    start_idx = find(abs(temp_switch)>0, 1, 'first')+1+(split_idx-1); 
end

% remove last trial if cut short
if cut_flag
    disp('last trial cut short: removing');
    events.start(end) = [];
    events.end(end) = [];
    events.interaction(end) = [];
end
%% get trial and interaction torque errors
pos_rmse = NaN(length(events.start),1);
tor_rmse = NaN(length(events.start),1);
trial_data = cell(length(events.start),1);
spring = NaN(length(events.start),1);
for k = 1:length(events.start)
    % exclude first s seconds of each trial
    si = events.start(k).index+round(fs*s);
    ei = events.end(k).index;
    
    act_tor_seg = act_tor(si:ei);
    des_tor_seg = des_tor(si:ei);
    act_pos_seg = act_pos(si:ei);
    des_pos_seg = des_pos(si:ei);
    time_seg = time(si:ei);
    interaction_seg = events.interaction(k)*ones(size(time_seg));
    
    pos_error_seg = des_pos_seg-act_pos_seg;
    tor_error_seg = des_tor_seg+act_tor_seg; % flip sign
    
    % calculate RMSE for position and torque
    pos_error = sqrt(mean(pos_error_seg.^2));
    tor_error = sqrt(mean(tor_error_seg.^2));

    % plot trial data
    if flag
        time_seg = time_seg - time_seg(1);
        fig_t = figure('Position',fig_pos+[0 0.3*ss(4) 0 0]);
        set(0, 'currentfigure', fig_t);
        hold on;
        plot(time_seg, des_pos_seg,'r-',...
             time_seg, act_pos_seg,'b-','linewidth', 1.5);
        ylim([0.2, 1.4]);
        xlabel('Time (s)');
        ylabel('Angle (rad)');
        legend('Target angle', 'Ankle Angle');
        title([fn ' T', num2str(k)],'Interpreter','none');
    end
    trial_data{k} = table(time_seg, des_pos_seg, act_pos_seg, des_tor_seg, act_tor_seg, interaction_seg);
    pos_rmse(k) = pos_error;
    tor_rmse(k) = tor_error;
    spring(k) = events.interaction(k);
end
%% plot mean data
fig_m = figure('Position',fig_pos+[0 0.1*ss(4) 0 0]);
set(0, 'currentfigure', fig_m);
c0 = [0.5 1.0 0.5];
c1 = [0.1 0.6 0.4];
c = {c0;c1};

subplot(1,2,1);
hold on;
for sp = 0:1
    if ~isempty(find(spring == sp,1))
        scatter(find(spring == sp)',pos_rmse(spring == sp)',75,'o',...
                     'MarkerEdgeColor','k','MarkerFaceColor',c{sp+1});
        yline(mean(pos_rmse(spring == sp)),'--','Color',c{sp+1},'LineWidth',1.5);
    end
end
xticks(1:k);
ylabel('Position error (rad)');
xlabel('Trial number');
set(gca,'FontSize',12,'LineWidth',1)

subplot(1,2,2);
hold on;
for sp = 0:1
    if ~isempty(find(spring == sp,1))
        scatter(find(spring == sp)',tor_rmse(spring == sp)',75,'o',...
                     'MarkerEdgeColor','k','MarkerFaceColor',c{sp+1});
        yline(mean(tor_rmse(spring == sp)),'--','Color',c{sp+1},'LineWidth',1.5);
    end
end
xticks(1:k);
ylabel('Interaction torque error (Nm)');
xlabel('Trial number');
set(gca,'FontSize',12,'LineWidth',1)
if sum(spring) == 0
    legend({'no interaction',''},'FontSize',12,'Location','northeast');
elseif sum(spring) == length(spring)
    legend({'interaction',''},'FontSize',12,'Location','northeast');
else
    legend({'no interaction','','interaction',''},'FontSize',12,'Location','northeast');
end
sgtitle(fn,'Interpreter','none');
else
   error('csv file does not exist!');
end