%% Step response
clc;
clear;
close all
dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Step_Response';

StepResponeFile = ["SR0.01.mat", "SR0.1.mat", "SR0.2.mat", "SR0.5.mat", "SR0.7.mat", "SR1.0.mat", "SR1.2.mat"];
legendNames = ["0.01", "0.1", "0.2", "0.5", "0.7", "1.0", "1.2"];

% Define color order manually to ensure consistency across subplots
colors = lines(length(StepResponeFile)); % Generates distinguishable colors
plotHandles1 = []; % Store plot handles for legend (subplot 1)
plotHandles2 = []; % Store plot handles for legend (subplot 2)
plotHandles3 = []; % Store plot handles for legend (subplot 2)
legends1 = {}; % Store legends for subplot 1
legends2 = {}; % Store legends for subplot 2
legends3 = {}; % Store legends for subplot 2

figure; % Create a new figure

for i = 1:length(StepResponeFile)
    fileName = StepResponeFile{i}; % Extract filename as string
    legendName = legendNames{i}; % Extract filename as string
    
    obj.fullPath = fullfile(dataFolder, fileName);
    data = load(obj.fullPath);
    obj = DataLoader(dataFolder, fileName);
    obj.computeNED_AddNoise(false);
    
    input = obj.CMD;
    output = obj.NED;
    hyrdo = obj.hyrdro;
    GTvelObj = obj.velocityGT;
    GTposObj = obj.positionGT;
    xGT = GTposObj.x_positions;
    yGT = GTposObj.y_positions;
    zGT = GTposObj.z_positions;
    tGT = GTposObj.timestamps;
    GTvel_t = GTvelObj.timestamps;
    

    % Time window selection
    endTime = 10;
    startTime = 3; 
    tol = 0.1;  % Define a small tolerance


    cmd_timestamps = input.cmd_timestamps;
    output_t = obj.dvl.timestamps;

    cmd_input = input.cmd_linear_x;
    output = output.V_x;
    GTvel = GTvelObj.x_vel;
    posGt = zGT;

    % cmd_input = input.cmd_linear_y;
    % output = output.V_y;
    % GTvel = GTvelObj.y_vel;
    % posGt = yGT;
    % 
    % cmd_input = input.cmd_linear_z;
    % output = output.V_z;
    % GTvel = GTvelObj.z_vel;

    

    idSIn = find(abs(cmd_timestamps - startTime) < tol, 1);
    idSOut = find(abs(output_t - startTime) < tol, 1);
    idxIn = find(abs(cmd_timestamps - endTime) < tol, 1);
    idxOut = find(abs(output_t - endTime) < tol, 1);

    cmd_linear_x_cut =cmd_input(idSIn:idxIn);
    cmd_timestamps_cut =cmd_timestamps(idSIn:idxIn);

    V_x_cut =output(idSOut:idxOut);
    output_t_cut =output_t(idSOut:idxOut);

    % Ensure indices are valid
    if isempty(idSIn) || isempty(idSOut) || isempty(idxIn) || isempty(idxOut)
        warning('Skipping %s: No matching timestamps found in range.', fileName);
        continue;
    end

    % Compute moving averages
    windowSize = 5; 
    output_moving_avg = movmean(output, windowSize);
    output_moving_avg_cut = output_moving_avg(idSOut:idxOut);

    % Get color for this iteration
    color = colors(i, :);

    % Plot original signals
    subplot(3,1,1);
    hold on;
    h1 = plot(cmd_timestamps_cut, cmd_linear_x_cut, 'Color', color, 'LineStyle', '--', 'LineWidth', 1.5);
    h2 = plot(output_t_cut, V_x_cut, 'Color', color, 'LineWidth', 1.5);
    % plotHandles1 = [plotHandles1, h1, h2]; % Store handles
    % legends1 = [legends1, sprintf('Input %s', legendName), sprintf('Output %s', legendName)];
    % 

    h5 = plot(GTvel_t, GTvel, 'Color', color, 'LineStyle', '-.','LineWidth', 1.5);
    plotHandles1 = [plotHandles1, h1, h2, h5]; % Store handles
    legends1 = [legends1, sprintf('Input %s', legendName), sprintf('Output %s', legendName), sprintf('ground truth%s', legendName)];
    
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('INPUT vs Output Velocities');
    xlim([startTime, endTime]);

    % Plot moving averages
    subplot(3,1,2);
    hold on;
    h3 = plot(cmd_timestamps_cut, cmd_linear_x_cut, 'Color', color,'LineStyle', '--',  'LineWidth', 2);
    h4 = plot(output_t_cut, output_moving_avg_cut, 'Color', color, 'LineWidth', 2);
    plotHandles2 = [plotHandles2, h3, h4]; % Store handles
    legends2 = [legends2, sprintf('Input Avg %s', legendName), sprintf('Output Avg %s', legendName)]; % Store legends
    
    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('INPUT vs Output Velocities with Moving Average');
    xlim([startTime, endTime]);


        % Plot moving averages
    subplot(3,1,3);
    hold on;
    h6= plot( tGT, posGt, 'Color', color,  'LineWidth', 2);
    plotHandles3 = [plotHandles3,h6]; % Store handles
    legends3 = [legends3, sprintf('Step Amplitude: %s', legendName)]; % Store legends
    
    grid on;
    xlabel('t(seconds)');
    ylabel('Z (meters)');
    title('Ground truth position');



    % =================== STEP RESPONSE ANALYSIS ====================
   [timeVector,  stepInput, stepOutput] = interpolate_signals(cmd_timestamps,  cmd_input, output_t, output);
   % [timeVector,  stepInput, stepOutput] = interpolate_signals(cmd_timestamps,  cmd_linear_x, output_t, output_moving_avg);

   % [timeVector,  stepInput, stepOutput] = interpolate_signals(cmd_timestamps_cut,  cmd_linear_x_cut, output_t_cut, V_x_cut);
   % [timeVector,  stepInput, stepOutput] = interpolate_signals(cmd_timestamps_cut,  cmd_linear_x_cut, output_t_cut, output_moving_avg_cut);



    % Identify step change (assume first value is baseline)
    stepChange = stepInput(end) - stepInput(1);
    if stepChange == 0
        warning('%s: No step change detected. Skipping step analysis.', fileName);
        continue;
    end

    % Normalize step response
    normOutput = (stepOutput - stepOutput(1)) / stepChange;
    
    % Use stepinfo to get response characteristics
    response_info = stepinfo(normOutput, timeVector);
    

    % Display results
    fprintf('\nStep Response for %s:\n', fileName);
    disp(response_info);
end

% Set legends after all loops
subplot(3,1,1);
legend(plotHandles1, legends1);

subplot(3,1,2);
legend(plotHandles2, legends2);

subplot(3,1,2);
legend(plotHandles3, legends3);
%% Going straight


clc;
clear;
close all
dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
StepResponeFile = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat",];
legendNames = ["η(0,0)", "η_1(0.2 ,0.1) η_2(0 ,10), ", "η_1(1.2 ,0.2)  η_2(0 ,10)", "η_1(-0.2 ,0.1)  η_2(0 ,10)", "η_1(-1.2 ,0.2)  η_2(0 ,10)"];

% Define color order manually to ensure consistency across subplots
colors = lines(length(StepResponeFile)); % Generates distinguishable colors
plotHandles1 = []; % Store plot handles for legend (subplot 1)
plotHandles2 = []; % Store plot handles for legend (subplot 2)
plotHandles3 = []; % Store plot handles for legend (subplot 2)
legends1 = {}; % Store legends for subplot 1
legends2 = {}; % Store legends for subplot 2
legends3 = {}; % Store legends for subplot 2

BaselineFileName = StepResponeFile{1}; 
obj.fullPath = fullfile(dataFolder, BaselineFileName);
data = load(obj.fullPath);
obj = DataLoader(dataFolder, BaselineFileName);
obj.computeNED_AddNoise(false); %no noise added
output = obj.NED;
baseline  = output.V_x;
baseline_time = obj.dvl.timestamps;


figure; % Create a new figure
allMeans = zeros(1, length(StepResponeFile));

for i = 1:length(StepResponeFile)
    fileName = StepResponeFile{i}; % Extract filename as string
    legendName = legendNames{i}; % Extract filename as string
    
    obj.fullPath = fullfile(dataFolder, fileName);
    data = load(obj.fullPath);
    obj = DataLoader(dataFolder, fileName);
    obj.computeNED_AddNoise(false); %no noise added
    
    input = obj.CMD;
    output = obj.NED;
    hyrdo = obj.hyrdro;

    GTvelObj = obj.velocityGT;
    GTposObj = obj.positionGT;
    xGT = GTposObj.x_positions;
    yGT = GTposObj.y_positions;
    zGT = GTposObj.z_positions;
    tGT = GTposObj.timestamps;
    GTvel_t = GTvelObj.timestamps;
    

    cmd_timestamps = input.cmd_timestamps;
    output_t = obj.dvl.timestamps;
    
    cmd_input = input.cmd_linear_x;
    output = output.V_x;
    posGt = xGT;
    % 
    % cmd_input = input.cmd_linear_y;
    % output = output.V_y;
    % posGt = yGT;
    % 
    % cmd_input = input.cmd_linear_z;
    % output = output.V_z;
    % posGt = zGT;

    % Time window selection
    tol = 0.1;  % Define a small tolerance
    endTime = 80;
    startTime = 10; 

    idSIn = find(abs(cmd_timestamps - startTime) < tol, 1);
    idSOut = find(abs(output_t - startTime) < tol, 1);
    idxIn = find(abs(cmd_timestamps - endTime) < tol, 1);
    idxOut = find(abs(output_t - endTime) < tol, 1);

    cmd_linear_x_cut =cmd_input(idSIn:idxIn);
    cmd_timestamps_cut =cmd_timestamps(idSIn:idxIn);


    V_x_cut =output(idSOut:idxOut);
    output_t_cut =output_t(idSOut:idxOut);

    allMeans(i)= mean(V_x_cut); 

    % Ensure indices are valid
    if isempty(idSIn) || isempty(idSOut) || isempty(idxIn) || isempty(idxOut)
        warning('Skipping %s: No matching timestamps found in range.', fileName);
        continue;
    end

    % Compute moving averages
    windowSize = 10; 
    output_moving_avg = movmean(output, windowSize);
    output_moving_avg_cut = output_moving_avg(idSOut:idxOut);

    % Get color for this iteration
    color = colors(i, :);

    % Plot original signals
    subplot(3,1,1);
    hold on;
    % h1 = plot(cmd_timestamps_cut, cmd_linear_x_cut, 'Color', color, 'LineStyle', '--', 'LineWidth', 1.5);
    h2 = plot(output_t_cut, V_x_cut, 'Color', color, 'LineWidth', 1.5);
    plotHandles1 = [plotHandles1, h2]; % Store handles
    legends1 = [legends1,  sprintf('Output %s', legendName)]; % Store legends

    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('INPUT vs Output Velocities');
    xlim([startTime, endTime]);

    % Plot moving averages
    subplot(3,1,2);
    hold on;
    % h3 = plot(cmd_timestamps_cut, cmd_linear_x_cut, 'Color', color,'LineStyle', '--',  'LineWidth', 2);
    h4 = plot(output_t_cut, output_moving_avg_cut, 'Color', color, 'LineWidth', 2);
    plotHandles2 = [plotHandles2, h4]; % Store handles
    legends2 = [legends2,  sprintf('Output Avg %s', legendName)]; % Store legends

    

    grid on;
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('INPUT vs Output Velocities with Moving Average');
    xlim([startTime, endTime]);


    % Plot moving averages
    subplot(3,1,3);
    hold on;
    h6= plot( tGT, posGt, 'Color', color,  'LineWidth', 2);
    plotHandles3 = [plotHandles3,h6]; % Store handles
    legends3 = [legends3, sprintf('Step Amplitude: %s', legendName)]; % Store legends
    
    grid on;
    xlabel('t(seconds)');
    ylabel('X (meters)');
    title('Ground truth position');

    % % Second subplot - Angular velocities
    % % figure
    % hold on;
    % plot(hyrdo.hyrdro_timestamps, hyrdo.hyrdro_linear_x,  'LineWidth', 1.5); 
    % plot(hyrdo.hyrdro_timestamps, hyrdo.hyrdro_linear_y,  'LineWidth', 1.5);
    % plot(hyrdo.hyrdro_timestamps, hyrdo.hyrdro_linear_z,  'LineWidth', 1.5);
    % % hold off;
    % grid on;
    % xlabel('Time (s)');
    % ylabel('Velocity (m/s)');
    % title('Hydro current');
    % legend('X', 'Y', 'Z');



end
allMeans
% Set legends after all loops
subplot(3,1,1);
legend(plotHandles1, legends1);

subplot(3,1,2);
legend(plotHandles2, legends2);

subplot(3,1,2);
legend(plotHandles3, legends3);




% % Second subplot - Angular velocities
% subplot(2,1,2); % 2 rows, 1 column, second plot
% plot(cmd_timestamps, input.cmd_angular_x, 'r', 'LineWidth', 1.5); hold on;
% plot(cmd_timestamps, input.cmd_angular_y, 'g', 'LineWidth', 1.5);
% plot(cmd_timestamps, input.cmd_angular_z, 'b', 'LineWidth', 1.5);
% hold off;
% grid on;
% xlabel('Time (s)');
% ylabel('Angular Velocity');
% title('Angular Velocities (cmd\_angular)');
% legend('X', 'Y', 'Z');
function [common_time, interp_signal1, interp_signal2] = interpolate_signals(signal1_time, signal1, signal2_time, signal2)

    % Find the longest time vector
    all_times = {signal1_time, signal2_time};
    [~, longest_idx] = max(cellfun(@length, all_times));
    common_time = all_times{longest_idx};
    
    % Ensure the time vector is sorted
    common_time = sort(unique(common_time));
    
    % Interpolate each signal to match the common time vector
    interp_signal1 = interp1(signal1_time, signal1, common_time, 'linear', 'extrap');
    interp_signal2 = interp1(signal2_time, signal2, common_time, 'linear', 'extrap');
end

