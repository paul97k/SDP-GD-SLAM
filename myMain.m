
clear all
close all
clc
addpath('helpers');
nT_vec = [15,27,30,100,200];
C = [1 1 1];
nT = 50;
scaleFactor = 1000;



%
% for bag_id=94:144
% for bag_id=1:5
for bag_id=13:24   
% for bag_id=24:144   
    bag_id
    
    % stabdard method
    % batchSize = 1;
    % bag_id = 13;
    % nT = 100;
    
    scaled = false;
    filename = getFileNameLiftedModel(bag_id, nT, scaled, scaleFactor);
    load(filename)
    
    nominalVecC = repmat(C, 1, nT); 
    nominalVecC = reshape(nominalVecC', [], 1);
    nominalVecC = double(nominalVecC);
    
    
    
    estimateBias = false;
    sigmaV = sigma_v;
    sigmaX= sigma_wx;
    nominalX= lifted_A*lifted_u; 
    measuredY= measured_y; 
    sigmaC= sigma_w_theta; %sigmaw_theta
    sizeCk= size(C); 
    desiredSnr= 30; 
    matrixScale= 1; 
    trueVecC= [] ; 
    measuredY = double(measuredY);

    
    processSDPResults(bag_id, nT, estimateBias, sigmaV, sigmaX, nominalX, measuredY, ...
        nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC, ...
        measured_y, sigma_v, sigma_wx, mu_theta, sigma_w_theta, @map_cost_func);
    
    % Using batches
    % clear
    
    batchSize = 1;
    % nT = 100;
    
    
    scaleFactor = 1;
    scaled = false; 
    estimateBias = false;
    desiredSnr= 30; 
    matrixScale= 1; 
    trueVecC= [] ; 
    
    filenameLiftedModelBatches = getFileNameLiftedModel_batches(bag_id, nT, scaled, scaleFactor, batchSize);
    filenameLiftedModel = getFileNameLiftedModel(bag_id, nT, scaled, scaleFactor);
    
    tic;
    [nominalX, initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output]...
                = mapEstimate_batches(filenameLiftedModelBatches, filenameLiftedModel, batchSize, estimateBias, desiredSnr, matrixScale, trueVecC);
    
    elapsedTime = toc; % Stop the timer and get elapsed time
    
    
    filenameLiftedModelBatchesResult = getFileNameSDPResult_batches(bag_id, nT, batchSize);
    
    Computation_time = num2str(elapsedTime);
    disp(['Computation time SDP Batches: ', Computation_time, ' seconds']);
    
    
    save(filenameLiftedModelBatchesResult, 'initialVecVar', 'optimalVecVar', 'optimalValueCost', 'exitFlag', 'output', 'Computation_time');
    % disp(['SDP-GD using batches saved to ', filenameLiftedModelBatchesResult]);
    
    disp('--------------------------------------')

end

%% check exit flag
clc
clear
addpath('helpers');
nT = 50;
batchSize = 1;
% get SDP_result standard method
% for bag_idTrain = 1:144
for bag_idTrain = 1:22
    filenameSDPResult = getFileNameSDPResult(bag_idTrain, nT);
    SDPResult = load(filenameSDPResult);
    
    
    filenameSDPResult_batches = getFileNameSDPResult_batches(bag_idTrain, nT, batchSize);
    SDPResult_batches = load(filenameSDPResult_batches);
    
    % Print the exitFlag for SDPResult_batcehes
    disp([num2str(bag_idTrain), ': '])
    disp(['Computation_time', ': ', num2str(SDPResult.Computation_time),',' num2str(SDPResult_batches.Computation_time)]);
    disp(['optimalValueCost', ': ', num2str(SDPResult.optimalValueCost),',' num2str(SDPResult_batches.optimalValueCost)]);
    disp(['iterations', ': ', num2str(SDPResult.output.iterations),',' num2str(SDPResult_batches.output.iterations)]);
    disp(' ')
end


%% Plot results using saved output map
clc
close all
addpath('helpers');
clear

bag_idTrain =12;          %camera_0.3_0.1_lidar_0_0.20
bag_idTest = 13;    %camera_0.18_0.1_lidar_0_0.02'

nT = 50;
batchSize = 1;
scaled = false;
scaleFactor = 1000;

getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor)
getFileNameLiftedModel(bag_idTrain, nT, scaled, scaleFactor)

test_data = load(getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor));
train_data = load(getFileNameLiftedModel(bag_idTrain, nT, scaled, scaleFactor));

if bag_idTrain>24
    over_id = mod((bag_idTrain-24),12);
    
    Slam_SDP_result = load(getFileNameSLAM(12+over_id, nT));
else
    Slam_SDP_result = load(getFileNameSLAM(bag_idTrain, nT));
end
if bag_idTest>24
    over_id = mod((bag_idTest-24),12);
    Slam_SDP_Test = load(getFileNameSLAM(12+over_id, nT));
else
    Slam_SDP_Test = load(getFileNameSLAM(bag_idTest, nT));
end

SDPResult = load(getFileNameSDPResult(bag_idTrain, nT));
SDPResult_batches= load(getFileNameSDPResult_batches(bag_idTrain, nT, batchSize));


% % 
% height_bySLAM = calculate_height_difference(Slam_SDP_result.slam_map, Slam_SDP_result.SLAM_trajectory);

figure('Position', [450, 200, 1000, 800]);
subplot(3, 3, 1); 

plot(test_data.measured_y, '*')
hold on
plot(train_data.measured_y)
legend('test data', 'train data')
title('Trained: Height by slam vs measured')

C = [1 1 1];
nominalX= test_data.lifted_A*test_data.lifted_u; 
sizeCk= size(C);


% get SDP_result standard method
% load(filenameSDPResult)
% analyze_and_visualize_comparison(initialVecVar, optimalVecVar, sizeCk, nT, ...
%     estimate_bias, nominalX, measured_y, SLAM_height_diff, sigma_v, sigma_wx, mu_theta, ...
%     sigma_w_theta, @map_cost_func);
subplot(3, 3, 2); 
disp('SDO-GD version 1')
analyze_and_visualize_comparison_withSLAM(SDPResult.optimalVecVar, sizeCk, nT, ...
    test_data.estimate_bias, nominalX, test_data.measured_y, test_data.SLAM_height_diff);


subplot(3, 3, 3); 

disp('SDO-GD version 2: Using batches')
analyze_and_visualize_comparison_withSLAM(SDPResult_batches.optimalVecVar, sizeCk, nT, ...
    test_data.estimate_bias, nominalX, test_data.measured_y, test_data.SLAM_height_diff,batchSize);

plotTrajectoryComparison(Slam_SDP_result.SLAM_trajectory, Slam_SDP_result.GT_trajectory)

% subplot(3, 3, 8); 
subplot('Position', [0.1, 0.1, 0.8, 0.23]);  % Custom position: [left, bottom, width, height]
plotGTComparison(Slam_SDP_result.GT_trajectory, Slam_SDP_Test.GT_trajectory)



% plotXYZ(x_simulated)
% Axes labels

%% weird box plots
close all
clc
clear
bag_idTrain = 13;

nT = 50;batchSize = 1;
scaled = false;
scaleFactor = 1000;

filenameSDPResult = getFileNameSDPResult(bag_idTrain, nT);
residual_stats = analyze_residual_statistics(bag_idTrain, nT, scaled, scaleFactor);
residual_stats_Quarters = analyze_residual_statistics_Quarters(filenameSDPResult, nT, scaled, scaleFactor);


% % 
% filenameSDPResult_batches = getFileNameSDPResult_batches (bag_idResult, nT, batchSize);
% residual_stats = analyze_residual_statistics(filenameSDPResult_batches, nT, scaled, scaleFactor);


% Suppose 'residual_stats' is your matrix of size [totalBags x 8], where:
% Q1_Mean = col 1, Q1_Var = col 2, Q2_Mean = col 3, Q2_Var = col 4, ...
means = residual_stats_Quarters(:, [1, 3, 5, 7]);
vars = residual_stats_Quarters(:, [2, 4, 6, 8]);
stds = sqrt(vars);

quarters = 1:4;
figure; hold on;
for i = 1:size(means,1)
    errorbar(quarters, means(i,:), stds(i,:), '-o', 'DisplayName', sprintf('Bag %d', i));
end
xlabel('Quarter');
ylabel('Residual Mean');
title('Residual Means with Error Bars (Std Dev) per Quarter');
legend('show');
grid on;


figure;
b = bar(means); % grouped bar chart
hold on;
% Calculate the number of groups and number of bars in each group
[ngroups, nbars] = size(means);
% Get the x-coordinate of the bars
x = nan(nbars, ngroups);
for i = 1:nbars
    x(i,:) = b(i).XData + b(i).XOffset;
end

% Plot error bars
errorbar(x', means, stds, 'k', 'linestyle', 'none');

xlabel('Bag ID');
ylabel('Residual Mean');
set(gca, 'XTick', 1:ngroups, 'XTickLabel', arrayfun(@num2str, 1:ngroups, 'UniformOutput', false));
legend({'Q1','Q2','Q3','Q4'}, 'Location','best');
title('Mean Residuals by Quarter with Error Bars');
grid on;
hold off;


boxplot(means, 'Labels', {'Q1','Q2','Q3','Q4'});
title('Boxplot of Residual Means Across Quarters');
ylabel('Mean Residual');


means = residual_stats_Quarters(:, [1, 3, 5, 7]);
stds = sqrt(residual_stats_Quarters(:, [2, 4, 6, 8]));

figure;
for q = 1:4
    subplot(2,2,q);
    errorbar(1:size(means,1), means(:,q), stds(:,q), 'o-');
    xlabel('Bag ID');
    ylabel('Mean Residual');
    title(sprintf('Quarter %d', q));
    grid on;
end
sgtitle('Residual Statistics Per Quarter');


%% this should be done for data with a standard noise distrubtion 
close all
clc;
bag_idTrain = 13;

nT = 50;batchSize = 1;
scaled = false;
scaleFactor = 1000;

% Calculate residual statistics
residual_stats = analyze_residual_statistics(bag_idTrain, nT, scaled, scaleFactor);

% Extract overall mean and variance from residual_stats for GD residuals
means_gd = residual_stats(:, 1); % Overall means for GD residuals
vars_gd = residual_stats(:, 2); % Overall variances for GD residuals
stds_gd = sqrt(vars_gd);        % Standard deviations for GD residuals

% Extract overall mean and variance from residual_stats for SLAM residuals
means_slam = residual_stats(:, 3); % Overall means for SLAM residuals
vars_slam = residual_stats(:, 4);  % Overall variances for SLAM residuals
stds_slam = sqrt(vars_slam);       % Standard deviations for SLAM residuals

% Generate synthetic residuals for the GD noise distribution
synthetic_residuals_gd = [];
for i = 1:length(means_gd)
    synthetic_residuals_gd = [synthetic_residuals_gd; normrnd(means_gd(i), stds_gd(i), 100, 1)];
end

% Generate synthetic residuals for the SLAM noise distribution
synthetic_residuals_slam = [];
for i = 1:length(means_slam)
    synthetic_residuals_slam = [synthetic_residuals_slam; normrnd(means_slam(i), stds_slam(i), 100, 1)];
end

% Plot the noise distribution for GD residuals
figure;
subplot(2, 1, 1);
histogram(synthetic_residuals_gd, 'Normalization', 'pdf', 'FaceColor', [0.7 0.7 0.9], 'EdgeColor', [0.4 0.4 0.6]);
hold on;
mu_gd = mean(synthetic_residuals_gd);  % Mean of all GD residuals
sigma_gd = std(synthetic_residuals_gd); % Standard deviation of all GD residuals
x_gd = linspace(min(synthetic_residuals_gd), max(synthetic_residuals_gd), 100);
pdf_gd = (1 / (sigma_gd * sqrt(2 * pi))) * exp(-(x_gd - mu_gd).^2 / (2 * sigma_gd^2)); % Gaussian PDF for GD
plot(x_gd, pdf_gd, 'r-', 'LineWidth', 2);
title('Noise Distribution (GD Residuals)');
xlabel('Residual Value');
ylabel('Probability Density');
legend({'Histogram', 'Gaussian Fit'}, 'Location', 'best');
grid on;
hold off;

% Plot the noise distribution for SLAM residuals
subplot(2, 1, 2);
histogram(synthetic_residuals_slam, 'Normalization', 'pdf', 'FaceColor', [0.9 0.7 0.7], 'EdgeColor', [0.6 0.4 0.4]);
hold on;
mu_slam = mean(synthetic_residuals_slam);  % Mean of all SLAM residuals
sigma_slam = std(synthetic_residuals_slam); % Standard deviation of all SLAM residuals
x_slam = linspace(min(synthetic_residuals_slam), max(synthetic_residuals_slam), 100);
pdf_slam = (1 / (sigma_slam * sqrt(2 * pi))) * exp(-(x_slam - mu_slam).^2 / (2 * sigma_slam^2)); % Gaussian PDF for SLAM
plot(x_slam, pdf_slam, 'r-', 'LineWidth', 2);
title('Noise Distribution (SLAM Residuals)');
xlabel('Residual Value');
ylabel('Probability Density');
legend({'Histogram', 'Gaussian Fit'}, 'Location', 'best');
grid on;
hold off;
%%
clc 
% clear all 


nT = 50;batchSize = 1;
scaled = false;
scaleFactor = 1000;

camera_mean = [0.08:0.02:0.3];  % Camera mean values
lidar_var = [0:0.02:0.2];       % Lidar variance values

nx = size(camera_mean,2);
ny = size(lidar_var,2);
nz = 144-12; %total bags

% residual_matrix = zeros(nx, ny, nz); %x,y,z
% residual_matrix_SLAM = zeros(nx, ny, nz); %x,y,z

load('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM');

% for bag_idResult = 13:144
tic
% for bag_idResult = 13:123
for bag_idResult = 13:24
    bag_idResult
    bag_idTrain = bag_idResult-12;
    residual_stats = analyze_residual_statistics(bag_idResult, nT, scaled, scaleFactor);
        
    residual_matrix(:, :,bag_idTrain) = reshape(residual_stats(:,1), 12, 11);
    residual_matrix_SLAM(:, :,bag_idTrain) = reshape(residual_stats(:,3), 12, 11);
    % for nx_i = 1:nx
    %     for ny_i = 1:ny
    % 
    %              residual_matrix(nx_i, ny_i,nz_i) = residual_stats(nx_i*ny_i,1);
    %         residual_matrix_SLAM(nx_i, ny_i,nz_i) = residual_stats(nx_i*ny_i,3);
    %     end
    % end
end

elapsetime_1 = toc
%%
save('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM');
disp('Matrices saved to residual_data.mat');


%% heat map 1 result 
clc
close all
% Extract first slice

% for bag_idResult = 1:12:120
% for bag_idResult = 1:12
for bag_idResult = 13:24
% for bag_idResult = 109-12:120-12

    first_slice = residual_matrix(:, :, bag_idResult);

    % Plot with customized axis labels
    figure('Position', [100, 10, 1200, 800]); % [left, bottom, width, height]
    subplot(1,2,1)
    imagesc(first_slice);
    colorbar;
    title(['Heatmap of residual\_matrix(:, :, ', num2str(bag_idResult), ')' ]);
    xlabel('Lidar Variance');
    ylabel('Camera Mean');
    set(gca, 'XTick', 1:ny, 'XTickLabel', lidar_var); % Set X-axis labels
    set(gca, 'YTick', 1:nx, 'YTickLabel', camera_mean); % Set Y-axis labels
    set(gca, 'YDir', 'normal'); % Ensure the y-axis is in normal order



    first_slice = residual_matrix_SLAM(:, :, bag_idResult);
    subplot(1,2,2)
    imagesc(first_slice);
    colorbar;
    title(['Heatmap of residual\_matrix\_SLAM(:, :, ', num2str(bag_idResult), ')' ]);
    % title(['Heatmap residual SLAM with (camera_mean_s, lidar_variance_s)', num2str(camera_mean
    xlabel('Sonar Variance');
    ylabel('Camera Mean');
    set(gca, 'XTick', 1:ny, 'XTickLabel', lidar_var); % Set X-axis labels
    set(gca, 'YTick', 1:nx, 'YTickLabel', camera_mean); % Set Y-axis labels
    set(gca, 'YDir', 'normal'); % Ensure the y-axis is in normal order

end

%% all elements
close all
clc
% Reshape th
load('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM')
reshaped_matrix = reshape(residual_matrix, 132, 132); % Reshape to 132x132reshaped_matrix = reshape(residual_matrix, 132, 132);

% Initialize reordered matrix
reordered_matrix = zeros(size(reshaped_matrix));

% Reorder the rows based on the given logic
for i = 1:11
    for n = 1:12
        % i 
        % n
        % d1 = (i-1)*12+n
        d2 = (n-1)*12+i;
        if d2 < 133
            % reordered_matrix(:, (i-1)*12+n) = reshaped_matrix(:,(n-1)*12+i);
            reordered_matrix((i-1)*12+n,:) = reshaped_matrix((n-1)*12+i,:);
        end
    end
end
% Variables for labels (lidar_var and camera_mean) need to be defined
camera_mean = [0.08:0.02:0.3];  % Camera mean values
lidar_var = [0:0.02:0.2];

% Plot the reshaped heatmap
figure('Position', [100, 15, 1200, 800]); % [left, bottom, width, height]
% subplot(1,2,1)
imagesc(reshaped_matrix);
colorbar;
title('Heatmap of Reshaped Last Element of residual\_matrix');
xlabel('sonar Variance');
ylabel('Camera Mean');

% Adjust X-axis ticks to match grouping of 12 elements for lidar_var
set(gca, 'XTick', 1:12:132, 'XTickLabel', lidar_var); 

% Adjust Y-axis ticks to map every mod(x,12) to an element in camera_mean
set(gca, 'YTick', 1:11:132, 'YTickLabel', camera_mean);

% Reverse y-axis order to ensure proper alignment
set(gca, 'YDir', 'normal'); % Ensure the y-axis is in normal order


% subplot(1,2,2)
% imagesc(reordered_matrix);
% colorbar;
% title('Heatmap of Reshaped Last Element of residual\_matrix');
% xlabel('Lidar Variance');
% ylabel('Camera Mean');
% 
% % Adjust X-axis ticks to match grouping of 12 elements for lidar_var
% set(gca, 'XTick', 1:12:132, 'XTickLabel', lidar_var); 
% 
% % Adjust Y-axis ticks to map every mod(x,12) to an element in camera_mean
% set(gca, 'YTick', 1:11:132, 'YTickLabel', camera_mean);
% 
% % Reverse y-axis order to ensure proper alignment
% set(gca, 'YDir', 'normal'); % Ensure the y-axis is in normal order

%% Parameters
nFrames = 74; % Number of frames for the animation

% Create the figure for the animation
figure('Position', [100, 100, 1200, 800]); % [left, bottom, width, height]
% Subplot 1: residual_matrix
subplot(1, 2, 1);
h1 = imagesc(lidar_var,camera_mean,  residual_matrix(:, :, 1));
colorbar; % Add color bar
title('Heatmap of residual\_matrix(:, :, 1)');
xlabel('sonar Variance');
ylabel('Camera Mean');
set(gca, 'YDir', 'normal'); % Ensure the y-axis is displayed in normal order

% Subplot 2: residual_matrix_SLAM
subplot(1, 2, 2);
h2 = imagesc(lidar_var, camera_mean, residual_matrix_SLAM(:, :, 1));
colorbar; % Add color bar
title('Heatmap of residual\_matrix\_SLAM(:, :, 1)');
xlabel('sonar Variance');
ylabel('Camera Mean');
set(gca, 'YDir', 'normal'); % Ensure the y-axis is displayed in normal order

% Loop through each slice and update both heatmaps
for frame = 1:nFrames
    % Update the first heatmap
    set(h1, 'CData', residual_matrix(:, :, frame));
    subplot(1, 2, 1);
    title(sprintf('Heatmap of residual\\_matrix - Frame %d/%d', frame, nFrames));
    
    % Update the second heatmap
    set(h2, 'CData', residual_matrix_SLAM(:, :, frame));
    subplot(1, 2, 2);
    title(sprintf('Heatmap of residual\\_matrix\\_SLAM - Frame %d/%d', frame, nFrames));
    
    % Pause for animation effect
    pause(0.5); % Adjust pause duration as needed
end



function residual_stats = analyze_residual_statistics(bag_idTrain, nT, scaled, scaleFactor)

    filenameSDPTrain = getFileNameSDPResult(bag_idTrain, nT);
    SDPResult = load(filenameSDPTrain);
    optimalVecVar = SDPResult.optimalVecVar;
    % filenameLiftedModel = getFileNameLiftedModel(bag_idResult, nT, scaled, scaleFactor);
    % [train_camera_mean, train_camera_variance, train_lidar_mean, train_lidar_variance] = extractMeanVariance(filenameLiftedModel);


    if bag_idTrain > 24
        over_id = mod((bag_idTrain - 24), 12);
        Slam_SDP_Train = load(getFileNameSLAM(12 + over_id, nT));
    else
        filenameSLAM = getFileNameSLAM(bag_idTrain, nT);
        Slam_SDP_Train = load(filenameSLAM);
    end

    totalBags = 144-12; % Define the total number of bags
    % Initialize matrix to store results (rows for bags, columns for mean and variance)
    residual_stats = zeros(totalBags, 4); % 4 columns: overall mean/variance for gd_predicted_y and height_bySLAM
    C = [1 1 1];
    sizeCk = size(C);

    % for bag_idTest = 13:totalBags+12
    for bag_idTest = 13:24
        bag_idTest
        % Load constant data files based on the given bag_idTest
        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        % [test_camera_mean, test_camera_variance, test_lidar_mean, test_lidar_variance] = extractMeanVariance(filenameLiftedModel);

        
        test_data = load(filenameLiftedModel);

        if bag_idTest > 24
            over_id = mod((bag_idTest - 24), 12);
            Slam_SDP_Test = load(getFileNameSLAM(12 + over_id, nT));
        else
            filenameSLAM = getFileNameSLAM(bag_idTest, nT);
            Slam_SDP_Test= load(filenameSLAM);
        end
        

        height_bySLAM = calculate_height_difference(Slam_SDP_Train.slam_map, Slam_SDP_Test.SLAM_trajectory);

        % Ground truth vs height_bySLAM
        % height_bySLAM = test_data.SLAM_height_diff;
        measured_y = test_data.measured_y; % Ground truth
        
        % Nominal trajectory
        nominalX = test_data.lifted_A * test_data.lifted_u;

        % Reconstruct and evaluate predictions
        [optimal_vec_theta, gd_predicted_y, residuals] = reconstruct_and_evaluate(...
            optimalVecVar, sizeCk, nT, test_data.estimate_bias, nominalX, measured_y);

        % % Compute residuals for gd_predicted_y and height_bySLAM
        residuals_squared_gd = pow2(residuals); % Squared residuals from gd_predicted_y
        residuals_height_bySLAM = pow2(measured_y - height_bySLAM); % Squared residuals from height_bySLAM

        % Calculate overall mean and variance for both sets of residuals
        overall_mean_gd = mean(residuals_squared_gd);
        overall_var_gd = var(residuals_height_bySLAM);

        overall_mean_height = mean(measured_y - height_bySLAM);
        overall_var_height = var(measured_y - height_bySLAM);

        % Store overall means and variances in the result matrix
        residual_stats(bag_idTest-12, 1) = overall_mean_gd; % Mean for gd_predicted_y
        residual_stats(bag_idTest-12, 2) = overall_var_gd;  % Variance for gd_predicted_y
        residual_stats(bag_idTest-12, 3) = overall_mean_height; % Mean for height_bySLAM
        residual_stats(bag_idTest-12, 4) = overall_var_height;  % Variance for height_bySLAM
    end

    % % Display the result matrix to the user
    % disp('Residual Statistics (Overall Mean and Variance):');
    % disp('Columns: [Overall_Mean_GD, Overall_Var_GD, Overall_Mean_Height, Overall_Var_Height]');
    % disp(residual_stats);
end




function [camera_mean, camera_variance, lidar_mean, lidar_variance] = extractMeanVariance(filenameLiftedModel)

    % Define a regular expression pattern
    pattern = 'camera_([\d.]+)_([\d.]+)_lidar_([\d.]+)_([\d.]+)';
    
    % Extract the values using regexp
    matches = regexp(filenameLiftedModel, pattern, 'tokens');
    
    % Check if matches were found
    if ~isempty(matches)
        % Convert the extracted strings to numbers
        camera_mean = str2double(matches{1}{1});
        camera_variance = str2double(matches{1}{2});
        lidar_mean = str2double(matches{1}{3});
        lidar_variance = str2double(matches{1}{4});
    else
        % If no matches found, set outputs to NaN and display a warning
        warning('No matches found in the filename.');
        camera_mean = NaN;
        camera_variance = NaN;
        lidar_mean = NaN;
        lidar_variance = NaN;
    end
end




function plotTrajectoryComparison(SLAM_trajectory, GT_trajectory)
    % Function to plot the comparison between SLAM and Ground Truth trajectories
    
    % Axes labels
    axes_labels = {'X', 'Y', 'Z'};
    
    % Create figure with specific position and size
    % figure('Position', [450, 200, 1000, 800]);
    
    % Plot each axis in a separate subplot
    for i = 1:length(axes_labels)
        subplot(3, 3, i+3);  % 3 rows, 1 column, plot on i-th subplot
        
        % Plot SLAM trajectory
        plot(SLAM_trajectory(:, i), 'LineStyle', '--', 'DisplayName', ['SLAM ', axes_labels{i}, '-axis']);
        hold on;
        
        % Plot Ground Truth trajectory
        plot(GT_trajectory(:, i), 'LineStyle', '-', 'DisplayName', ['GT ', axes_labels{i}, '-axis']);
        
        % Add titles, labels, and grid
        title([axes_labels{i}, '-Axis Comparison']);
        xlabel('Index');
        ylabel('Value');
        legend('show');
        grid on;
        
    end
end

function plotGTComparison(GT_Result, GT_Test)
    % Function to plot the comparison between SLAM and Ground Truth trajectories
    
    % Axes labels
    axes_labels = {'X', 'Y', 'Z'};
    
    % Create figure with specific position and size
    % figure('Position', [450, 200, 1000, 800]);
    
    % Plot SLAM trajectory
    plot(GT_Result(:, 1), GT_Result(:, 2), 'LineStyle', '-', 'DisplayName', 'GT trained trajectory');
    hold on;
    
    % Plot Ground Truth trajectory
    plot(GT_Test(:, 1), GT_Test(:, 2), 'LineStyle', '--', 'DisplayName', 'GT Test trajectory');
    
    % Add titles, labels, and grid
    title('XY-Axis Comparison ground truth test and results trajectory');
    xlabel('X [m]');
    ylabel('Y [m]');
    legend('show');
    grid on;
end
function residual_stats = analyze_residual_statistics_Quarters(filenameSDPResult, nT, scaled, scaleFactor)
   
    
    SDPResult = load(filenameSDPResult);

    totalBags = 144; % Define the total number of bags
    numQuarters = 4; % Number of quarters (25% segments)
    % Initialize matrix to store results (rows for bags, columns for mean/variance of each quarter)
    residual_stats = zeros(totalBags, numQuarters * 2); % 2 columns per quarter: mean and variance
    C = [1 1 1];
    sizeCk = size(C);

    % for bag_idTest = 13:totalBags
    for bag_idTest = 13:24
         % Load constant data files based on the given bag_idTest
        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        test_data = load(filenameLiftedModel);


        % filenameSLAM = getFileNameSLAM(bag_idTest, nT);
        % load(filenameSLAM);
        if bag_idTest>24
            over_id = mod((bag_idTest-24),12);
            load(getFileNameSLAM(12+over_id, nT));
        else
            filenameSLAM = getFileNameSLAM(bag_idTest, nT);
            load(filenameSLAM);
        end
        height_bySLAM = calculate_height_difference(slam_map, SLAM_trajectory);
        nominalX = test_data.lifted_A * test_data.lifted_u;
        
    

        % Reconstruct and evaluate predictions
        [optimal_vec_theta, gd_predicted_y, residuals] = reconstruct_and_evaluate(...
            SDPResult.optimalVecVar, sizeCk, nT, test_data.estimate_bias, nominalX, test_data.measured_y);

        % Square the residuals to calculate the error
        residuals_squared = pow2(residuals);

        % Divide the signal into quarters and compute mean/variance
        len = length(residuals_squared);
        quarterLength = floor(len / numQuarters);

        for q = 1:numQuarters
            startIdx = (q - 1) * quarterLength + 1;
            if q == numQuarters
                endIdx = len; % Ensure last quarter includes all remaining data
            else
                endIdx = q * quarterLength;
            end

            quarterData = residuals_squared(startIdx:endIdx);
            meanQuarter = mean(quarterData);
            varQuarter = var(quarterData);

            % Store mean and variance in the result matrix
            residual_stats(bag_idTest, (q - 1) * 2 + 1) = meanQuarter;
            residual_stats(bag_idTest, (q - 1) * 2 + 2) = varQuarter;
        end
    end

    % Display the result matrix to the user
    disp('Residual Statistics (Mean and Variance per Quarter):');
    disp('Columns: [Q1_Mean, Q1_Var, Q2_Mean, Q2_Var, Q3_Mean, Q3_Var, Q4_Mean, Q4_Var]');
    disp(residual_stats);
end





function analyze_and_plot_all_bags_with_constant_files_errors(bag_idResult, nT, scaled, scaleFactor, batchSize, subfig_i)
    filenameSDPResult = getFileNameSDPResult(bag_idResult, nT);
    SDPResult = load(filenameSDPResult);
    optimalVecVarSDP = SDPResult.optimalVecVar;

    totalBags = 11; % Define the total number of bags
    numQuarters = 4; % Number of quarters (25% segments)
    % Initialize matrix to store results (rows for bags, columns for mean/variance of each quarter)
    residual_stats = zeros(totalBags, numQuarters * 2); % 2 columns per quarter: mean and variance
    C = [1 1 1];
    sizeCk = size(C);
   

    totalBags = 11;

    % Initialize the first plot
    % figure;
    subplot(2, 2, subfig_i+1); 
    hold on;
    title(['Error SDP-GD vs SLAM for Bags (1-', num2str(totalBags), '), nT = ', num2str(nT), ' trained from ID', num2str(bag_idResult)]);
    xlabel('Time Step');
    ylabel('Output');
    ylim([0,5])
    legendEntries = {};

    % Iterate over all bags from 1 to totalBags
    for bag_idTest = 1:totalBags
        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        load(filenameLiftedModel);
        filenameSLAM = getFileNameSLAM(bag_idTest, nT);
        SLAM_data = load(filenameSLAM);
    
        % Compute SLAM height difference and initialize parameters
        height_bySLAM = calculate_height_difference(SLAM_data.slam_map, SLAM_data.SLAM_trajectory);
        C = [1 1 1];
        nominalX = lifted_A * lifted_u;
  
        % Reconstruct and evaluate predictions
        [optimal_vec_theta, gd_predicted_y, residuals] = reconstruct_and_evaluate(...
            optimalVecVarSDP, sizeCk, nT, estimate_bias, nominalX, measured_y);

        % Plot results for the current bag
        plot(pow2(residuals), 'DisplayName', ['SDP-GD (Bag ', num2str(bag_idTest), ')']);
        legendEntries{end+1} = ['SDP-GD (Bag ', num2str(bag_idTest), ')'];
        h1 = plot(pow2(measured_y-height_bySLAM), 'k-', 'LineWidth', 1.5);
        legendEntries{end+1} = '';
    
        % Finalize the legend
        legend(legendEntries, 'Location', 'Best');
        grid on;
    end


    % Initialize the first plot
    % figure;
    subplot(2, 2, subfig_i+2); 
    hold on;
    title(['Moving avg Error SDP-GD vs SLAM for Bags (1-', num2str(totalBags), '), nT = ', num2str(nT), ' trained from ID', num2str(bag_idResult)]);
    xlabel('Time Step');
    ylabel('Output');
    ylim([0,3])
    legendEntries = {};

    % Iterate over all bags from 1 to totalBags
    for bag_idTest = 1:totalBags
        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        load(filenameLiftedModel);
        filenameSLAM = getFileNameSLAM(bag_idTest, nT);
        SLAM_data = load(filenameSLAM);
    
        % Compute SLAM height difference and initialize parameters
        height_bySLAM = calculate_height_difference(SLAM_data.slam_map, SLAM_data.SLAM_trajectory);
        C = [1 1 1];
        nominalX = lifted_A * lifted_u;
  
        % Reconstruct and evaluate predictions
        [optimal_vec_theta, gd_predicted_y, residuals] = reconstruct_and_evaluate(...
            optimalVecVarSDP, sizeCk, nT, estimate_bias, nominalX, measured_y);
        
        residuals_moving_avg = movmean(pow2(residuals), 10);
        plot(residuals_moving_avg, 'DisplayName', ['Moving Average (10 points) SDP-GD (Bag ', num2str(bag_idTest), ')']);

        legendEntries{end+1} = ['SDP-GD (Bag ', num2str(bag_idTest), ')'];
        
        residuals_moving_avg = movmean(pow2(measured_y - height_bySLAM), 10);
        h2 = plot(residuals_moving_avg, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Moving Average (10 points)');
        % legendEntries{end+1} = 'Moving Average (10 points)';
    
        legendEntries{end+1} = '';
    
        % Finalize the legend
        legend(legendEntries, 'Location', 'Best');
        grid on;
    end

    


end

function analyze_and_plot_all_bags_with_constant_files(bag_idResult, nT, scaled, scaleFactor, batchSize, subfig_i)
    % Load constant data files based on the given bag_idTest

    filenameSDPResult = getFileNameSDPResult(bag_idResult, nT);
    SDPResult = load(filenameSDPResult);
    optimalVecVarSDP = SDPResult.optimalVecVar;


    filenameSDPResult_batches = getFileNameSDPResult_batches(bag_idResult, nT, batchSize);
    SDPResult_batches =load(filenameSDPResult_batches);
    optimalVecVarSDP_batches = SDPResult_batches.optimalVecVar;


    totalBags = 11; % Define the total number of bags
    numQuarters = 4; % Number of quarters (25% segments)
    % Initialize matrix to store results (rows for bags, columns for mean/variance of each quarter)
    residual_stats = zeros(totalBags, numQuarters * 2); % 2 columns per quarter: mean and variance
    C = [1 1 1];
    sizeCk = size(C);
    

    % Initialize the first plot
    % figure;
    subplot(2, 2, subfig_i+1); 
    hold on;
    title(['Method 1: Measured vs. SDP-GD vs SLAM for Bags (1-', num2str(totalBags), '), nT = ', num2str(nT), ' trained from ID', num2str(bag_idResult)]);
    xlabel('Time Step');
    ylabel('Output');
    legendEntries = {};

    % Iterate over all bags from 1 to totalBags
    for bag_idTest = 1:totalBags

        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        load(filenameLiftedModel);
        filenameSLAM = getFileNameSLAM(bag_idTest, nT);
        load(filenameSLAM);
        
    
        % Compute SLAM height difference and initialize parameters
        height_bySLAM = calculate_height_difference(slam_map, SLAM_trajectory);
        C = [1 1 1];
        nominalX = lifted_A * lifted_u;
  
        % Reconstruct and evaluate predictions
        [optimal_vec_theta, gd_predicted_y, residuals] = reconstruct_and_evaluate(...
            optimalVecVarSDP, sizeCk, nT, estimate_bias, nominalX, measured_y);

        % Plot results for the current bag
        plot(gd_predicted_y, 'DisplayName', ['SDP-GD (Bag ', num2str(bag_idTest), ')']);
        legendEntries{end+1} = ['SDP-GD (Bag ', num2str(bag_idTest), ')'];
        
        
        % Plot constant SLAM and measured outputs
        h1 = plot(measured_y, 'k-', 'LineWidth', 1.5);
        legendEntries{end+1} = 'Measured Output';

        h2 = plot(height_bySLAM, 'b', 'LineWidth', 1.5);
        legendEntries{end+1} = 'SLAM Output';

    end


    % Finalize the legend
    legend(legendEntries, 'Location', 'Best');
    grid on;

    % Initialize the second plot for batch analysis
    % figure;
    subplot(2, 2, subfig_i+2); 
    hold on;
    title(['Method 2: Using batches. Measured vs. SDP-GD vs SLAM for Bags (1-', num2str(totalBags), ') in batches, nT = ', num2str(nT), ' trained from ID', num2str(bag_idResult)]);
    xlabel('Time Step');
    ylabel('Output');
    legendEntries = {};

    % Iterate over all bags from 1 to totalBags
    for bag_idTest = 1:totalBags
        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        load(filenameLiftedModel);
        filenameSLAM = getFileNameSLAM(bag_idTest, nT);
        load(filenameSLAM);

        height_bySLAM = calculate_height_difference(slam_map, SLAM_trajectory);
        C = [1 1 1];
        nominalX = lifted_A * lifted_u;
    
 
        [optimal_vec_theta, gd_predicted_y_batches, residuals] = reconstruct_and_evaluate(...
            optimalVecVarSDP_batches, sizeCk, nT, estimate_bias, nominalX, measured_y);

        % Plot results for the current bag
        plot(gd_predicted_y_batches, 'DisplayName', ['SDP-GD (Bag ', num2str(bag_idTest), ')']);
        legendEntries{end+1} = ['SDP-GD (Bag ', num2str(bag_idTest), ')'];
        

            % % Plot constant SLAM and measured outputs
            h3 = plot(measured_y, 'k-', 'LineWidth', 1.5);
            legendEntries{end+1} = 'Measured Output';
            % h4 = plot(height_bySLAM, 'b--', 'LineWidth', 1.5);
            % legendEntries{end+1} = 'SLAM Output';
    end
    % Finalize the legend
    legend(legendEntries, 'Location', 'Best');
    grid on;
end



function analyze_and_visualize_comparison(initialVecVar, optimalVecVar, sizeCk, nT, ...
    estimate_bias, nominalX, measured_y, SLAM_height_diff, sigma_v, sigma_wx, mu_theta, ...
    sigma_w_theta, map_cost_func, varargin)

    % Check if batchSize is provided as an optional argument
    if ~isempty(varargin)
        batchSize = varargin{1};  % The first optional argument
        titleI = ['Error Over Time (SDP vs. GD) in batches with, nT = ', num2str(nT), ' batchSize = ',  num2str(batchSize)];
    else
        batchSize = [];  % Set a default value if not provided
        titleI = ['Error Over Time (SDP vs. GD), nT = ', num2str(nT)];
    end


    % 1. Reconstruct Parameters for SDP
    [sdp_vec_theta, sdp_predicted_y, sdp_residuals] = reconstruct_and_evaluate(...
        initialVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y, ...
        sigma_v, sigma_wx, mu_theta, sigma_w_theta, map_cost_func);

    % 2. Reconstruct Parameters for GD
    [gd_vec_theta, gd_predicted_y, gd_residuals] = reconstruct_and_evaluate(...
        optimalVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y, ...
        sigma_v, sigma_wx, mu_theta, sigma_w_theta, map_cost_func);

    % 3. Visualization: Measured vs. Predicted Outputs
    figure;
    plot(measured_y, 'DisplayName', 'Sonar output');
    hold on;
    plot(SLAM_height_diff, 'DisplayName', 'SLAM Output');
    plot(sdp_predicted_y, 'x-', 'DisplayName', 'SDP Predicted Output');
    plot(gd_predicted_y, '*-', 'DisplayName', 'GD Predicted Output');
    xlabel('Time Step');
    ylabel('Output');
    title('Measured vs. Predicted Output (SDP vs. GD)');
    legend;
    grid on;

    % % 4. Visualization: Residuals
    % figure;
    % plot(sdp_residuals, '.-', 'DisplayName', 'SDP Residuals');
    % hold on;
    % plot(gd_residuals, 's-', 'DisplayName', 'GD Residuals');
    % xlabel('Time Step');
    % ylabel('Residual');
    % title(titleI);
    % legend;
    % grid on;
    % 
    % % 5. Residual Statistics
    % sdp_residual_mean = mean(sdp_residuals);
    % sdp_residual_variance = var(sdp_residuals);
    % gd_residual_mean = mean(gd_residuals);
    % gd_residual_variance = var(gd_residuals);
    % 
    % disp(['SDP Residual Mean: ', num2str(sdp_residual_mean)]);
    % disp(['SDP Residual Variance: ', num2str(sdp_residual_variance)]);
    % disp(['GD Residual Mean: ', num2str(gd_residual_mean)]);
    % disp(['GD Residual Variance: ', num2str(gd_residual_variance)]);

    % 6. Compare Parameters (if needed)
    % You can extend this part if comparing the parameters between SDP and GD is required.
end

function analyze_and_visualize_comparison_withSLAM(optimalVecVar, sizeCk, nT, ...
    estimate_bias, nominalX, measured_y, SLAM_height_diff, varargin)

    % Check if batchSize is provided as an optional argument
    if ~isempty(varargin)
        batchSize = varargin{1};  % The first optional argument
        titleI = ['Measured vs. SDP-GD vs SLAM in batches, nT = ', num2str(nT), ' batchSize = ',  num2str(batchSize)];
    else
        batchSize = [];  % Set a default value if not provided
        titleI = ['Measured vs. SDP-GD vs SLAM, nT = ', num2str(nT)];
    end

    % Reconstruct and evaluate predicted outputs
    [~, gd_predicted_y, ~] = reconstruct_and_evaluate(... 
        optimalVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y);

    % Calculate Mean Squared Error (MSE)
    mse_gd = mean((gd_predicted_y - measured_y).^2);
    mse_slam = mean((SLAM_height_diff - measured_y).^2);

    % Calculate Variance
    var_gd = var(gd_predicted_y - measured_y);
    var_slam = var(SLAM_height_diff - measured_y);

    % Display MSE and Variance
    fprintf('MSE between SDP-GD and Measured: %.4f\n', mse_gd);
    fprintf('Variance between SDP-GD and Measured: %.4f\n', var_gd);
    fprintf('MSE between SLAM and Measured: %.4f\n', mse_slam);
    fprintf('Variance between SLAM and Measured: %.4f\n', var_slam);
    disp('----------------------------------------------------------')

    % Visualization: Measured vs. Predicted Outputs
    % figure;
    plot(SLAM_height_diff, 'DisplayName', 'SLAM Output');
    hold on;
    plot(measured_y, 'DisplayName', 'Measured Output');
    plot(gd_predicted_y, '*-', 'DisplayName', 'SDP-GD Predicted Output');
    xlabel('Time Step');
    ylabel('Output');
    title(titleI);
    legend;
    grid on;

end


function [optimal_vec_theta, predicted_y, residuals] = reconstruct_and_evaluate(...
    optimal_vec_var, size_ck, nT, estimate_bias, nominalX, measured_y)
    
    % Extract sizes
    vec_theta_size = prod(size_ck) * nT;
    optimal_vec_theta = reshape(optimal_vec_var(1:vec_theta_size), [], 1);
    
    % Reconstruct `C_theta`
    estimated_C_theta = construct_C_theta(optimal_vec_theta, size_ck, nT);
    
    % Bias estimation
    if estimate_bias
        optimal_bias = optimal_vec_var(vec_theta_size + (1:size_ck(1)));
        vec_bias = repmat(optimal_bias, nT, 1);
    else
        vec_bias = zeros(size_ck(1) * nT, 1);
    end
    
    % Predicted output and residuals
    predicted_y = estimated_C_theta * nominalX + vec_bias;
    residuals = measured_y - predicted_y;
    

end


function C_theta = construct_C_theta(vec_theta, size_ck, nT)
    nC0 = size_ck(1);
    nC1 = size_ck(2);
    C_theta_blocks = cell(nT, nT);
    
    idx = 1; % Start index for extracting elements
    
    for i = 1:nT
        for j = 1:nT
            if i == j
                % Extract diagonal block
                block_size = nC0 * nC1;
                block_elements = vec_theta(idx:idx + block_size - 1);
                block = reshape(block_elements, nC0, nC1);
                idx = idx + block_size;
            else
                % Off-diagonal blocks are zeros
                block = zeros(nC0, nC1);
            end
            C_theta_blocks{i, j} = block;
        end
    end
    
    % Construct block matrix
    C_theta = cell2mat(C_theta_blocks);
end


function find_min_max_nonzero(matrix)

    % Ensure we are working with absolute values
    abs_elements = abs(matrix(:));
    
    % Exclude zeros for the lowest element
    non_zero_elements = abs_elements(abs_elements > 0);

    % Find the lowest element (smallest non-zero absolute value)
    lowest_element = min(non_zero_elements);

    % Find the highest element (largest absolute value)
    highest_element = max(abs_elements);

    % Display results
    disp(['Lowest element: ', num2str(lowest_element)]);
    disp(['Highest element: ', num2str(highest_element)]);
end


function plotXYZ(x_simulated)
    time = 1:size(x_simulated, 1);
    figure;
    plot(time, x_simulated(:, 1), 'DisplayName', 'x'); % x signal
    hold on;
    plot(time, x_simulated(:, 2), 'DisplayName', 'y'); % y signal
    plot(time, x_simulated(:, 3), 'DisplayName', 'z'); % z signal
    hold off;
    
    % Add labels, legend, and title
    xlabel('Time (samples)');
    ylabel('Signal Value');
    title('Simulated Signals');
    legend('show');
    grid on;
end


function filename = getFileNameSDPResult_batches(bag_id, nT, batchSize)
    bag_names = get_bag_name();
    bag_name = bag_names{bag_id};
    filename = sprintf('%s_nT_%d_batchSize_%d.mat', bag_name, nT, batchSize);
    folder_path = 'SDP_result/using_batches';
    
    filename = fullfile(folder_path, filename);

    % disp(['load SDP Result from ', filename])
    

end


function filename = getFileNameSDPResult(bag_id, nT)
    bag_names = get_bag_name();
    bag_name = bag_names{bag_id};
    filename = sprintf('%s_nT_%d.mat', bag_name, nT);
    folder_path = 'SDP_result';
    
    filename = fullfile(folder_path, filename);

    % disp(['load SDP Result from ', filename])
    

end




function filename = saveSDPResultInFilename(bag_id, nT)
    bag_names = get_bag_name();
    bag_name = bag_names{bag_id};

    filename = sprintf('%s_nT_%d.mat', bag_name, nT);
    folder_path = 'SDP_result';
    
    filename = fullfile(folder_path, filename);
    
end


function processSDPResults(bag_id, nT, estimateBias, sigmaV, sigmaX, nominalX, measuredY, ...
    nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC, ...
    measured_y, sigma_v, sigma_wx, mu_theta, sigma_w_theta, map_cost_func)

    % Start timer
    tic;

    % Call the mapEstimateC function
    [initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
        nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC);

    % Stop the timer and get elapsed time
    Computation_time = toc;
    disp(['Computation time SDP: ', num2str(Computation_time), ' seconds']);

    % Save results to a file
    filename = saveSDPResultInFilename(bag_id, nT);
    save(filename, 'initialVecVar', 'optimalVecVar', 'optimalValueCost', 'exitFlag', 'output', 'Computation_time');
    % disp(['Results saved to ', filename]);



end



function filename = getFileNameSLAM(bag_id, nT)
    bag_names = get_bag_name();
    

    % Validate the bag_id
    % if isKey(bag_names, bag_id)
    % 
    % else
    %     error('Invalid bag_id: %d. Please choose a valid ID.', bag_id);
    % end
    bag_name = bag_names{bag_id};

    filename = sprintf('%s_nT_%d.mat', bag_name, nT);
    folder_path = 'data/SLAM';
    filename = fullfile(folder_path, filename);

    % disp(['load SLAM from ', filename])
    
end





function filename = getFileNameLiftedModel_batches(bag_id, nT, scaled, scaleFactor, Batchsize)
    bag_names = get_bag_name();
    

    % Validate the bag_id
    % if isKey(bag_names, bag_id)
    % 
    % else
    %     error('Invalid bag_id: %d. Please choose a valid ID.', bag_id);
    % end
    bag_name = bag_names{bag_id};
    % Construct the filename
    if scaled
        filename = sprintf('%s_nT_%d_scaled_%d.mat', bag_name, nT, scaleFactor);
    else
        filename = sprintf('%s_nT_%d_Batchsize_%d.mat', bag_name, nT, Batchsize);
    end


    % Prepend the folder path
    folder_path = 'data/lifted_model/Batches';
    filename = fullfile(folder_path, filename);

    % disp(['load Lifted Model from ', filename])
    
end



function height_diff = calculate_height_difference(map_points, trajectory)


    % Extract X, Y coordinates from map and trajectory
    map_2d = map_points(:, 1:2);  % X, Y of the map
    traj_2d = trajectory(:, 1:2); % X, Y of the trajectory

    % Find nearest neighbors using knnsearch
    indices = knnsearch(map_2d, traj_2d);

    % Get the Z-values for the nearest points in the map
    map_z_nearest = map_points(indices, 3);

    % Get the Z-values for the trajectory
    traj_z = trajectory(:, 3);

    % Calculate the height difference (Z difference)
    height_diff = traj_z - map_z_nearest;
end
