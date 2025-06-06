
clear all
close all
clc
addpath('helpers');
nT_vec = [15,27,30,100,200];
C = [1 1 1];
nT = 50;
scaleFactor = 1000;


bag_names = get_bag_name();

%
% for bag_id=94:144
% for bag_id=1:5
% parfor bag_id=1:length(bag_names)
for bag_id=25:144   
    bag_id
    
    % stabdard method
    % batchSize = 1;
    % bag_id = 13;
    % nT = 100;
    scaleFactor = 1000;
    scaled = false;
    filename = getFileNameLiftedModel(bag_id, nT, scaled, scaleFactor);
    filenameObj = load(filename);
    
    nominalVecC = repmat(C, 1, nT); 
    nominalVecC = reshape(nominalVecC', [], 1);
    nominalVecC = double(nominalVecC);
    
    
    
    estimateBias = false;
    sigmaV = filenameObj.sigma_v;
    sigmaX= filenameObj.sigma_wx;
    nominalX= filenameObj.lifted_A*filenameObj.lifted_u; 
    measuredY= filenameObj.measured_y; 
    sigmaC= filenameObj.sigma_w_theta; %sigmaw_theta
    sizeCk= size(C); 
    desiredSnr= 30; 
    matrixScale= 1; 
    trueVecC= [] ; 
    measuredY = double(measuredY);

    
    processSDPResults(bag_id, nT, estimateBias, sigmaV, sigmaX, nominalX, measuredY, ...
        nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC, ...
        filenameObj.measured_y, filenameObj.sigma_v, filenameObj.sigma_wx, filenameObj.mu_theta, filenameObj.sigma_w_theta, @map_cost_func);

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
    
    
    % filenameLiftedModelBatchesResult = getFileNameSDPResult_batches(bag_id, nT, batchSize);
    
    Computation_time = num2str(elapsedTime);
    disp(['Computation time SDP Batches: ', Computation_time, ' seconds']);
    
    
    % save(filenameLiftedModelBatchesResult, 'initialVecVar', 'optimalVecVar', 'optimalValueCost', 'exitFlag', 'output', 'Computation_time');
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
% for bag_idTrain = 1:1
bag_idTrain = 9
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
% end


%% Plot results using saved output map
clc
close all
addpath('helpers');
clear
% 1 9 17
bag_idTrain =17;          %camera_0.3_0.1_lidar_0_0.20
bag_idTest = bag_idTrain+1;    %camera_0.18_0.1_lidar_0_0.02'

nT = 50;
batchSize = 1;
scaled = false;
scaleFactor = 1000;

getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor)
getFileNameLiftedModel(bag_idTrain, nT, scaled, scaleFactor)

test_data = load(getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor));
train_data = load(getFileNameLiftedModel(bag_idTrain, nT, scaled, scaleFactor));
% 
% if bag_idTrain>24
%     over_id = mod((bag_idTrain-24),12);
% 
%     Slam_SDP_result = load(getFileNameSLAM(12+over_id, nT));
% else
%     Slam_SDP_result = load(getFileNameSLAM(bag_idTrain, nT));
% end
% if bag_idTest>24
%     over_id = mod((bag_idTest-24),12);
%     Slam_SDP_Test = load(getFileNameSLAM(12+over_id, nT));
% else
%     Slam_SDP_Test = load(getFileNameSLAM(bag_idTest, nT));
% end
Slam_SDP_result = load(getFileNameSLAM(bag_idTrain, nT));
Slam_SDP_Test = load(getFileNameSLAM(bag_idTest, nT));
SDPResult = load(getFileNameSDPResult(bag_idTrain, nT));
SDPResult_batches= load(getFileNameSDPResult_batches(bag_idTrain, nT, batchSize));


% % 
% height_bySLAM = calculate_height_difference(Slam_SDP_result.slam_map, Slam_SDP_result.SLAM_trajectory);

figure('Position', [450, 200, 1000, 800]);
% subplot(3, 3, 1); 
% 
% plot(test_data.measured_y, '*')
% hold on
% plot(train_data.measured_y)
% legend('test data', 'train data')
% title('Trained: Height by slam vs measured')

% C = [1 0 0;0 1 0; 0 0 1; 1 1 1];
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
gd_predicted_y = analyze_and_visualize_comparison_withSLAM(SDPResult.optimalVecVar, sizeCk, nT, ...
    test_data.estimate_bias, nominalX, test_data.measured_y, test_data.SLAM_height_diff, Slam_SDP_result.sonar_height);


subplot(3, 3, 3); 

disp('SDO-GD version 2: Using batches')
gd_predicted_y_batch = analyze_and_visualize_comparison_withSLAM(SDPResult_batches.optimalVecVar, sizeCk, nT, ...
    test_data.estimate_bias, nominalX, test_data.measured_y, test_data.SLAM_height_diff,Slam_SDP_result.sonar_height,batchSize);

plotTrajectoryComparison(Slam_SDP_result.SLAM_trajectory, Slam_SDP_result.GT_trajectory)

% subplot(3, 3, 8); 
subplot('Position', [0.1, 0.1, 0.8, 0.23]);  % Custom position: [left, bottom, width, height]
plotGTComparison(Slam_SDP_result.GT_trajectory, Slam_SDP_Test.GT_trajectory)



% plotXYZ(x_simulated)
% Axes labels
% Compute correlation matrix
R_SLAM = corrcoef(Slam_SDP_result.sonar_height, test_data.SLAM_height_diff);
R_SDP_GD = corrcoef(Slam_SDP_result.sonar_height, gd_predicted_y);
R_SDP_GD_batch = corrcoef(Slam_SDP_result.sonar_height, gd_predicted_y_batch);

% The off-diagonal elements (1,2) and (2,1) contain the correlation
pearson_corr_SLAM = R_SLAM(1,2);
pearson_corr_SDP_GD = R_SDP_GD(1,2);
pearson_corr_SDP_GD_batch = R_SDP_GD_batch(1,2);

% Display the result
disp(['Pearson correlation SLAM: ', num2str(pearson_corr_SLAM)]);
disp(['Pearson correlation SDP-GD: ', num2str(pearson_corr_SDP_GD)]);
disp(['Pearson correlation SDP-GD batch: ', num2str(pearson_corr_SDP_GD_batch)]);
%% correlation matrix multiple 
clc
% close all
addpath('helpers');
clear


nT = 50;
batchSize = 1;
scaled = false;
scaleFactor = 1000;

% Initialize matrix for correlations
correlation_matrix_SLAM = zeros(3, 4);
correlation_matrix_SDP_GD = zeros(3, 4);
correlation_matrix_SDP_GD_batch = zeros(3, 4);


% Define row and column indices for correlations
row_start = 1;  % Starting index for the first row
col_start = 1;  % Starting index for the first column
row_end = 3;    % End index for rows
col_end = 4;    % End index for columns

% Counters for row and column of the correlation_matrix
matrix_row = 1;
matrix_col = 1;



bag_idTrain =-1;
bag_idTest = bag_idTrain+1;    

for i = row_start:row_end
    for j = col_start:col_end
        % Compute correlation (example shown between SLAM and SDP-GD predicted values)

        bag_idTrain =bag_idTrain+2
        bag_idTest = bag_idTrain+1

        correlation_results =analyzeCorrelations(bag_idTrain, bag_idTest, nT,  batchSize, scaled, scaleFactor);

        correlation_matrix_SLAM(i,j)= correlation_results.pearson_corr_SLAM;
        correlation_matrix_SDP_GD(i,j)= correlation_results.pearson_corr_SDP_GD;
        correlation_matrix_SDP_GD_batch(i,j)= correlation_results.pearson_corr_SDP_GD_batch;


    end
end
%%
clc
% Display the correlation matrix
disp('Correlation Matrix:');
disp(correlation_matrix_SLAM');
disp(correlation_matrix_SDP_GD');
disp(correlation_matrix_SDP_GD_batch');

%% error matrix agains others 
clc 
% clear all 


nT = 50;batchSize = 1;
scaled = false;
scaleFactor = 1000;

camera_mean = [0.08:0.02:0.3];  % Camera mean values
lidar_var = [0:0.02:0.2];       % Lidar variance values

nx = 24;
ny = 1;
nz = 24; %total bags

residual_matrix = zeros(nx, ny, nz); %x,y,z
residual_matrix_SLAM = zeros(nx, ny, nz); %x,y,z

% load('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM');

% for bag_idResult = 13:144
tic
% for bag_idResult = 13:123
for bag_idResult = 1:24
    % bag_idResult
    bag_idTrain = bag_idResult;
    residual_stats = analyze_residual_statistics(bag_idResult, nT, scaled, scaleFactor);
        
    residual_matrix(:, :,bag_idTrain) = reshape(residual_stats(:,1), 24, 1);
    residual_matrix_SLAM(:, :,bag_idTrain) = reshape(residual_stats(:,3), 24, 1);
    % for nx_i = 1:nx
    %     for ny_i = 1:ny
    % 
    %              residual_matrix(nx_i, ny_i,nz_i) = residual_stats(nx_i*ny_i,1);
    %         residual_matrix_SLAM(nx_i, ny_i,nz_i) = residual_stats(nx_i*ny_i,3);
    %     end
    % end
end

elapsetime_1 = toc
%
save('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM');
disp('Matrices saved to residual_data.mat');



% all elements
close all
% clc
% Reshape th
load('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM')
reshaped_matrix =       reshape(residual_matrix, 24, 24); % Reshape to 132x132reshaped_matrix = reshape(residual_matrix, 132, 132);
reshaped_matrix_SLAM = reshape(residual_matrix_SLAM, 24, 24); % Reshape to 132x132reshaped_matrix = reshape(residual_matrix, 132, 132);

% % Initialize reordered matrix
% reordered_matrix = zeros(size(reshaped_matrix));
% 
% % Reorder the rows based on the given logic
% for i = 1:11
%     for n = 1:12
%         % i 
%         % n
%         % d1 = (i-1)*12+n
%         d2 = (n-1)*12+i;
%         if d2 < 12
%             % reordered_matrix(:, (i-1)*12+n) = reshaped_matrix(:,(n-1)*12+i);
%             reordered_matrix((i-1)*12+n,:) = reshaped_matrix((n-1)*12+i,:);
%         end
%     end
% end
% Variables for labels (lidar_var and camera_mean) need to be defined
camera_mean = [0.08:0.02:0.3];  % Camera mean values
lidar_var = [0:0.02:0.2];

% Plot the reshaped heatmap
figure('Position', [100, 15, 1200, 800]); % [left, bottom, width, height]
subplot(1,2,1)
imagesc(reshaped_matrix);
colorbar;
title('Heatmap of Reshaped Last Element of residual\_matrix');
xlabel('Camera Mean (train)');
ylabel('Camera Mean (test)');

% Adjust X-axis ticks to match grouping of 12 elements for lidar_var
% set(gca, 'XTick', 1:1:12, 'XTickLabel', camera_mean); 
set(gca, 'XTick', 1:1:12, 'XTickLabel', [1:24]); 

% Adjust Y-axis ticks to map every mod(x,12) to an element in camera_mean
% set(gca, 'YTick', 1:1:12, 'YTickLabel', camera_mean);
set(gca, 'YTick', 1:1:12, 'YTickLabel', [1:24]);

% Reverse y-axis order to ensure proper alignment
set(gca, 'YDir', 'normal'); % Ensure the y-axis is in normal order
% 

subplot(1,2,2)
imagesc(reshaped_matrix_SLAM);
colorbar;
title('Heatmap of Reshaped Last Element of residual\_matrix');
xlabel('Camera Mean (train)');
ylabel('Camera Mean (test)');

% Adjust X-axis ticks to match grouping of 12 elements for lidar_var
% set(gca, 'XTick', 1:1:12, 'XTickLabel', camera_mean); 
set(gca, 'XTick', 1:1:12, 'XTickLabel', [13:24]); 

% Adjust Y-axis ticks to map every mod(x,12) to an element in camera_mean
% set(gca, 'YTick', 1:1:12, 'YTickLabel', camera_mean);
set(gca, 'YTick', 1:1:12, 'YTickLabel', [13:24]);


% Reverse y-axis order to ensure proper alignment
set(gca, 'YDir', 'normal'); % Ensure the y-axis is in normal order

%% Robustness agains noise. noise distribution 
close all
clc;
bag_idTrain = 13;

nT = 50;
batchSize = 1;
scaled = false;
scaleFactor = 1000;

% Calculate residual statistics
residual_stats = analyze_residual_statistics(bag_idTrain, nT, scaled, scaleFactor);

% Extract overall mean and variance from residual_stats for GD residuals
means_gd = residual_stats(:, 1);    % Overall means for GD residuals
vars_gd = residual_stats(:, 2); 	% Overall variances for GD residuals
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



function residual_stats = analyze_residual_statistics(bag_idTrain, nT, scaled, scaleFactor)

    filenameSDPTrain = getFileNameSDPResult(bag_idTrain, nT);
    SDPResult = load(filenameSDPTrain);
    optimalVecVar = SDPResult.optimalVecVar;
    % filenameLiftedModel = getFileNameLiftedModel(bag_idResult, nT, scaled, scaleFactor);
    % [train_camera_mean, train_camera_variance, train_lidar_mean, train_lidar_variance] = extractMeanVariance(filenameLiftedModel);


    % if bag_idTrain > 24
    %     over_id = mod((bag_idTrain - 24), 12);
    %     Slam_SDP_Train = load(getFileNameSLAM(12 + over_id, nT));
    % else
    %     filenameSLAM = getFileNameSLAM(bag_idTrain, nT);
    %     Slam_SDP_Train = load(filenameSLAM);
    % end\
    filenameSLAM = getFileNameSLAM(bag_idTrain, nT);
    Slam_SDP_Train = load(filenameSLAM);

    totalBags = 24; % Define the total number of bags
    % Initialize matrix to store results (rows for bags, columns for mean and variance)
    residual_stats = zeros(totalBags, 4); % 4 columns: overall mean/variance for gd_predicted_y and height_bySLAM
    C = [1 0 0;0 1 0; 0 0 1; 1 1 1];
    sizeCk = size(C);

    % for bag_idTest = 13:totalBags+12
    for bag_idTest = 1:24
        % bag_idTest
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
        [optimal_vec_theta, gd_predicted_y, residuals_gd] = reconstruct_and_evaluate(...
            optimalVecVar, sizeCk, nT, test_data.estimate_bias, nominalX, measured_y);
  

        residuals_SLAM = measured_y - height_bySLAM;
        % % % Compute residuals for gd_predicted_y and height_bySLAM
        % residuals_squared_gd = residuals; % Squared residuals from gd_predicted_y
        % residuals_height_bySLAM = residuals_SLAM; % Squared residuals from height_bySLAM

        % Calculate starting index (25% onwards)
        signal_length = length(measured_y);
        start_index = ceil(0 * signal_length);  % Start from 25% onwards
        if start_index < 2  % Ensure the starting index is >= 2 for very short signals
            start_index = 1;
        end
        
        % Compute squared residuals from 25% onwards
        residuals_squared_gd =      residuals_gd(start_index:end).^2; % GD residuals
        residuals_height_bySLAM =   residuals_SLAM(start_index:end).^2; % SLAM residuals
        


        % Calculate overall mean and variance for both sets of residuals        
        overall_mean_gd = mean(residuals_squared_gd);
        overall_var_gd =   var(residuals_squared_gd);

        overall_mean_height =   mean(residuals_height_bySLAM);
        overall_var_height =     var(residuals_height_bySLAM);

        % Store overall means and variances in the result matrix
        residual_stats(bag_idTest, 1) = overall_mean_gd; % Mean for gd_predicted_y
        residual_stats(bag_idTest, 2) = overall_var_gd;  % Variance for gd_predicted_y
        residual_stats(bag_idTest, 3) = overall_mean_height; % Mean for height_bySLAM
        residual_stats(bag_idTest, 4) = overall_var_height;  % Variance for height_bySLAM
    end

    % % Display the result matrix to the user
    % disp('Residual Statistics (Overall Mean and Variance):');
    % disp('Columns: [Overall_Mean_GD, Overall_Var_GD, Overall_Mean_Height, Overall_Var_Height]');
    % disp(residual_stats);
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
    plot(GT_Result(:, 1), GT_Result(:, 2), 'LineStyle', '-', 'DisplayName', 'Ground trajectory for estimating C');
    hold on;
    
    % Plot Ground Truth trajectory
    plot(GT_Test(:, 1), GT_Test(:, 2), 'LineStyle', '--', 'DisplayName', 'Ground trajectory for testing C');
    
    % Add titles, labels, and grid
    title('XY-Axis Comparison ground truth test and results trajectory');
    xlabel('X [m]');
    ylabel('Y [m]');
    legend('show');
    grid on;
end

function gd_predicted_y = analyze_and_visualize_comparison_withSLAM(optimalVecVar, sizeCk, nT, ...
    estimate_bias, nominalX, measured_y, SLAM_height_diff, sonar_height, varargin)

    % Check if batchSize is provided as an optional argument
    if ~isempty(varargin)
        batchSize = varargin{1};  % The first optional argument
        titleI = ['Measured vs. SDP-GD vs ORB-SLAM2 in batches, nT = ', num2str(nT), ' batchSize = ',  num2str(batchSize)];
    else
        batchSize = [];  % Set a default value if not provided
        titleI = ['Measured vs. SDP-GD vs ORB-SLAM2, nT = ', num2str(nT)];
    end

    % Reconstruct and evaluate predicted outputs
    [~, gd_predicted_y, residuals_gd] = reconstruct_and_evaluate(... 
        optimalVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y);

    %     gd_predicted
    % gd_predicted =reshape(gd_predicted, 4, 20);
    % gd_predicted_y = gd_predicted(4,:);
    % nominalX_reshape = reshape(nominalX, 3, 20);
    % 
    % figure()
    % plot(gd_predicted(1,:),gd_predicted(3,:))
    % hold on
    % plot(nominalX_reshape(1,:),nominalX_reshape(3,:))
    
    % Calculate Mean Squared Error (MSE)
    mse_gd = mean((residuals_gd).^2);
    mse_slam = mean((measured_y- SLAM_height_diff ).^2);

    % Calculate Variance
    var_gd = var(residuals_gd.^2);
    var_slam = var((measured_y- SLAM_height_diff).^2);

    % Display MSE and Variance
    fprintf('MSE between SDP-GD and Measured: %.4f\n', mse_gd);
    fprintf('Variance between SDP-GD and Measured: %.4f\n', var_gd);
    fprintf('MSE between SLAM and Measured: %.4f\n', mse_slam);
    fprintf('Variance between SLAM and Measured: %.4f\n', var_slam);
    disp('----------------------------------------------------------')

    % Visualization: Measured vs. Predicted Outputs
    % figure;
    % plot(SLAM_height_diff, 'DisplayName', 'SLAM seabed profile');
    % hold on;
    % plot(sonar_height, 'DisplayName', 'Sonar/Ground truth seabed profile');
    % plot(gd_predicted_y, '*-', 'DisplayName', 'SDP-GD Predicted seabed profile');
    % xlabel('Time Step');
    % ylabel('Output');
    % title(titleI);
    % legend;
    % grid on;
    
    
    plot((sonar_height-SLAM_height_diff).^2, 'DisplayName', 'Squared error SLAM seabed profile');
    hold on;
    plot((sonar_height-gd_predicted_y).^2, 'DisplayName', 'Squared error SDP-GD seabed profile');
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








function correlation_results = analyzeCorrelations(bag_idTrain, bag_idTest, nT,  batchSize, scaled, scaleFactor)
    % analyzeCorrelations: Computes correlations and displays results.
    %
    % Inputs:
    %   bag_idTrain, bag_idTest: IDs of training and test datasets
    %   nT: Time horizon
    %   scaled: Boolean flag for scaling
    %   scaleFactor: Scaling factor
    %   batchSize: Batch size for batch processing
    %   SDPResult, SDPResult_batches: Data structures containing optimization results
    %   Slam_SDP_result: SLAM result data structure
    %   test_data, train_data: Structures with test and train data
    %
    % Output:
    %   correlation_results: Struct containing Pearson correlation coefficients


    % getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor)
    % getFileNameLiftedModel(bag_idTrain, nT, scaled, scaleFactor)
    % 
    test_data = load(getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor));
    train_data = load(getFileNameLiftedModel(bag_idTrain, nT, scaled, scaleFactor));


    Slam_SDP_result = load(getFileNameSLAM(bag_idTrain, nT));
    Slam_SDP_Test = load(getFileNameSLAM(bag_idTest, nT));
    SDPResult = load(getFileNameSDPResult(bag_idTrain, nT));
    SDPResult_batches= load(getFileNameSDPResult_batches(bag_idTrain, nT, batchSize));

    % Compute nominal trajectory
    C = [1 1 1];
    nominalX = test_data.lifted_A * test_data.lifted_u;
    sizeCk = size(C);

    % Analyze data with SDP-GD
    gd_predicted_y = analyze_and_visualize_comparison_withSLAM(SDPResult.optimalVecVar, sizeCk, nT, ...
        test_data.estimate_bias, nominalX, test_data.measured_y, test_data.SLAM_height_diff, Slam_SDP_result.sonar_height);

    % Analyze data with SDP-GD using batches
    gd_predicted_y_batch = analyze_and_visualize_comparison_withSLAM(SDPResult_batches.optimalVecVar, sizeCk, nT, ...
        test_data.estimate_bias, nominalX, test_data.measured_y, test_data.SLAM_height_diff, Slam_SDP_result.sonar_height, batchSize);

    % Compute correlation coefficients
    R_SLAM = corrcoef(Slam_SDP_result.sonar_height, test_data.SLAM_height_diff);
    R_SDP_GD = corrcoef(Slam_SDP_result.sonar_height, gd_predicted_y);
    R_SDP_GD_batch = corrcoef(Slam_SDP_result.sonar_height, gd_predicted_y_batch);

    % Extract Pearson correlations
    pearson_corr_SLAM = R_SLAM(1, 2);
    pearson_corr_SDP_GD = R_SDP_GD(1, 2);
    pearson_corr_SDP_GD_batch = R_SDP_GD_batch(1, 2);

    % Display results
    disp(['Pearson correlation SLAM: ', num2str(pearson_corr_SLAM)]);
    disp(['Pearson correlation SDP-GD: ', num2str(pearson_corr_SDP_GD)]);
    disp(['Pearson correlation SDP-GD batch: ', num2str(pearson_corr_SDP_GD_batch)]);

    % Store results in a struct
    correlation_results = struct(...
        'pearson_corr_SLAM', pearson_corr_SLAM, ...
        'pearson_corr_SDP_GD', pearson_corr_SDP_GD, ...
        'pearson_corr_SDP_GD_batch', pearson_corr_SDP_GD_batch);
end
