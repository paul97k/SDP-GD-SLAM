
clear all
% close all
clc
addpath('helpers');
C = [1 1 1];
nT = 50;
scaleFactor = 1000;


bag_names = get_bag_name();

%
% for bag_id=94:144
% for bag_id=1:5
% parfor bag_id=1:length(bag_names)
for bag_id=1:1
% for bag_id=24:144   
    bag_id
    
    % stabdard method
    % batchSize = 1;
    % bag_id = 13;
    % nT = 100;
    scaleFactor = 1000;
    scaled = false;
    filename = getFileNameLiftedModel(bag_id, nT, scaled, scaleFactor);
    filenameObj = load(filename);
    Slam_SDP_Train = load(getFileNameSLAM(bag_id, nT));
    
    nominalVecC = repmat(C, 1, nT); 
    nominalVecC = reshape(nominalVecC', [], 1);
    nominalVecC = double(nominalVecC);
    
    
    
    estimateBias = false;
    sigmaV = filenameObj.sigma_v;
    sigmaX= filenameObj.sigma_wx;
    nominalX= filenameObj.lifted_A*filenameObj.lifted_u; 
    measuredY= filenameObj.measured_y; % equal to filenameObj.SLAM_height_diff
    measuredY= Slam_SDP_Train.sonar_height; 

    noiseStdDev = 0.2; % standard deviation of the Gaussian noise (adjust as needed)
    measuredY = measuredY + noiseStdDev * randn(size(measuredY));



    sigmaC= filenameObj.sigma_w_theta; %sigmaw_theta
    sizeCk= size(C); 
    desiredSnr= 30; 
    matrixScale= 1; 
    trueVecC= [] ; 
    measuredY = double(measuredY);

    
    processSDPResults(bag_id, nT, estimateBias, sigmaV, sigmaX, nominalX, measuredY, ...
        nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC, ...
        measuredY, filenameObj.sigma_v, filenameObj.sigma_wx, filenameObj.mu_theta, filenameObj.sigma_w_theta, @map_cost_func);

    
    batchSize = 1;
    
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
    
    % Inside your parfor or regular loop
    saveSDPBatchedResult(bag_id, nT, batchSize, elapsedTime, ...
        initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output);
    
    disp('--------------------------------------')

end


%% Plot results using saved output map
clc
close all
addpath('helpers');
clear
% 1 9 17

bag_names = get_bag_name();

% for bag_idTrain = 1:length(bag_names)
bagtrain = 8
for bag_idTrain = bagtrain:bagtrain
    % bag_idTrain =4;          %camera_0.3_0.1_lidar_0_0.20
    % bag_idTrain =bag_idTrain+8;          %camera_0.3_0.1_lidar_0_0.20
    % bag_idTrain =bag_idTrain+8;          %camera_0.3_0.1_lidar_0_0.20
    % bag_idTest = bag_idTrain+1;    %camera_0.18_0.1_lidar_0_0.02'
    
    
    
    
        bag_name = bag_names{bag_idTrain};
        prefix = extractBefore(bag_name, '-6');
         matches = contains(bag_names, prefix) & ~strcmp(bag_names, bag_name);
         % matches = contains(bag_names, prefix) ;
        matching_indices = find(matches)
        
    [~, idx] = min(abs(matching_indices - bag_idTrain));
    bag_idTest = matching_indices(idx);
    
    % bag_idTest = 10;    
    
    % 0.1969    3.2288    2.6625    2.9387    3.5695
    nT = 50;
    batchSize = 1;
    scaled = false;
    scaleFactor = 1000;
    
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
    getFileNameSDPResult(bag_idTrain, nT)
    getFileNameSDPResult_batches(bag_idTrain, nT, batchSize)
    
    Slam_SDP_Train = load(getFileNameSLAM(bag_idTrain, nT));
    Slam_SDP_Test = load(getFileNameSLAM(bag_idTest, nT));
    SDPResult = load(getFileNameSDPResult(bag_idTrain, nT));
    SDPResult_batches= load(getFileNameSDPResult_batches(bag_idTrain, nT, batchSize));
    
    
    
    C = [1 1 1];
    nominalX= test_data.lifted_A*test_data.lifted_u; 
    sizeCk= size(C);
      
    
    SLAM_height_diff= test_data.SLAM_height_diff;
    sonar_height_train =  Slam_SDP_Train.sonar_height;
    GroundTruthHeight =  Slam_SDP_Test.sonar_height;
    sonar_height_test = calculate_height_difference(Slam_SDP_Train.slam_map, Slam_SDP_Test.SLAM_trajectory);
    % 
    % function gd_predicted_y = analyze_and_visualize_comparison_withSLAM(optimalVecVar, sizeCk, nT, ...
    % estimate_bias, nominalX, measured_y, SLAM_height_diff, sonar_height, varargin)

    gd_predicted_y = analyze_and_visualize_comparison_withSLAM(SDPResult.optimalVecVar, sizeCk, nT, ...
    test_data.estimate_bias, nominalX,GroundTruthHeight, sonar_height_test, sonar_height_train);
    
    
    % disp('SDO-GD version 2: Using batches')
    % gd_predicted_y_batch = analyze_and_visualize_comparison_withSLAM(SDPResult_batches.optimalVecVar, sizeCk, nT, ...
    % test_data.estimate_bias, nominalX, GroundTruthHeight,sonar_height_test, sonar_height_train,batchSize);
    
    
    
    
    plotfigures = true;
    % plotfigures = false;
    
    % % 
    % height_bySLAM = calculate_height_difference(Slam_SDP_result.slam_map, Slam_SDP_result.SLAM_trajectory);
    if plotfigures
        figure('Position', [450, 200, 1000, 800]);
        subplot(3, 3, 1); 
        
        plot(Slam_SDP_Test.sonar_height, '*')
        hold on
        % plot(train_data.measured_y)
        
        plot(Slam_SDP_Train.sonar_height)
        % plot(sonar_height_test)
        legend('Test data', 'Train data')
        title('Trained: Height by slam vs measured')
        
        % C = [1 0 0;0 1 0; 0 0 1; 1 1 1];
    
        subplot(3, 3, [2,3]); 
        disp('SDO-GD version 1')
    
        plot((GroundTruthHeight-SLAM_height_diff).^2, 'DisplayName', 'Orb-SLam2');
        hold on;
        plot((GroundTruthHeight-gd_predicted_y).^2, 'DisplayName', 'SDP-GD');
        % plot((sonar_height_train-gd_predicted_y_batch).^2, 'DisplayName', 'SDP-GD Batches');
        title('Square root error ORB-SLAM2 vs SDP-GD');
        ylim([0,6])
        legend;
        grid on;
        
        
        
        
        
        
        
        plotTrajectoryComparison(Slam_SDP_Train.SLAM_trajectory, Slam_SDP_Train.GT_trajectory)
        
        % subplot(3, 3, 8); 
        subplot('Position', [0.1, 0.1, 0.8, 0.23]);  % Custom position: [left, bottom, width, height]
        % plotGTComparison(Slam_SDP_result.GT_trajectory, Slam_SDP_Test.GT_trajectory)
        % plotGTComparison(Slam_SDP_result.GT_trajectory, Slam_SDP_Test.GT_trajectory)
        plot(gd_predicted_y)
        hold on
        % plot(gd_predicted_y_batch)
        plot(sonar_height_test)
        plot(GroundTruthHeight)
        legend('SDP-GD', 'ORB-SLAM2', 'Ground truth (Sonar)')
        xlabel('Index');
        ylabel('Depth [m]');
    
    
    end 
    % plotXYZ(x_simulated)
    % Axes labels
    % Compute correlation matrix
    R_SLAM =   corrcoef(GroundTruthHeight(10:end,:), sonar_height_test(10:end,:));
    R_SDP_GD = corrcoef(GroundTruthHeight(10:end,:), gd_predicted_y(10:end,:));
    % R_SDP_GD_batch = corrcoef(GroundTruthHeight, gd_predicted_y_batch);
    
    % The off-diagonal elements (1,2) and (2,1) contain the correlation
    pearson_corr_SLAM = R_SLAM(1,2)^2;
    pearson_corr_SDP_GD = R_SDP_GD(1,2)^2;
    % pearson_corr_SDP_GD_batch = R_SDP_GD_batch(1,2);

   windowSize = 10;  % for example
    n = length(GroundTruthHeight);
    rollingCorrSLAM = zeros(n,1);
    rollingCorrSDP = zeros(n,1);
    
    for i = 1:(n - windowSize + 1)
        x_window = GroundTruthHeight(i:i+windowSize-1);
        y_window = sonar_height_test(i:i+windowSize-1);
        y_window2 = gd_predicted_y(i:i+windowSize-1);
        rollingCorrSLAM(i + floor(windowSize/2)) = corr(x_window(:), y_window(:));
        rollingCorrSDP(i + floor(windowSize/2)) = corr(x_window(:), y_window2(:));
    end
    % [rollingCorr rollingCorr2]
    mean([rollingCorrSLAM rollingCorrSDP])
    
    % Display the result

    disp(['Pearson correlation SDP-GD: ', num2str(pearson_corr_SDP_GD)]);
    disp(['Pearson correlation SLAM: ', num2str(pearson_corr_SLAM)]);
    % disp(['Pearson correlation SDP-GD batch: ', num2str(pearson_corr_SDP_GD_batch)]);




end












%% error matrix agains others 
% clc 


nT = 50;batchSize = 1;
scaled = false;
scaleFactor = 1000;

camera_mean = [0.08:0.02:0.3];  % Camera mean values
lidar_var = [0:0.02:0.2];       % Lidar variance values

nx = 24;
ny = 1;
nz = 24; %total bags

residual_matrix = zeros(nx, nz); %x,y,z
residual_matrix_SLAM = zeros(nx, nz); %x,y,z

r2_matrix = zeros(nx, nz); %x,y,z
r2_matrix_SLAM = zeros(nx, nz); %x,y,z

% load('residual_data.mat', 'residual_matrix', 'residual_matrix_SLAM');

% for bag_idResult = 13:144
tic
% for bag_idResult = 13:123
for bag_idResult = 1:24
    % bag_idResult
    bag_idTrain = bag_idResult;
    % disp('...........')
    residual_stats = analyze_residual_statistics(bag_idResult, nT, scaled, scaleFactor);
        
    residual_matrix(:, bag_idTrain) = reshape(residual_stats(:,1), 24, 1);
    residual_matrix_SLAM(:, bag_idTrain) = reshape(residual_stats(:,3), 24, 1);

    r2_matrix(:, bag_idTrain) = reshape(residual_stats(:,5), 24, 1);
    r2_matrix_SLAM(:, bag_idTrain) = reshape(residual_stats(:,6), 24, 1);
end

% residual_matrix
desired_rows = 8;
nz = 3;
residual_matrix(residual_matrix == 0) = [];
residual_matrix_SLAM(residual_matrix_SLAM == 0) = [];
r2_matrix(r2_matrix == 0) = [];
r2_matrix_SLAM(r2_matrix_SLAM == 0) = [];

if mod(numel(residual_matrix), desired_rows) == 0 && numel(residual_matrix)/desired_rows == nz
    reshaped_matrix = reshape(residual_matrix, desired_rows, nz)';
    reshaped_matrix_SLAM = reshape(residual_matrix_SLAM, desired_rows, nz)';
    reshaped_matrix_r2 = reshape(r2_matrix, desired_rows, nz)';
    reshaped_matrix_r2_SLAM = reshape(r2_matrix_SLAM, desired_rows, nz)';
else
    error('The number of non-zero elements is not compatible with a size of 5 x %d', nz);
end
reshaped_matrix 
reshaped_matrix_SLAM 
reshaped_matrix_r2 
reshaped_matrix_r2_SLAM 
% Reshape each row into pairs of 2 elements and take the mean across each pair
reshaped_matrix_avg = mean(reshape(reshaped_matrix.', 2, []), 1);  % Average in pairs (column-wise)
reshaped_matrix_avg_SLAM  = mean(reshape(reshaped_matrix_SLAM .', 2, []), 1);  % Average in pairs (column-wise)
reshaped_matrix_avg_r2  = mean(reshape(reshaped_matrix_r2 .', 2, []), 1);  % Average in pairs (column-wise)
reshaped_matrix_avg_r2_SLAM  = mean(reshape(reshaped_matrix_r2_SLAM.', 2, []), 1);  % Average in pairs (column-wise)



reshaped_matrix_avg = reshape(reshaped_matrix_avg, 4, []).' ;    % Reshape to 3x4
reshaped_matrix_avg_SLAM = reshape(reshaped_matrix_avg_SLAM, 4, []).' ;    % Reshape to 3x4
reshaped_matrix_avg_r2 = reshape(reshaped_matrix_avg_r2, 4, []).' ;    % Reshape to 3x4
reshaped_matrix_avg_r2_SLAM = reshape(reshaped_matrix_avg_r2_SLAM, 4, []).' ;    % Reshape to 3x4

rowNames = {'C0', 'C1', 'C3'};
columnNames = {'traj1', 'traj2', 'traj3', 'traj4'};

% Create table
T = array2table(reshaped_matrix_avg, 'VariableNames', columnNames, 'RowNames', rowNames);
T_SLAM = array2table(reshaped_matrix_avg_SLAM, 'VariableNames', columnNames, 'RowNames', rowNames);
T_r2 = array2table(reshaped_matrix_avg_r2, 'VariableNames', columnNames, 'RowNames', rowNames);
T_r2_SLAM = array2table(reshaped_matrix_avg_r2_SLAM, 'VariableNames', columnNames, 'RowNames', rowNames);

% Display
disp('MSE SDP')
disp(T)
disp('MSE OrbSLAM2')
disp(T_SLAM)
disp('Pearson Correlation SDP')
disp(T_r2)
disp('Pearson Correlation OrbSLAM2')
disp(T_r2_SLAM)


%% check exit flag
clc
clear
addpath('helpers');
nT = 50;
batchSize = 1;
% get SDP_result standard method


bag_names = get_bag_name();

% bag_name = bag_names{bag_idTrain};
% prefix = extractBefore(bag_name, '-6');
% matches = contains(bag_names, prefix) & ~strcmp(bag_names, bag_name);
% % matches = contains(bag_names, prefix) ;
% matching_indices = find(matches)



computational_info = zeros(length(bag_names),3); 
computational_infoBatches = zeros(length(bag_names),3);


for bag_idTrain = 1:length(bag_names)
% for bag_idTrain = 1:1
% bag_idTrain = 9
    filenameSDPResult = getFileNameSDPResult(bag_idTrain, nT);
    SDPResult = load(filenameSDPResult);
    
    
    filenameSDPResult_batches = getFileNameSDPResult_batches(bag_idTrain, nT, batchSize);
    SDPResult_batches = load(filenameSDPResult_batches);
    
    % Print the exitFlag for SDPResult_batcehes
    % disp([num2str(bag_idTrain), ': '])
    % disp(['Computation_time', ': ', num2str(SDPResult.Computation_time),',' num2str(SDPResult_batches.Computation_time)]);
    % disp(['optimalValueCost', ': ', num2str(SDPResult.optimalValueCost),',' num2str(SDPResult_batches.optimalValueCost)]);
    % disp(['iterations', ': ', num2str(SDPResult.output.iterations),',' num2str(SDPResult_batches.output.iterations)]);
    % disp(' ')

    computational_info(bag_idTrain,:) = [SDPResult.Computation_time SDPResult.optimalValueCost SDPResult.output.iterations];
    computational_infoBatches(bag_idTrain,:) = [str2double(SDPResult_batches.Computation_time) SDPResult_batches.optimalValueCost SDPResult_batches.output.iterations];
end
computational_info= computational_info'; 
computational_infoBatches= computational_infoBatches'; 

[m, n] = size(computational_info);

% Reshape each row into groups of 2 columns and take the mean
info_reshaped = reshape(computational_info, m, 2, []);
info_reshaped_Batches = reshape(computational_infoBatches, m, 2, []);
info_mean = squeeze(mean(info_reshaped, 2));  % Result is 3x12
info_mean_Batches = squeeze(mean(info_reshaped_Batches, 2));  % Result is 3x12



rowNames = {'C0 traj1', 'C0 traj2', 'C0 traj3', 'C0 traj4','C1 traj1', 'C1 traj2', 'C1 traj3', 'C1 traj4','C2 traj1', 'C2 traj2', 'C2 traj3', 'C2 traj4'};
columnNames = {'Computation time', 'optimalValueCost', 'Iterations', 'Batches Computation time', 'Batches optimalValueCost', 'Batches Iterations'};

% Create table
T = array2table([info_mean' info_mean_Batches'], 'VariableNames', columnNames, 'RowNames', rowNames);

% Display
disp(T)
% Desired row order: group all traj1, traj2, ...
reordered_rows = {'C0 traj1', 'C1 traj1', 'C2 traj1', ...
                  'C0 traj2', 'C1 traj2', 'C2 traj2', ...
                  'C0 traj3', 'C1 traj3', 'C2 traj3', ...
                  'C0 traj4', 'C1 traj4', 'C2 traj4'};

% Reorder table
T = T(reordered_rows, :);

% Display
disp(T)
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

    filenameSLAM = getFileNameSLAM(bag_idTrain, nT);
    Slam_SDP_Train = load(filenameSLAM);

    totalBags = 24; % Define the total number of bags
    residual_stats = zeros(totalBags, 6); % 4 columns: overall mean/variance for gd_predicted_y and height_bySLAM
    C = [1 1 1];
    sizeCk = size(C);


    bag_names = get_bag_name();

    bag_name = bag_names{bag_idTrain};
    prefix = extractBefore(bag_name, '-6');
     matches = contains(bag_names, prefix) & ~strcmp(bag_names, bag_name);
    matching_indices = find(matches);

    [~, idx] = min(abs(matching_indices - bag_idTrain));
    bag_idTest = matching_indices(idx);

    % bag_idTrain
    % bag_idTest

    % % for bag_idTest = 13:totalBags+12
    % for i = 1:length(matching_indices)

        % bag_idTest = matching_indices(i);
        filenameLiftedModel = getFileNameLiftedModel(bag_idTest, nT, scaled, scaleFactor);
        
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
        measured_y = Slam_SDP_Test.sonar_height; % Ground truth
        
        % Nominal trajectory
        nominalX = test_data.lifted_A * test_data.lifted_u;

        % Reconstruct and evaluate predictions
        [optimal_vec_theta, gd_predicted_y, residuals_gd] = reconstruct_and_evaluate(...
            optimalVecVar, sizeCk, nT, test_data.estimate_bias, nominalX, measured_y);
  

        residuals_SLAM = Slam_SDP_Test.sonar_height - height_bySLAM;
        
        % % Calculate starting index (25% onwards)
        % signal_length = length(measured_y);
        % start_index = ceil(0 * signal_length);  % Start from 25% onwards
        % if start_index < 2  % Ensure the starting index is >= 2 for very short signals
        %     start_index = 1;
        % end
        start_index = 1;
        % Compute squared residuals from 25% onwards
        residuals_squared_gd =      residuals_gd(start_index:end).^2; % GD residuals
        residuals_height_bySLAM =   residuals_SLAM(start_index:end).^2; % SLAM residuals
        


        % Calculate overall mean and variance for both sets of residuals        
        overall_mean_gd = mean(residuals_squared_gd);
        overall_var_gd =   var(residuals_squared_gd);

        overall_mean_height =   mean(residuals_height_bySLAM);
        overall_var_height =     var(residuals_height_bySLAM);

        % Store overall means and variances in the result matrix
        residual_stats(bag_idTest, 1) = overall_mean_gd; % Mean squared for gd_predicted_y
        residual_stats(bag_idTest, 2) = overall_var_gd;  % Variance for gd_predicted_y
        residual_stats(bag_idTest, 3) = overall_mean_height; % Mean for height_bySLAM
        residual_stats(bag_idTest, 4) = overall_var_height;  % Variance for height_bySLAM


        R_SLAM = corrcoef(Slam_SDP_Test.sonar_height,  height_bySLAM);
        R_SDP_GD = corrcoef(Slam_SDP_Test.sonar_height, gd_predicted_y);
        % R_SDP_GD_batch = corrcoef(Slam_SDP_Test.sonar_height, gd_predicted_y_batch);
        
        % The off-diagonal elements (1,2) and (2,1) contain the correlation
        pearson_corr_SLAM = R_SLAM(1,2);
        pearson_corr_SDP_GD = R_SDP_GD(1,2);
        % pearson_corr_SDP_GD_batch = R_SDP_GD_batch(1,2);
        

        residual_stats(bag_idTest, 5) = pearson_corr_SDP_GD; % Mean for height_bySLAM
        residual_stats(bag_idTest, 6) = pearson_corr_SLAM;  % Variance for height_bySLAM
        % Display the result
        % disp(['Pearson correlation SLAM: ', num2str(pearson_corr_SLAM)]);
        % disp(['Pearson correlation SDP-GD: ', num2str(pearson_corr_SDP_GD)]);
        % disp(['Pearson correlation SDP-GD batch: ', num2str(pearson_corr_SDP_GD_batch)]);
        
        


    % end

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
        ylabel('Amplitude [m]');
        legend('show');
        grid on;
        
    end
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

    grad_gd = gradient(residuals_gd);
    grad_slam = gradient(measured_y - SLAM_height_diff);
    
    % You can then compute:
    mean_grad_gd = mean(abs(grad_gd));
    mean_grad_slam = mean(abs(grad_slam));
    
    % or the variance:
    var_grad_gd = var(grad_gd);
    var_grad_slam = var(grad_slam);


    % Display MSE and Variance
    fprintf('MSE between SDP-GD and Measured: %.4f\n', mse_gd);
    fprintf('mean_grad SDP-GD and Measured: %.4f\n', mean_grad_gd);
    fprintf('MSE between SLAM and Measured: %.4f\n', mse_slam);
    fprintf('mean_grad SLAM and Measured: %.4f\n', mean_grad_slam);
    disp('----------------------------------------------------------')
    % 
    % % Visualization: Measured vs. Predicted Outputs
    % % figure;
    % plot(SLAM_height_diff, 'DisplayName', 'SLAM seabed profile');
    % hold on;
    % plot(sonar_height, 'DisplayName', 'Sonar/Ground truth seabed profile');
    % plot(gd_predicted_y, '*-', 'DisplayName', 'SDP-GD Predicted seabed profile');
    % xlabel('Time Step');
    % ylabel('Output');
    % title(titleI);
    % legend;
    % grid on;
    % 

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




function filename = saveSDPResultInFilename(bag_id, nT)
    bag_names = get_bag_name();
    bag_name = bag_names{bag_id};

    filename = sprintf('%s_nT_%d.mat', bag_name, nT);
    folder_path = 'SDP_result';
    
    filename = fullfile(folder_path, filename);
    
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



function saveSDPBatchedResult(bag_id, nT, batchSize, elapsedTime, ...
    initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output)

    % Construct filename from your custom function
    filenameLiftedModelBatchesResult = getFileNameSDPResult_batches(bag_id, nT, batchSize);

    % Convert computation time to string
    Computation_time = num2str(elapsedTime);
    disp(['Computation time SDP Batches: ', Computation_time, ' seconds']);

    % Pack all data into a struct (required for parfor compatibility)
    
    
    save(filenameLiftedModelBatchesResult, 'initialVecVar', 'optimalVecVar', 'optimalValueCost', 'exitFlag', 'output', 'Computation_time');
    disp(['SDP-GD using batches saved to ', filenameLiftedModelBatchesResult]);

    disp(['SDP-GD using batches saved to ', filenameLiftedModelBatchesResult]);
end
