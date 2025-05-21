clc
clear
% Specify the disturbance level to average over
DisturbanceArray = [0 1 2 3 4 6 7 8 10 12];  % or whatever your disturbance level is
for i=1:length(DisturbanceArray)
    Disturbance = DisturbanceArray(i);

    % Create a pattern that matches all test files for that disturbance
    FolderPath = "C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code\Result_section\";
    addpath(FolderPath) 
    filePattern = "Disturbance_" + Disturbance + "_Test_*.mat";
    % filePattern = "Noise_Disturbance_" + Disturbance + "_Test_1.mat";
    
    filePath = FolderPath+filePattern;
    
    % Get the list of files that match the pattern
    files = dir(filePath);
    
    % Preallocate arrays for the metrics (using one metric as an example)
    numFiles = numel(files);
    mse_x_all = zeros(numFiles, 1);
    rmse_x_all = zeros(numFiles, 1);
    mse_y_all = zeros(numFiles, 1);
    rmse_y_all = zeros(numFiles, 1);
    mse_depth_all = zeros(numFiles, 1);
    rmse_depth_all = zeros(numFiles, 1);
    R2_all = zeros(numFiles, 1);
    mse_x_kalman_all = zeros(numFiles, 1);
    rmse_x_kalman_all = zeros(numFiles, 1);
    mse_y_kalman_all = zeros(numFiles, 1);
    rmse_y_kalman_all = zeros(numFiles, 1);
    mse_depth_kalman_all = zeros(numFiles, 1);
    rmse_depth_kalman_all = zeros(numFiles, 1);
    R2_kalman_all = zeros(numFiles, 1);
    
    if numFiles ==0
        continue
    end
    % Loop through each file and load the metrics
    for i = 1:numFiles
        data = load(files(i).name);
        
        mse_x_all(i)           = data.mse_x;
        rmse_x_all(i)          = data.rmse_x;
        mse_y_all(i)           = data.mse_y;
        rmse_y_all(i)          = data.rmse_y;
        mse_depth_all(i)       = data.mse_depth;
        rmse_depth_all(i)      = data.rmse_depth;
        R2_all(i)              = 1-data.R2;
        mse_x_kalman_all(i)    = data.mse_x_kalman;
        rmse_x_kalman_all(i)   = data.rmse_x_kalman;
        mse_y_kalman_all(i)    = data.mse_y_kalman;
        rmse_y_kalman_all(i)   = data.rmse_y_kalman;
        mse_depth_kalman_all(i)= data.mse_depth_kalman;
        rmse_depth_kalman_all(i)= data.rmse_depth_kalman;
        R2_kalman_all(i)       = 1-data.R2_kalman;
    end
    
    % Calculate the average for each metric
    avg_mse_x           = mean(mse_x_all);
    avg_rmse_x          = mean(rmse_x_all);
    avg_mse_y           = mean(mse_y_all);
    avg_rmse_y          = mean(rmse_y_all);
    avg_mse_depth       = mean(mse_depth_all);
    avg_rmse_depth      = mean(rmse_depth_all);
    avg_R2              = mean(R2_all);
    avg_mse_x_kalman    = mean(mse_x_kalman_all);
    avg_rmse_x_kalman   = mean(rmse_x_kalman_all);
    avg_mse_y_kalman    = mean(mse_y_kalman_all);
    avg_rmse_y_kalman   = mean(rmse_y_kalman_all);
    avg_mse_depth_kalman= mean(mse_depth_kalman_all);
    avg_rmse_depth_kalman= mean(rmse_depth_kalman_all);
    avg_R2_kalman       = mean(R2_kalman_all);
    
    % Display the average metrics
    fprintf('\nAverages for Disturbance Level: %d\n', Disturbance);
    fprintf('%-20s | %-15s | %-15s\n', 'Metric', 'SDP-GD', 'Sonar SLAM');
    fprintf('%s\n', repmat('-', 1, 55));
    fprintf('%-20s | %.10f m       | %.10f m\n', 'MSE X', avg_mse_x, avg_mse_x_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'RMSE X', avg_rmse_x, avg_rmse_x_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'MSE Y', avg_mse_y, avg_mse_y_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'RMSE Y', avg_rmse_y, avg_rmse_y_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'MSE Depth', avg_mse_depth, avg_mse_depth_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'RMSE Depth', avg_rmse_depth, avg_rmse_depth_kalman);
    fprintf('%-20s | %.10f         | %.10f\n', '1-R²', avg_R2, avg_R2_kalman);
end