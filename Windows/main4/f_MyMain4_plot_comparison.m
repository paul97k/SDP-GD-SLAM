
function f_MyMain4_plot_comparison( ...
    measurementsStruct, groundTruthStruct, SDP_predictedStruct, ...
    KalmanStruct, OutputMapsDataFileName, TestDataFileName, ...
    DisturbanceLevel, itt, plot1,plot2)



    % [mainFreqs, magnitudes, f, Z_mag]= extractMainHarmonics( ...
    % groundTruthStruct.timestampsUn, ...
    % groundTruthStruct.z_positionsUn);
    % % weightedFreq = sum(mainFreqs .* magnitudes) / sum(magnitudes)
    % totalEnergy = sum(magnitudes);
    % envelopeArea = trapz(f, Z_mag);
    % envStd = std(Z_mag);
    % 
    % % % Fit exponential decay to envelope
    % fitIdx = f > 0 & f < 3;  % Restrict range for fitting
    % p = polyfit(f(fitIdx), log(Z_mag(fitIdx) + eps), 1);
    % decayRate = -p(1);  % Higher = faster decay = more tonal


    % 
    % 
    % figure;
    % plot(f, Z_mag);
    % hold on;
    % plot(mainFreqs, magnitudes, 'rx');
    % xlabel('Frequency (Hz)');
    % ylabel('Amplitude');
    % title('FFT Magnitude Spectrum with Main Harmonics');
    % grid on;

    % [mainFreqs1, amps1] = extractMainHarmonics( ...
    % SDP_predictedStruct.timestamps, ...
    % SDP_predictedStruct.depth);
    % 
    % [mainFreqs2, amps2] = extractMainHarmonics( ...
    % KalmanStruct.timestamps, ...
    % KalmanStruct.z_depth);
    % 
    % maxFreqGT = max(mainFreqs);
    % % maxFreqGT = totalEnergy;
    % maxFreqSDP = max(mainFreqs1);
    % maxFreqKalman = max(mainFreqs2);
    maxFreqGT =1;
    maxFreqSDP = 1;
    maxFreqKalman =1;
    
    
    if plot1
        % figure
        figure('Position', [-1920, 0, 1920, 1080]); % Adjusted height for additional subplot
        % figure('Position', [-1440, 270, 960, 540]); % Adjusted height for additional subplot
        % figure
    
        subplot(2,2,1);
        hold on;
        grid on;
        plot(groundTruthStruct.timestamps, groundTruthStruct.x_positions, 'g', 'LineWidth', 2, 'DisplayName', 'Ground Truth X');
        plot(SDP_predictedStruct.timestamps, SDP_predictedStruct.x_positions , 'b.', 'LineWidth', 2, 'DisplayName', 'SDP-GD X ');
        plot(KalmanStruct.timestamps, KalmanStruct.x, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar SLAM');
        % plot(measurementsStruct.timestamps, measurementsStruct.x_positions, 'r--', 'LineWidth', 2, 'DisplayName', 'Measured');
        plot(measurementsStruct.timestamps, SDP_predictedStruct.X_norm(:,1), 'm--', 'LineWidth', 2, 'DisplayName', 'X_{norm}');
    
        xlabel('Time Step [s]', 'FontSize', 12);
        ylabel('x Distance [m]', 'FontSize', 12);
        title('x position', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
    
    
        subplot(2,2,2);
        hold on;
        grid on;
        plot(SDP_predictedStruct.timestamps, SDP_predictedStruct.y_positions , 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD y ');
        plot(groundTruthStruct.timestamps, groundTruthStruct.y_positions, 'g.', 'LineWidth', 2, 'DisplayName', 'Ground Truth y');
        plot(KalmanStruct.timestamps, KalmanStruct.y, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar SLAM');
        % plot(measurementsStruct.timestamps, measurementsStruct.y_positions, 'r--', 'LineWidth', 2, 'DisplayName', 'Measured');
        plot(measurementsStruct.timestamps, SDP_predictedStruct.X_norm(:,2), 'm--', 'LineWidth', 2, 'DisplayName', 'X_{norm}');
    
        xlabel('Time Step [s]', 'FontSize', 12);
        ylabel('x Distance [m]', 'FontSize', 12);
        title('y position', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
    
    
        subplot(2,2,[3 4]);
        hold on;
        grid on;
        plot(SDP_predictedStruct.timestamps, SDP_predictedStruct.depth , 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD depth ');
        plot(groundTruthStruct.timestampsUn, groundTruthStruct.z_positionsUn, 'g-', 'LineWidth', 2, 'DisplayName', 'Ground Truth depth');
        plot(KalmanStruct.timestamps, KalmanStruct.z_depth, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar SLAM');
        % plot(measurementsStruct.timestamps, measurementsStruct.depth, 'r--', 'LineWidth', 2, 'DisplayName', 'Measured');
        % plot(measurementsStruct.timestamps, SDP_predictedStruct.X_norm(:,3), 'm--', 'LineWidth', 2, 'DisplayName', 'X_{norm}');
    
        xlabel('Time Step [s]', 'FontSize', 12);
        ylabel('Depth Distance [m]', 'FontSize', 12);
        title('depth', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
    end

    if plot2

        figure('Position', [-1920, 0, 1920, 1080]); % Adjusted height for additional subplot
    
        % Third subplot: Measured vs Predicted Depth (Z)
        % subplot(3,3,[4,5,6,7,8,9]);
        subplot(2,2,1);
        hold on;
        grid on;
        plot(SDP_predictedStruct.x_positions , SDP_predictedStruct.depth, 'b', 'LineWidth', 2, 'DisplayName', 'SDP-GD');
        plot(groundTruthStruct.x_positions , groundTruthStruct.depth, 'g', 'LineWidth', 2, 'DisplayName', 'Groundtruth Depth');
        % plot(measurementsStruct.x_positions, measurementsStruct.depth, 'r--', 'LineWidth', 2, 'DisplayName', 'Measurement');
        plot(KalmanStruct.x, KalmanStruct.z_depth, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar Based SLAM');
        % ylim([2.5,6])
        xlabel('x Distance [m]', 'FontSize', 12);
        ylabel('z Distance [m]', 'FontSize', 12);
        title('Reconstructed XZ map usinf SDP-GD vs Sonar Based SLAM', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
    end
    % % Title = "Disturbance Level: "+ DisturbanceLevel;
    % Title = OutputMapsDataFileName + " vs "+ TestDataFileName;
    % sgtitle(Title)
    

        % Remove prefix like S0.22_D10_R0.5_
    TestDataFileNamecleanName = regexprep(TestDataFileName, '^S[\d.]+_D\d+_R[\d.]+_', '');
    TestDataFileNamecleanName = regexprep(TestDataFileName, '^S[\d.]+_D\d+_R[\d.]+_', '');
    TestDataFileNamecleanName = strrep(TestDataFileNamecleanName, '_', ' ');


    OutputMapsDataFileNamecleanName = char(regexprep(OutputMapsDataFileName, '^S[\d.]+_D\d+_R[\d.]+_', ''));
    
    % Step 2: Extract everything before and including '.nT'
    nTIndex = strfind(OutputMapsDataFileNamecleanName, '.nT');
    if ~isempty(nTIndex)
        OutputMapsDataFileNamecleanName = OutputMapsDataFileNamecleanName(1:nTIndex-1); % Include '.nT'
    end
    
    % Step 3: Replace underscores with spaces
    OutputMapsDataFileNamecleanName = strrep(OutputMapsDataFileNamecleanName, '_', ' ');
    
    Title = [['map: '  OutputMapsDataFileNamecleanName] ' vs ' TestDataFileNamecleanName];
    if plot1
        sgtitle(Title)
    end


    


    
    [~, maxIdGTKalman] = min(abs(measurementsStruct.timestamps - max(KalmanStruct.timestamps)));
    [~, maxIdGTgd] = min(abs(measurementsStruct.timestamps - max(SDP_predictedStruct.timestamps)));

    [~, maxIdGTKalmanDepth] = min(abs(measurementsStruct.timestamps - max(KalmanStruct.timestamps)));
    [~, maxIdGTgdDepth] = min(abs(measurementsStruct.timestamps - max(SDP_predictedStruct.timestamps)));




    % Error Calculations
    estimated_x = SDP_predictedStruct.x_positions';
    estimated_x_kalman = KalmanStruct.x';
    % groundtruth_x = groundTruthStruct.x_positions;
    groundtruth_x = measurementsStruct.x_positions;

    estimated_y = SDP_predictedStruct.y_positions';
    estimated_y_kalman = KalmanStruct.y';
    % groundtruth_y = groundTruthStruct.y_positions;
    groundtruth_y = measurementsStruct.y_positions;

    estimated_depth = SDP_predictedStruct.depth;
    estimated_depth_kalman = KalmanStruct.z_depth;
    % groundtruth_depth = groundTruthStruct.depth;
    groundtruth_depth = measurementsStruct.depth';



    % 
    groundtruth_depth(groundtruth_depth == Inf) = 5;
    groundtruth_depth(groundtruth_depth == -Inf) = -5;
    estimated_depth_kalman(isnan(estimated_depth_kalman)) = 5;


    
    % Compute Absolute Error (with resampling for alignment)
    error_x = abs(estimated_x' - resample(groundtruth_x(1:maxIdGTgd), length(estimated_x), length(groundtruth_x(1:maxIdGTgd))));
    error_x_kalman = abs(estimated_x_kalman - resample(groundtruth_x(1:maxIdGTKalman), length(estimated_x_kalman), length(groundtruth_x(1:maxIdGTKalman))));

    error_y = abs(estimated_y' - resample(groundtruth_y(1:maxIdGTgd), length(estimated_y), length(groundtruth_y(1:maxIdGTgd))));
    error_y_kalman = abs(estimated_y_kalman - resample(groundtruth_y(1:maxIdGTKalman), length(estimated_y_kalman), length(groundtruth_y(1:maxIdGTKalman))));

    error_depth = abs(estimated_depth - resample(groundtruth_depth(1:maxIdGTgdDepth), length(estimated_depth), length(groundtruth_depth(1:maxIdGTgdDepth))));
    error_depth_kalman = abs(estimated_depth_kalman - resample(groundtruth_depth(1:maxIdGTKalmanDepth), length(estimated_depth_kalman), length(groundtruth_depth(1:maxIdGTKalmanDepth))));

    % Compute Mean Squared Error (MSE)
    mse_x = mean(error_x.^2);
    mse_y = mean(error_y.^2);
    mse_depth = mean(error_depth.^2);

    mse_x_kalman = mean(error_x_kalman.^2);
    mse_y_kalman = mean(error_y_kalman.^2);
    mse_depth_kalman = mean(error_depth_kalman.^2);


    % Compute Root Mean Square Error (RMSE)
    rmse_x = sqrt(mean(error_x.^2));
    rmse_y = sqrt(mean(error_y.^2));
    rmse_depth = sqrt(mean(error_depth.^2));

    rmse_x_kalman = sqrt(mean(error_x_kalman.^2));
    rmse_y_kalman = sqrt(mean(error_y_kalman.^2));
    rmse_depth_kalman = sqrt(mean(error_depth_kalman.^2));

    
    groundtruth_x_resampled     = resample(groundtruth_x(1:maxIdGTgd), length(estimated_x), length(groundtruth_x(1:maxIdGTgd)));
    groundtruth_y_resampled     = resample(groundtruth_y(1:maxIdGTgd), length(estimated_y), length(groundtruth_y(1:maxIdGTgd)));
    groundtruth_depth_resampled = resample(groundtruth_depth(1:maxIdGTgdDepth), length(estimated_depth), length(groundtruth_depth(1:maxIdGTgdDepth)));


    R_x = corrcoef(estimated_x', groundtruth_x_resampled);
    R_y = corrcoef(estimated_y', groundtruth_y_resampled);
    R_z = corrcoef(estimated_depth', groundtruth_depth_resampled);
    
    R2 = [R_x(1,2), R_y(1,2), R_z(1,2)];

    groundtruth_x_resampled     = resample(groundtruth_x(1:maxIdGTKalman),          length(estimated_x_kalman), length(groundtruth_x(1:maxIdGTKalman)));
    groundtruth_y_resampled     = resample(groundtruth_y(1:maxIdGTKalman),          length(estimated_y_kalman), length(groundtruth_y(1:maxIdGTKalman)));
    groundtruth_depth_resampled = resample(groundtruth_depth(1:maxIdGTKalmanDepth),  length(estimated_depth_kalman), length(groundtruth_depth(1:maxIdGTKalmanDepth)));


    R_x =     corrcoef(estimated_x_kalman', groundtruth_x_resampled);
    R_y =     corrcoef(estimated_y_kalman', groundtruth_y_resampled);
    R_z = corrcoef(estimated_depth_kalman', groundtruth_depth_resampled);
    
    R2_kalman = [R_x(1,2), R_y(1,2), R_z(1,2)];
    
    R2 = R2.^2;
    R2_kalman = R2_kalman.^2;

    
    % Similarly, for Kalman estimates:
    % groundtruth_x_resampled_kalman = resample(groundtruth_x(:,1:maxIdGTKalman), length(estimated_x_kalman), length(groundtruth_x(:,1:maxIdGTKalman)));
    % groundtruth_x_resampled_kalman = resample(groundtruth_x(1:maxIdGTKalman), length(estimated_x_kalman), length(groundtruth_x(1:maxIdGTKalman)));
    % R_kalman = corrcoef(estimated_x_kalman, groundtruth_x_resampled_kalman);
    % corr_coef_kalman = R_kalman(1,2);
    % R2_kalman = corr_coef_kalman^2;
    
    % For cross correlation:
    % Compute cross correlation between estimated and ground truth (resampled) x positions
    [crossCorr, lags] = xcorr(estimated_x', groundtruth_x_resampled);
   
    
    timeElapse = SDP_predictedStruct.timeElapse;
    timeElapseKalman = KalmanStruct.timeElapseKalman;

    fprintf('\n%-20s | %-15s | %-15s\n', 'Metric', 'SDP-GD', 'Sonar SLAM');
    fprintf('%s\n', repmat('-', 1, 55));
    % fprintf('%-20s | %.10f m       | %.10f m\n', 'MSE X', mse_x, mse_x_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'RMSE X', rmse_x, rmse_x_kalman);
    % fprintf('%-20s | %.10f m       | %.10f m\n', 'MSE Y', mse_y, mse_y_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'RMSE Y', rmse_y, rmse_y_kalman);
    % fprintf('%-20s | %.10f m       | %.10f m\n', 'MSE Depth', mse_depth, mse_depth_kalman);
    fprintf('%-20s | %.10f m       | %.10f m\n', 'RMSE Depth', rmse_depth, rmse_depth_kalman);
    fprintf('%-20s | %.10f         | %.10f\n', 'R² X', R2(1), R2_kalman(1));
    fprintf('%-20s | %.10f         | %.10f\n', 'R² Y', R2(2), R2_kalman(2));
    fprintf('%-20s | %.10f         | %.10f\n', 'R² Depth', R2(3), R2_kalman(3));
    fprintf('%-20s | %.10f         | %.10f\n', 'Time elapse',  timeElapse, timeElapseKalman);








    

    

    % % figure('Position', [-1500, 200, 1200, 500]); % Adjusted height for additional subplot
    % figure('Position', [-1440, 270, 960, 540]); % Adjusted height for additional subplot
    % Error in x
    % subplot(2,2,2);
    % 
    % hold on;
    % grid on;
    
    sdp_x_error =error_x;
    kalman_x_error = error_x_kalman;
    


    % --- DRIFT COMPUTATION (Slope of error over time) ---

% X Drift
p_x_sdp = polyfit(SDP_predictedStruct.timestamps, error_x, 1);
drift_x_sdp = p_x_sdp(1);  % [m/s] drift rate in x for SDP-GD

p_x_kalman = polyfit(KalmanStruct.timestamps, error_x_kalman, 1);
drift_x_kalman = p_x_kalman(1);  % [m/s] drift rate in x for Sonar SLAM

% Y Drift
p_y_sdp = polyfit(SDP_predictedStruct.timestamps, error_y, 1);
drift_y_sdp = p_y_sdp(1);

p_y_kalman = polyfit(KalmanStruct.timestamps, error_y_kalman, 1);
drift_y_kalman = p_y_kalman(1);

% Depth Drift
p_depth_sdp = polyfit(SDP_predictedStruct.timestamps, error_depth, 1);
drift_depth_sdp = p_depth_sdp(1);

p_depth_kalman = polyfit(KalmanStruct.timestamps, error_depth_kalman, 1);
drift_depth_kalman = p_depth_kalman(1);

% Display results
fprintf("Drift Rates (slope of error over time):\n");
fprintf("SDP-GD    -> X: %.4f m/s, Y: %.4f m/s, Depth: %.4f m/s\n", drift_x_sdp, drift_y_sdp, drift_depth_sdp);
fprintf("Sonar SLAM -> X: %.4f m/s, Y: %.4f m/s, Depth: %.4f m/s\n", drift_x_kalman, drift_y_kalman, drift_depth_kalman);


    FolderPath = "C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code\Result_section\";
    % Filename = "Noise_Disturbance_" + DisturbanceLevel+  "_Test_"+itt;
    Filename = "Disturbance_" + DisturbanceLevel+  "_Test_"+itt;
    filePath = ""+FolderPath+Filename;

    
    save(filePath, 'mse_x', 'rmse_x', 'mse_y', 'rmse_y', ...
    'mse_depth', 'rmse_depth', 'R2', ...
    'mse_x_kalman', 'rmse_x_kalman', 'mse_y_kalman', 'rmse_y_kalman', ...
    'mse_depth_kalman', 'rmse_depth_kalman', 'R2_kalman', ...
    'timeElapse','timeElapseKalman', ...
    'drift_x_sdp', 'drift_y_sdp', 'drift_depth_sdp', ...
    'drift_x_kalman', 'drift_y_kalman', 'drift_depth_kalman', 'maxFreqGT', 'maxFreqSDP','maxFreqKalman');



    if plot2
        plot(SDP_predictedStruct.timestamps, sdp_x_error, 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD Error');
        plot(KalmanStruct.timestamps, kalman_x_error, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar SLAM Error');
        
        xlabel('Time Step [s]', 'FontSize', 12);
        ylabel('x Error [m]', 'FontSize', 12);
        title('x Error (Prediction - Measured)', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
        
        % Error in y
        subplot(2,2,3);
        hold on;
        grid on;
        
        sdp_y_error = error_y;
        kalman_y_error = error_y_kalman;
        
        plot(SDP_predictedStruct.timestamps, sdp_y_error, 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD Error');
        plot(KalmanStruct.timestamps, kalman_y_error, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar SLAM Error');
        
        xlabel('Time Step [s]', 'FontSize', 12);
        ylabel('y Error [m]', 'FontSize', 12);
        title('y Error (Prediction - Measured)', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
        
        % Error in depth
        subplot(2,2,4);
        hold on;
        grid on;
        
        sdp_depth_error = error_depth;
        kalman_depth_error = error_depth_kalman;
        
        plot(SDP_predictedStruct.timestamps, sdp_depth_error, 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD Error');
        plot(KalmanStruct.timestamps, kalman_depth_error, 'm', 'LineWidth', 2, 'DisplayName', 'Sonar SLAM Error');
        
        xlabel('Time Step [s]', 'FontSize', 12);
        ylabel('Depth Error [m]', 'FontSize', 12);
        title('Depth Error (Prediction - Measured)', 'FontSize', 14);
        legend('Location', 'best');
        set(gca, 'FontSize', 12);
        hold off;
    end


end
