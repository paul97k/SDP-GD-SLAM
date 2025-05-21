
clear
clc
close all
obj_ID_ = load('identifiedSystem.mat', 'obj');
obj_ID = obj_ID_.obj;
dataFolder_Chirp = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Chirp';
ChirpFiles = ["chirp0.001-1x.mat", "chirp0.0001-1xyz.mat", "chirp0.0001-1xyz+1.mat"];


dataFolder_Going_straight = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
StraightControllerFile = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat"];

dataFolder_Identification = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Identification';
InputFile = ["u1_0.05.mat", "u1_0.5.mat", "u6_0.5.mat", "u7_0.5.mat"]; %#4

dataFolder_Step_Response = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Step_Response';
StepResponeFile = ["SR0.01.mat", "SR0.1.mat", "SR0.2.mat", "SR0.5.mat", "SR0.7.mat", "SR1.0.mat", "SR1.2.mat"]; %#7

dataFolder_Straight_differentBottoms = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/SDP_GD';
Straight_differentBottoms = ["S0.22_D10_R0.5_D0_withoutAnything.mat", ...
    "S0.22_D10_R0.5_D0.mat", "S0.22_D10_R0.5_D1.mat", "S0.22_D10_R0.5_D2.mat", "S0.22_D10_R0.5_D3.mat" , ... 
    "S0.22_D10_R0.5_D4.mat", "S0.22_D10_R0.5_D6.mat", "S0.22_D10_R0.5_D8.mat", "S0.22_D10_R0.5_D10.mat", ...
    "S0.22_D10_R0.5_D12.mat"]; %#10



fileNameTheta = [...
    "estimated_C_thetaT60Ts1_D0_withoutAnything.mat",  ...
    "estimated_C_thetaT60Ts1_D0.mat",  ...
    "estimated_C_thetaT60Ts1_D1.mat", ...
    "estimated_C_thetaT60Ts1_D2.mat", ...
    "estimated_C_thetaT60Ts1_D3.mat", ...
    "estimated_C_thetaT60Ts1_D4.mat", ...
    "estimated_C_thetaT60Ts1_D5.mat", ...
    "estimated_C_thetaT60Ts1_D8.mat", ...
    "estimated_C_thetaT60Ts1_D10.mat", ...
    "estimated_C_thetaT60Ts1_D12.mat"
    ]; 

% delayAllGT = [0,...
%     0,0.75,1,1,...
%     0.75,0,0,0,0.75];
% 

fileNumber  = 5;
for fileNumber = 1:10
    folderName = dataFolder_Straight_differentBottoms;
    file =  Straight_differentBottoms(fileNumber);
    % delay = delayAllGT(fileNumber);
    loadSaveCTheta = fileNameTheta(fileNumber);
        
    % load('identifiedSystemTs2.mat', 'obj');
    % load('estimated_C_thetaTs2.t100.mat');
    % cutIdx = 100;
    
                %------------------load identified system ------------------
    % load('identifiedSystemTs0.5.mat', 'obj');
    % load('estimated_C_theta.mat');
    cutIdx = 60;
    
    % A_ = obj_ID.identification.A(1:2,1:2); 
    % B_ = obj_ID.identification.B(1:2,1:2);
    A= obj_ID.identification.A; 
    B = obj_ID.identification.B;
    dt = obj_ID.identification.dt;
    Ts = obj_ID.downSampleFreq;
    
    
    
                %------------------load Data ------------------
    
    
    
    objVal = DataLoader(folderName, file);
    objVal.downSampleFreq = Ts;
    
    objVal.computeNED_AddNoise(true);
    
    objVal.computeXZsonar(false);
    
    
                    %------------------Downsample 
                    % IMU, DVL, Thrusters, eulerGT, CMD, hydro forces
                    % ------------------
    objVal.DownsampleData();
    
    
    objVal.computeXZsonar(true);
    
    
    
    middleIdx = round(size(objVal.sonarPositions.z_positions,2)/2);
    measured_z_Original = objVal.sonarPositions.z_positions(:,middleIdx);
    time_measured_z_Original =  objVal.sonarPositions.timestamps;
    
    
    sonarStruct = struct( ...
        'timestampsUn', objVal.sonarPositions.timestamps, ...
        'z_positionsUn', objVal.sonarPositions.z_positions(:,middleIdx)...
    );
    
                %------------------Set input output data------------------
    
    inputVal = [objVal.CMD.cmd_linear_x objVal.CMD.cmd_linear_y objVal.CMD.cmd_linear_z];
    
    
    u1 = zeros(length(inputVal),3);
    u2 = inputVal;
    inputs = [u1 u2];
    VelocityNED = [objVal.NED.V_x objVal.NED.V_y objVal.NED.V_z];
    t = objVal.imu.timestamps;
    
    if size(inputVal,1) > size(VelocityNED,1)
        inputVal = inputVal(1:size(VelocityNED,1),:);
    elseif size(inputVal,1) < size(VelocityNED,1)
        VelocityNED = VelocityNED(1:size(inputVal,1),:);
        t = t(1:size(inputVal,1),:);
        
    end
    
    
    
                %------------------Integrate velocity=>position ------------------
    
    Vx = VelocityNED(:,1);  % X velocity
    Vy = VelocityNED(:,2);  % Y velocity
    
    X_intg = objVal.positionIntegrated.X_intg; % X position
    Y_intg = objVal.positionIntegrated.Y_intg; % Y position
    
                
    middleIdx =round(size(objVal.sonarPositions.z_positions,2)/2);
    measured_z = objVal.sonarPositions.z_positions(:,middleIdx);
    % measured_z(measured_z == Inf) = 5;
    % measured_z(measured_z == -Inf) = -5;
    
    
    if cutIdx>length(measured_z)
        cutIdx = length(measured_z);
    end
        measured_z = measured_z(1:cutIdx,:); %and then measurement will be also cut
        inputs = inputs(1:cutIdx,:);
    
    
    
                %------------------Set Measurement ------------------
                %here sonar and dvl measurement differs by only 1-2 elements (neglegtable)
    measurements = {X_intg(1:length(measured_z),:) , Y_intg(1:length(measured_z),:), measured_z};
    time_measurement = objVal.positionIntegrated.timestamps(1:length(measured_z)); 
    position_gt = [objVal.positionGT.x_positions; objVal.positionGT.y_positions; objVal.positionGT.z_positions];
    
    
    measurementsStruct = struct( ...
        'timestamps', time_measurement, ...
        'x_positions', measurements{1},...
        'y_positions', measurements{2},...
        'depth', measurements{3}...
    );
    
    groundTruthStruct = struct( ...
        'timestamps', objVal.positionGT.timestamps, ...
        'x_positions', objVal.positionGT.x_positions,...
        'y_positions', objVal.positionGT.y_positions,...
        'z_positions', objVal.positionGT.z_positions,...
        'x_velocity', objVal.velocityGT.x_vel,...
        'y_velocity', objVal.velocityGT.y_vel,...
        'z_velocity', objVal.velocityGT.z_vel...
    );
    
                %------------------Init class------------------
    
    x_0 = [0 0 -5 0 0 0];
    
    objSDP = classSDP_GD(A, B, dt, inputs, measurements, x_0);
    
    
    nx = size(objSDP.A,1);
    ny = 1;
    ntheta = nx*ny;
    
    
    
                %------------------Set Noise ------------------
    factorx = 1;
    mu_theta0 = ones(ntheta,1)*0.01;
    mu_eta = ones(ntheta,1)*0.01;
    sigma_v = 0.01; %output noice
    sigma_wx0       = diag(ones(nx,1)*0.01); %process noise
    sigma_wx        = sigma_wx0;    
    sigma_theta0    = diag(ones(ntheta,1))*0.01;
    sigma_eta       = sigma_theta0; %theta noise*
    
    objSDP.setNoise(mu_theta0, mu_eta, sigma_v, sigma_wx0, sigma_wx, sigma_theta0, sigma_eta);
    
    
    
    
                % -----------------perform SDP-GD------------------
    
    % 
    % objSDP.performSDP_GD(); %performs C for all 3 measurements
    % 
    % objSDP.mergeC() %merge to 1 C
    % estimated_C_theta = objSDP.estimated_C_theta;
    % nominalX                   =objSDP.nominalX{3};
    % save(loadSaveCTheta', 'estimated_C_theta',  'nominalX');
    % 
    % %% Test C
    
    load(loadSaveCTheta);
    % load('C_TS2t150_S0.22_D10_R0.5_D0.mat');
    
    measurment = [measurements{1}, measurements{2}, measurements{3}];
    % measurment = measurment(1:cutIdx,:);
    measured_yT = measurment';
    measuredY = measured_yT(:);
    
    nT = length(measurment);
    nx = 6;
    ny = 3;
    
    measurment_normal = reshape(measuredY,    ny, nT)';
    X_norm =            reshape(nominalX,       nx, nT)';
    
    % PLot C
    % load('myMain3_estimated_C_theta.mat')
    gd_predicted_y2 = estimated_C_theta * nominalX;
    gd_predicted_y_norm2 =    reshape(gd_predicted_y2, ny, nT)';
    
    gd_predictedStruct = struct( ...
        'timestamps', measurementsStruct.timestamps, ...
        'x_positions', gd_predicted_y_norm2(:,1),...
        'y_positions', gd_predicted_y_norm2(:,2),...
        'depth', gd_predicted_y_norm2(:,3),...
        'X_norm', X_norm...
    );
    
                -----------------perform Kalman------------------
    
    PerformKalman(folderName, file)
    KalmanResultData= load('KalmanResult.mat');
    % % delay =1;
    
    KalmanStruct =KalmanResultData.KalmanStruct;
    
    endTimeSDP = max(gd_predictedStruct.timestamps);
    % endTimeSDP = 40;
    
    
    [~, idx_gd_predicted] = min(abs(gd_predictedStruct.timestamps - endTimeSDP));
    [~, idx_sonar] = min(abs(sonarStruct.timestampsUn - endTimeSDP));
    [~, idx_measurements] = min(abs(measurementsStruct.timestamps - endTimeSDP));
    [~, idx_groundTruth] = min(abs(groundTruthStruct.timestamps - endTimeSDP));
    [~, idx_Kalman] = min(abs(KalmanStruct.timestamps - endTimeSDP));
    idx_Kalman = idx_Kalman;
    sonarStruct.timestampsUn = sonarStruct.timestampsUn(1:idx_sonar);
    sonarStruct.z_positionsUn = sonarStruct.z_positionsUn(1:idx_sonar);
    
    gd_predictedStruct.depth = gd_predictedStruct.depth(1:idx_gd_predicted);
    gd_predictedStruct.timestamps = gd_predictedStruct.timestamps(1:idx_gd_predicted);
    gd_predictedStruct.X_norm = gd_predictedStruct.X_norm(1:idx_gd_predicted,:);
    gd_predictedStruct.x_positions = gd_predictedStruct.x_positions(1:idx_gd_predicted);
    gd_predictedStruct.y_positions = gd_predictedStruct.y_positions(1:idx_gd_predicted);
    
    
    
    measurementsStruct.depth = gd_predictedStruct.depth(1:idx_measurements);
    measurementsStruct.timestamps = measurementsStruct.timestamps(1:idx_measurements);
    measurementsStruct.x_positions = measurementsStruct.x_positions(1:idx_measurements);
    measurementsStruct.y_positions = measurementsStruct.y_positions(1:idx_measurements);
    
    groundTruthStruct1=groundTruthStruct; 
    groundTruthStruct1.timestamps = groundTruthStruct1.timestamps(1:idx_groundTruth);
    groundTruthStruct1.x_positions = groundTruthStruct1.x_positions(1:idx_groundTruth)+gd_predictedStruct.x_positions(1);
    groundTruthStruct1.x_velocity = groundTruthStruct1.x_velocity(1:idx_groundTruth);
    groundTruthStruct1.y_positions = groundTruthStruct1.y_positions(1:idx_groundTruth);
    groundTruthStruct1.y_velocity = groundTruthStruct1.y_velocity(1:idx_groundTruth);
    
    KalmanStruct.timestamps = KalmanStruct.timestamps(1:idx_Kalman);
    KalmanStruct.vx = KalmanStruct.vx(1:idx_Kalman);
    KalmanStruct.vy = KalmanStruct.vy(1:idx_Kalman);
    KalmanStruct.x = KalmanStruct.x(1:idx_Kalman);
    KalmanStruct.y = KalmanStruct.y(1:idx_Kalman);
    KalmanStruct.z_depth = KalmanStruct.z_depth(1:idx_Kalman);
    
    
    file
    plot_comparison(sonarStruct, measurementsStruct, groundTruthStruct1, gd_predictedStruct, KalmanStruct)
end
% %%
% signal =  measurements{3};
% Fs = Ts; % Sampling frequency (adjust as needed)
% N = length(signal);
% f = (0:N-1)*(Fs/N); % Frequency axis
% Y = abs(fft(signal)); % Magnitude of FFT
% 
% figure;
% plot(f, Y);
% xlabel('Frequency (Hz)');
% ylabel('Magnitude');
% title('Frequency Spectrum');


function plot_comparison(sonarStruct, measurementsStruct, groundTruthStruct, gd_predictedStruct, KalmanStruct)
    % Create figure with a specific position and size

    % Resample signal for better alignment
    groundTruthStruct.depth = sonarStruct.z_positionsUn;

    skipI = length(groundTruthStruct.x_positions)/length(groundTruthStruct.depth);
    for i =1:length(groundTruthStruct.depth)
        groundTruthStruct.x_positionsDepth(i) =groundTruthStruct.x_positions(1,round(i*skipI));
        groundTruthStruct.depthTime(i) = groundTruthStruct.timestamps(1,round(i*skipI));
    end


    % 
    figure('Position', [-1920, 0, 1920, 1200]); % Adjusted height for additional subplot

    % subplot(3,3,1);
    % hold on;
    % grid on;
    % plot(gd_predictedStruct.timestamps, gd_predictedStruct.X_norm(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Nominal Vx ');
    % plot(groundTruthStruct.timestamps, groundTruthStruct.x_velocity, 'r-', 'LineWidth', 2, 'DisplayName', 'GT Vx');
    % plot(gd_predictedStruct.timestamps, gd_predictedStruct.X_norm(:,4), 'b-', 'LineWidth', 2, 'DisplayName', 'Nominal Vy');
    % plot(groundTruthStruct.timestamps, groundTruthStruct.y_velocity, 'r-', 'LineWidth', 2, 'DisplayName', 'GT Vy');
    % plot(KalmanStruct.timestamps, KalmanStruct.vx, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman');
    % xlabel('Time Step [s]', 'FontSize', 12);
    % ylabel('Velocity [m/s]', 'FontSize', 12);
    % title('Vx Vy', 'FontSize', 14);
    % legend('Location', 'best');
    % set(gca, 'FontSize', 12);
    % hold off;

    subplot(3,3,1);
    hold on;
    grid on;
    plot(groundTruthStruct.timestamps, groundTruthStruct.x_positions, 'g', 'LineWidth', 2, 'DisplayName', 'GT X');
    plot(gd_predictedStruct.timestamps, gd_predictedStruct.x_positions , 'b.', 'LineWidth', 2, 'DisplayName', 'SDP-GD X ');
    plot(KalmanStruct.timestamps, KalmanStruct.x, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman');

    xlabel('Time Step [s]', 'FontSize', 12);
    ylabel('x Distance [m]', 'FontSize', 12);
    title('x position', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;


    subplot(3,3,2);
    hold on;
    grid on;
    plot(gd_predictedStruct.timestamps, gd_predictedStruct.y_positions , 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD y ');
    plot(groundTruthStruct.timestamps, groundTruthStruct.y_positions, 'g.', 'LineWidth', 2, 'DisplayName', 'GT y');
    plot(KalmanStruct.timestamps, KalmanStruct.y, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman');

    xlabel('Time Step [s]', 'FontSize', 12);
    ylabel('x Distance [m]', 'FontSize', 12);
    title('y position', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;


    % Fourth subplot: Comparison of Predicted vs Measured XY
    subplot(3,3,3);
    hold on;
    grid on;
    plot(measurementsStruct.x_positions, measurementsStruct.y_positions, 'r--', 'LineWidth', 2, 'DisplayName', 'Measured');
    plot(gd_predictedStruct.X_norm(:,1), gd_predictedStruct.X_norm(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Nominal');
    plot(groundTruthStruct.x_positions, groundTruthStruct.y_positions, 'g-', 'LineWidth', 2, 'DisplayName', 'Ground truth');
    plot(KalmanStruct.x, KalmanStruct.y, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman');
    plot(gd_predictedStruct.x_positions , gd_predictedStruct.y_positions, 'b.', 'LineWidth', 2, 'DisplayName', 'SDP-GD');
    xlabel('x Distance [m]', 'FontSize', 12);
    ylabel('y Distance [m]', 'FontSize', 12);
    title('XY', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;


    subplot(3,3,3);
    hold on;
    grid on;
    plot(gd_predictedStruct.timestamps, gd_predictedStruct.depth , 'b-', 'LineWidth', 2, 'DisplayName', 'SDP-GD depth ');
    plot(groundTruthStruct.depthTime, groundTruthStruct.depth, 'g-', 'LineWidth', 2, 'DisplayName', 'GT depth');
    plot(KalmanStruct.timestamps, KalmanStruct.z_depth, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman');
    plot(measurementsStruct.timestamps, measurementsStruct.depth, 'r--', 'LineWidth', 2, 'DisplayName', 'Measured');

    xlabel('Time Step [s]', 'FontSize', 12);
    ylabel('Depth Distance [m]', 'FontSize', 12);
    title('depth', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;



    % Third subplot: Measured vs Predicted Depth (Z)
    subplot(3,3,[4,5,6,7,8,9]);
    hold on;
    grid on;
    plot(gd_predictedStruct.x_positions , gd_predictedStruct.depth, 'b', 'LineWidth', 2, 'DisplayName', 'SDP-D');
    plot(groundTruthStruct.x_positionsDepth , groundTruthStruct.depth, 'g', 'LineWidth', 2, 'DisplayName', 'Groundtruth Depth');
    plot(measurementsStruct.x_positions, measurementsStruct.depth, 'r--', 'LineWidth', 2, 'DisplayName', 'Measurement');
    plot(KalmanStruct.x, KalmanStruct.z_depth, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman');
    xlabel('x Distance [m]', 'FontSize', 12);
    ylabel('z Distance [m]', 'FontSize', 12);
    title('XZ', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;

    [~, maxIdGTKalman] = min(abs(groundTruthStruct.timestamps - max(KalmanStruct.timestamps)));
    [~, maxIdGTgd] = min(abs(groundTruthStruct.timestamps - max(gd_predictedStruct.timestamps)));

    [~, maxIdGTKalmanDepth] = min(abs(groundTruthStruct.depthTime - max(KalmanStruct.timestamps)));
    [~, maxIdGTgdDepth] = min(abs(groundTruthStruct.depthTime - max(gd_predictedStruct.timestamps)));




    % Error Calculations
    estimated_x = gd_predictedStruct.x_positions;
    estimated_x_kalman = KalmanStruct.x;
    groundtruth_x = groundTruthStruct.x_positions;

    estimated_y = gd_predictedStruct.y_positions;
    estimated_y_kalman = KalmanStruct.y;
    groundtruth_y = groundTruthStruct.y_positions;

    estimated_depth = gd_predictedStruct.depth;
    estimated_depth_kalman = KalmanStruct.z_depth;
    groundtruth_depth = groundTruthStruct.depth;



    % 
    groundtruth_depth(groundtruth_depth == Inf) = 5;
    groundtruth_depth(groundtruth_depth == -Inf) = -5;
    estimated_depth_kalman(isnan(estimated_depth_kalman)) = 5;




    % Compute Absolute Error (with resampling for alignment)
    error_x = abs(estimated_x' - resample(groundtruth_x(:,1:maxIdGTgd), length(estimated_x), length(groundtruth_x(:,1:maxIdGTgd))));
    error_x_kalman = abs(estimated_x_kalman - resample(groundtruth_x(:,1:maxIdGTKalman), length(estimated_x_kalman), length(groundtruth_x(:,1:maxIdGTKalman))));

    error_y = abs(estimated_y' - resample(groundtruth_y(:,1:maxIdGTgd), length(estimated_y), length(groundtruth_y(:,1:maxIdGTgd))));
    error_y_kalman = abs(estimated_y_kalman - resample(groundtruth_y(:,1:maxIdGTKalman), length(estimated_y_kalman), length(groundtruth_y(:,1:maxIdGTKalman))));

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

gt_x_resampled = resample(groundtruth_x, length(estimated_x), length(groundtruth_x));
gt_depth_resampled = resample(groundtruth_depth, length(estimated_depth), length(groundtruth_depth));





%     For correlation coefficient and R²:
% Make sure the arrays are column vectors (or both are row vectors)
% For the x positions (similar for y and depth)
groundtruth_x_resampled = resample(groundtruth_x(:,1:maxIdGTgd), length(estimated_x), length(groundtruth_x(:,1:maxIdGTgd)));
R = corrcoef(estimated_x', groundtruth_x_resampled);  % Note: estimated_x is transposed in your code
corr_coef = R(1,2);  % This is the Pearson correlation coefficient
R2 = corr_coef^2;    % R² value

% Similarly, for Kalman estimates:
groundtruth_x_resampled_kalman = resample(groundtruth_x(:,1:maxIdGTKalman), length(estimated_x_kalman), length(groundtruth_x(:,1:maxIdGTKalman)));
R_kalman = corrcoef(estimated_x_kalman, groundtruth_x_resampled_kalman);
corr_coef_kalman = R_kalman(1,2);
R2_kalman = corr_coef_kalman^2;

% For cross correlation:
% Compute cross correlation between estimated and ground truth (resampled) x positions
[crossCorr, lags] = xcorr(estimated_x', groundtruth_x_resampled);
% % You can then plot the cross-correlation:
% figure;
% plot(lags, crossCorr);
% xlabel('Lag');
% ylabel('Cross-correlation');
% title('Cross-correlation between estimated and ground truth x positions');
% 

fprintf('%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n', ...
    rmse_x, rmse_y, rmse_depth, rmse_x_kalman, rmse_y_kalman, rmse_depth_kalman, R2, R2_kalman);

    % % Display errors
    % fprintf('\nError Metrics SDP-GD:\n');
    % fprintf('MSE X: %.4f m, RMSE X: %.4f m\n', mse_x, rmse_x);
    % fprintf('MSE Y: %.4f m, RMSE Y: %.4f m\n', mse_y, rmse_y);
    % fprintf('MSE Depth: %.4f m, RMSE Depth: %.4f m\n', mse_depth, rmse_depth);
    % fprintf('R2: %.4f m\n', R2);
    % 
    % 
    % fprintf('\nError Metrics Kalman:\n');
    % fprintf('MSE X: %.4f m, RMSE X: %.4f m\n', mse_x_kalman, rmse_x_kalman);
    % fprintf('MSE Y: %.4f m, RMSE Y: %.4f m\n', mse_y_kalman, rmse_y_kalman);
    % fprintf('MSE Depth: %.4f m, RMSE Depth: %.4f m\n', mse_depth_kalman, rmse_depth_kalman);
    % fprintf('R2: %.4f m\n', R2_kalman);

    % figure;
    % 
    % % First Subplot: Error in X
    % subplot(3,1,1);
    % hold on;
    % grid on;
    % plot(gd_predictedStruct.timestamps, error_x, 'r-', 'LineWidth', 2, 'DisplayName', 'SDP-GD Error in X');
    % plot(KalmanStruct.timestamps, error_x_kalman, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman Error in X');
    % xlabel('Time Step [s]', 'FontSize', 12);
    % ylabel('Error X [m]', 'FontSize', 12);
    % title('Absolute Error in X', 'FontSize', 14);
    % legend('Location', 'best');
    % set(gca, 'FontSize', 12);
    % hold off;
    % 
    % % Second Subplot: Error in Y
    % subplot(3,1,2);
    % hold on;
    % grid on;
    % plot(gd_predictedStruct.timestamps, error_y, 'r-', 'LineWidth', 2, 'DisplayName', 'SDP-GD Error in Y');
    % plot(KalmanStruct.timestamps, error_y_kalman, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman Error in Y');
    % xlabel('Time Step [s]', 'FontSize', 12);
    % ylabel('Error Y [m]', 'FontSize', 12);
    % title('Absolute Error in Y', 'FontSize', 14);
    % legend('Location', 'best');
    % set(gca, 'FontSize', 12);
    % hold off;
    % 
    % % Third Subplot: Error in Depth
    % subplot(3,1,3);
    % hold on;
    % grid on;
    % plot(gd_predictedStruct.timestamps, error_depth, 'r-', 'LineWidth', 2, 'DisplayName', 'SDP-GD Error in Depth');
    % plot(KalmanStruct.timestamps, error_depth_kalman, 'm', 'LineWidth', 2, 'DisplayName', 'Kalman Error in Depth');
    % xlabel('Time Step [s]', 'FontSize', 12);
    % ylabel('Error Depth [m]', 'FontSize', 12);
    % title('Absolute Error in Depth', 'FontSize', 14);
    % legend('Location', 'best');
    % set(gca, 'FontSize', 12);
    % hold off;


end

function PerformKalman(folderName, file)

    % -------------------Init Identified system ---------------------middleIdx--
    % obj_ID_ = load('identifiedSystem_Kalman.mat', 'obj');
    obj_ID_ = load('identifiedSystem.mat', 'obj');
    obj_ID = obj_ID_.obj;
    Ts = obj_ID.downSampleFreq;
    
    
    % -------------------Init Data to perform Kalman on-----------------------
    objData = DataLoader(folderName,file); %folder and fileName
    objData.downSampleFreq = Ts;
    objData.computeNED_AddNoise(true, 0.02);
    objData.DownsampleData();
    objData.computeXZsonar(true);
    
    
    % -------------------Init Kalman Class-----------------------
    objKal = classKalman(objData, obj_ID.identification);
    
    
    % -------------------Set input and measurement-----------------------
    objKal.setInputAndMeasurement();
    input = objKal.u;
    time = objKal.timeId;
    z_meas_val = objKal.z_meas_val ; 
    % -------------------perform EKF-----------------------
    % Q = diag([0.001,0.001,0.001,0.001,0.001,0.01 ,0.01,0.01,0.01]); % Process noise
    % R = diag([0.0001,0.001,0.001 ,0.01,0.01,0.01]);  % Measurement noise
    % x_0 = zeros(9,1);
    % P = eye(size(Q,1));  
    Q = diag([0.01,0.01,0.01,0.01,0.01,0.01]); % Process noise
    R = diag([0.01,0.01,0.01]);  % Measurement noise
    x_0 = [0 0 -5 0 0 0]';
    P = eye(size(Q,1));  
    
    
    paramaters = struct( ...
                    'Q', Q, ...
                    'R', R, ...
                    'x_0', x_0, ... % Fix variable name mix-up
                    'P', P ...
                    );
    
    objKal.PerformEKF(input, z_meas_val, paramaters)
    x_estimates = objKal.x_estimates;       
    
    
    % -------------------Plot result-----------------------

    
    
    skip_i = 2;
    skip_j = 20;
 
    map_By_Kalman  = objKal.CreateKalmanMAP(x_estimates, skip_i, skip_j);
    [timeGT,timeLoc,...
                      map_pointsGT, ...
                      vxGT, vyGT, ...
                      X_intg, Y_intg, ...
                      xGT, yGT,  Z_AUV,...
                      Z_est, Z_nearest, Z_GT] = objKal.CutAndGetNearestDataFromMap(skip_i, skip_j, x_estimates, map_By_Kalman);

    
    KalmanStruct = struct( ...
        'timestamps', timeLoc, ...
        'x', x_estimates(1,:), ...
        'y', x_estimates(2,:), ...
        'z', x_estimates(3,:), ...
        'z_depth', Z_est, ...
        'vx', x_estimates(4,:), ...
        'vy', x_estimates(5,:), ...
        'vz', x_estimates(6,:) ...
    );
    
        % 'vz', x_estimates(6,:) ...
    
    save('KalmanResult.mat', 'KalmanStruct');

end