function [SDP_predictedStruct, sonarStruct, measurementsStruct, groundTruthStruct1, KalmanStruct] = ...
    f_MyMain4_trimAndStructureData(SDP_predictedStruct, sonarStruct, measurementsStruct, groundTruthStruct, KalmanStruct)
    
    endTimeSDP = max(SDP_predictedStruct.timestamps);
    % Find the closest indices to endTimeSDP
    [~, idx_gd_predicted] = min(abs(SDP_predictedStruct.timestamps - endTimeSDP));
    [~, idx_sonar]        = min(abs(sonarStruct.timestamps - endTimeSDP));
    [~, idx_sonarUn]      = min(abs(sonarStruct.timestampsUn - endTimeSDP));
    [~, idx_measurements] = min(abs(measurementsStruct.timestamps - endTimeSDP));
    [~, idx_groundTruth]  = min(abs(groundTruthStruct.timestamps - endTimeSDP));
    [~, idx_Kalman]       = min(abs(KalmanStruct.timestamps - endTimeSDP));
    
    
    % Trim sonar data
    sonarStruct.timestamps     = sonarStruct.timestamps(1:idx_sonar);
    sonarStruct.z_positions    = sonarStruct.z_positions(1:idx_sonar);
    sonarStruct.timestampsUn   = sonarStruct.timestampsUn(1:idx_sonarUn);
    sonarStruct.z_positionsUn  = sonarStruct.z_positionsUn(1:idx_sonarUn);

    % Trim gd_predictedStruct
    SDP_predictedStruct.depth       = SDP_predictedStruct.depth(1:idx_gd_predicted);
    SDP_predictedStruct.timestamps  = SDP_predictedStruct.timestamps(1:idx_gd_predicted);
    SDP_predictedStruct.X_norm      = SDP_predictedStruct.X_norm(1:idx_gd_predicted, :);
    SDP_predictedStruct.x_positions = SDP_predictedStruct.x_positions(1:idx_gd_predicted);
    SDP_predictedStruct.y_positions = SDP_predictedStruct.y_positions(1:idx_gd_predicted);

    % Trim measurements
    measurementsStruct.depth        = measurementsStruct.depth(1:idx_measurements);
    measurementsStruct.timestamps   = measurementsStruct.timestamps(1:idx_measurements);
    measurementsStruct.x_positions  = measurementsStruct.x_positions(1:idx_measurements);
    measurementsStruct.y_positions  = measurementsStruct.y_positions(1:idx_measurements);

    % Prepare ground truth struct
    groundTruthStruct1 = groundTruthStruct;
    groundTruthStruct1.timestamps   = groundTruthStruct1.timestamps(1:idx_groundTruth);
    groundTruthStruct1.x_positions  = groundTruthStruct1.x_positions(1:idx_groundTruth);
    groundTruthStruct1.x_velocity   = groundTruthStruct1.x_velocity(1:idx_groundTruth);
    groundTruthStruct1.y_positions  = groundTruthStruct1.y_positions(1:idx_groundTruth);
    groundTruthStruct1.y_velocity   = groundTruthStruct1.y_velocity(1:idx_groundTruth);

    % Add sonar-based depth
    groundTruthStruct1.depth        = sonarStruct.z_positionsUn;
    groundTruthStruct1.z_positionsUn = sonarStruct.z_positionsUn;
    groundTruthStruct1.timestampsUn  = sonarStruct.timestampsUn;

    % Interpolate depth to match x_positions length
    x_orig = linspace(0, 1, length(groundTruthStruct1.depth));
    x_new  = linspace(0, 1, length(groundTruthStruct1.x_positions));
    groundTruthStruct1.depth = interp1(x_orig, groundTruthStruct1.depth, x_new, 'linear');

    % Trim Kalman data
    KalmanStruct.timestamps = KalmanStruct.timestamps(1:idx_Kalman);
    KalmanStruct.vx         = KalmanStruct.vx(1:idx_Kalman);
    KalmanStruct.vy         = KalmanStruct.vy(1:idx_Kalman);
    KalmanStruct.x          = KalmanStruct.x(1:idx_Kalman);
    KalmanStruct.y          = KalmanStruct.y(1:idx_Kalman);
    KalmanStruct.z_depth    = KalmanStruct.z_depth(1:idx_Kalman);
end
