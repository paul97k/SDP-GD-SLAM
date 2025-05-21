function f_MyMain4_Test_outputMaps(obj_ID, cutIdx, SDPOutputMapFilePath, ...
    SonarOutputMapFilePath, folderDataPath,  OutputMapsDataFileName, ... 
    TestDataFileName, itt, trusterInput, plot1,plot2, sigma)

    A= obj_ID.identification.A; 
    B = obj_ID.identification.B;
    Ts = obj_ID.downSampleFreq;
    
    DatafilePath     = folderDataPath           + TestDataFileName+'.mat';

    objData = DataLoader(DatafilePath);
    objData.downSampleFreq = Ts;
    objData.computeNED_AddNoise(true);
    objData.computeXZsonar(false);

    timestampsUn = objData.sonarPositions4.timestamps0; 
    z_positionsUn = objData.sonarPositions4.ranges0; 


    %------------------Downsample 
    % IMU, DVL, Thrusters, eulerGT, CMD, hydro forces
    % ------------------
    objData.DownsampleData();
    objData.computeXZsonar(true);

    sonarStruct = struct( ...
        'timestampsUn', timestampsUn, ...
        'z_positionsUn', z_positionsUn-z_positionsUn(1), ...
        'timestamps', objData.sonarPositions4.timestamps0, ...
        'z_positions', objData.sonarPositions4.ranges0 ...
    );

    %------------------Set input ------------------
    % inputVal = [objData.CMD.cmd_linear_x objData.CMD.cmd_linear_y objData.CMD.cmd_linear_z];
    if trusterInput
        inputs = [objData.thrusters.input.thruster0 objData.thrusters.input.thruster1 objData.thrusters.input.thruster2];
        
    else
        inputVal = [objData.CMD.cmd_linear_x objData.CMD.cmd_linear_y objData.CMD.cmd_linear_z];
        u1 = zeros(length(inputVal),3); %the x,y,z, doesnt have 0 refernce inputs
        u2 = inputVal;
        inputs = [u1 u2];
    end



    % 

    
%------------------------------------1. measurment data:  Set Measurement ------------------
    %------------------ Cut the measurment to size cutIdx ------------------

    measured_z = sonarStruct.z_positions';
    if cutIdx>length(measured_z)
        cutIdx = length(measured_z);
    end
        % measured_z = measured_z(1:cutIdx,:); %and then measurement will be also cut
        measured_z = measured_z(1:cutIdx); %and then measurement will be also cut
        % time_measurementSonar = sonarStruct.timestamps(1:cutIdx); 
        inputs = inputs(1:cutIdx,:);


    X_intg = objData.positionIntegrated.X_intg; % X position
    Y_intg = objData.positionIntegrated.Y_intg; % Y position
    Z_intg = objData.positionIntegrated.Z_intg; % Y position
    time_measurement = objData.positionIntegrated.timestamps(1:length(measured_z)); 

    %------------------ by:taking the middle element of the sonar range ------------------



    %here sonar and dvl measurement differs by only 1-2 elements (neglegtable)
    measurements = {X_intg(1:length(measured_z),:) , Y_intg(1:length(measured_z),:), measured_z};


    % measurements{3}(measurements{3} == Inf) = 5;
    % measurements{3}(measurements{3} == -Inf) = -5;
    % measurements{3}(isnan(measurements{3})) = 5;

    measurementsStruct = struct( ...
        'timestamps', time_measurement, ...
        'x_positions', measurements{1},...
        'y_positions', measurements{2},...
        'z_positions', Z_intg(1:length(measured_z),:),...
        'depth', measurements{3}-measurements{3}(1)...
    );



    %------------------------------------set ground truth ------------------
    groundTruthStruct = struct( ...
        'timestamps', objData.positionGT.timestamps, ...
        'x_positions', objData.positionGT.x_positions,...
        'y_positions', objData.positionGT.y_positions,...
        'z_positions', objData.positionGT.z_positions,...
        'x_velocity', objData.velocityGT.x_vel,...
        'y_velocity', objData.velocityGT.y_vel,...
        'z_velocity', objData.velocityGT.z_vel...
    );



    % % -----------------Load SDP-GD Output map------------------
    % 
    estimated_C_thetaFile = load(SDPOutputMapFilePath); %'estimated_C_theta' and   'nominalX'
    estimated_C_theta= estimated_C_thetaFile.estimated_C_theta;
    timeElapse                   =estimated_C_thetaFile.timeElapse;
    
    nT = cutIdx;
    nx = 6;
    ny = 3;
    x_0 = [0 0 5 0 0 0];


    nominalX = f_MyMain4_createNominalX(A,B,inputs, nT, x_0);
    X_norm =            reshape(nominalX,       nx, nT)';

    SDP_predicted_y2 = estimated_C_theta * nominalX;
    SDP_predicted_y_norm2 =    reshape(SDP_predicted_y2, ny, nT)';


    X_norm(:,3)=X_norm(:,3)-X_norm(1,3);
    SDP_predictedStruct = struct( ...
        'timestamps', measurementsStruct.timestamps, ...
        'x_positions', SDP_predicted_y_norm2(:,1),...
        'y_positions', SDP_predicted_y_norm2(:,2),...
        'depth',       SDP_predicted_y_norm2(:,3)-SDP_predicted_y_norm2(1,3),...
        'X_norm', X_norm, ...
        'timeElapse', timeElapse...
    );



    % -----------------Load Kalman Map------------------
    z = measurements{3}-measurements{3}(1);

    % [mainFrequencies, magnitudes, f, Z_mag] = extractMainHarmonics(time_measurement, z);

    
    fs = 1 / mean(diff(time_measurement));     % Sampling frequency (Hz)
    fc = 2.5;                   % Cutoff frequency (Hz) — adjust as needed
    order = 4;                  % Filter order
    
    % Normalize cutoff frequency
    Wn = fc / (fs/2);           % Normalize to Nyquist frequency
    
    % Design low-pass Butterworth filter
    [b, a] = butter(order, Wn, 'low');
    
    % Apply filter
    z_filtered = filtfilt(b, a, z);  % Zero-phase filtering to avoid lag

% [mainFrequencies, magnitudes, f, Z_mag] = extractMainHarmonics(time_measurement, z_filtered);

        measurementsStructKalman = struct( ...
        'timestamps', time_measurement, ...
        'x_positions', measurements{1},...
        'y_positions', measurements{2},...
        'z_positions', Z_intg(1:length(measured_z),:),...
        'depth',z_filtered ...
    );

    KalmanStruct = PerformKalman( obj_ID, inputs, measurementsStruct,sigma); %saves result in KalmanResult.mat

    KalmanResultData= load(SonarOutputMapFilePath); 
    SLAM_OutputMap =KalmanResultData.KalmanStruct.map_By_Kalman;
    
    
    % KalmanStruct = PerformKalman( obj_ID, inputs, measurementsStructKalman,sigma); %saves result in KalmanResult.mat
    % SLAM_OutputMap =KalmanStruct.map_By_Kalman;

    % KalmanStruct.z_depth =  findClosestZ(KalmanStruct.x,KalmanStruct.y, map_By_KalmanSaved);

            X_map = SLAM_OutputMap(1, :)';
            Y_map = SLAM_OutputMap(2, :)';
            Z_map = SLAM_OutputMap(3, :)';
            
            % Find nearest neighbor indices
            idx = knnsearch([X_map, Y_map], [KalmanStruct.x; KalmanStruct.y]');
        
            % Retrieve corresponding Z values
            
            z_depth_prev = Z_map(idx);
    KalmanStruct.z_depth =z_depth_prev-z_depth_prev(1);

    % -----------------Set end time for each data same as the SDP ent time and struct the data------------------   

    [SDP_predictedStruct, sonarStruct, measurementsStruct, groundTruthStruct1, KalmanStruct] = ...
    f_MyMain4_trimAndStructureData(SDP_predictedStruct, sonarStruct, measurementsStruct, groundTruthStruct, KalmanStruct);


% ----------------------------------Plot results-----------------------------------   


    timeElapseSonarSLAM = KalmanStruct.timeElapseKalman;


    if contains(DatafilePath, "Test")
        checkTextDisturbance = 'S0\.22_D10_R0\.5_D(.*?)_Test';
        checkTextTest = 'Test(.*?)\.mat';

    elseif contains(DatafilePath, "Noise")
        checkTextDisturbance = 'S0\.22_D10_R0\.5_D(.*?)_Noise';
        checkTextTest = 'Noise(.*?)\.mat';
    end
    
    DisturbanceLevel = regexp(DatafilePath, checkTextDisturbance, 'tokens');
    DisturbanceLevel = DisturbanceLevel{1}{1};  % Extract from nested cell

    Test = regexp(DatafilePath, checkTextTest, 'tokens');
    Test = Test{1}{1};  % Extract from nested cel


    TestTitle = "Disturbance level "+ DisturbanceLevel+ " Test: " + Test;
    % 
    disp("----------------------------------------------------------------")




    f_MyMain4_plot_comparison( measurementsStruct, groundTruthStruct1, SDP_predictedStruct, KalmanStruct, OutputMapsDataFileName, TestDataFileName,DisturbanceLevel, itt, plot1,plot2)


    disp('---------------------------------')
    disp(" ")
    disp(" ")

end
