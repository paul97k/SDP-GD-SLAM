function f_MyMain4_create_outputMaps(obj_ID, cutIdx, folderDataPath, dataFolder_OutputMapSDP, dataFolder_OutputMapSonarBased, DataFileName, trusterInput, sigma)

    A= obj_ID.identification.A; 
    B = obj_ID.identification.B;
    dt = obj_ID.identification.dt;
    Ts = obj_ID.downSampleFreq;

    MyMain4_variables; 
    
    %------------------Generate file path for Data CTheta and Sonar results
    DatafilePath     = folderDataPath           + DataFileName +'.mat';
    CThetaFilePath   = dataFolder_OutputMapSDP     + DataFileName+'.nT_'+cutIdx+'.Ts_'+Ts+'.mat'; %get previously saved C theta
    SonarMapFilePath = dataFolder_OutputMapSonarBased + DataFileName+'.nT_'+cutIdx+'.Ts_'+Ts+'.mat'; %get previously saved C theta
        
    %------------------Load data
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
    signal = objData.sonarPositions4.ranges0;
    noisySignal =  signal-signal(1)+ sigma* randn(size(signal));


    
    
    % Compute noise
    noise = noisySignal - signal;
    
    % Calculate powers
    signalPower = mean(signal.^2);
    noisePower = mean(noise.^2);
    
    % Calculate SNR in dB
    snr_dB = 10 * log10(signalPower / noisePower);
    
    % fprintf('SNR = %.2f dB\n', snr_dB);
        

    figure
    sonarStruct = struct( ...
        'timestampsUn', timestampsUn, ...
        'z_positionsUn', z_positionsUn, ...
        'timestamps', objData.sonarPositions4.timestamps0, ...
        'z_positions', noisySignal ...
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
    
%------------------------------------1. measurment data:  Set Measurement ------------------
    
    %------------------ Cut the measurment to size cutIdx ------------------

    measured_z = sonarStruct.z_positions';
    if cutIdx>length(measured_z)
        cutIdx = length(measured_z);
    end
        % measured_z = measured_z(1:cutIdx,:); %and then measurement will be also cut
        measured_z = measured_z(1:cutIdx); %and then measurement will be also cut
        inputs = inputs(1:cutIdx,:);


    X_intg = objData.positionIntegrated.X_intg; % X position
    Y_intg = objData.positionIntegrated.Y_intg; % Y position
    Z_intg = objData.positionIntegrated.Z_intg; % Y position
    

    %here sonar and dvl measurement differs by only 1-2 elements (neglegtable)
    measurements = {X_intg(1:length(measured_z),:) , Y_intg(1:length(measured_z),:), measured_z};

    measurements{3}(measurements{3} == Inf) = 0;
    measurements{3}(measurements{3} == -Inf) = 0;
    measurements{3}(isnan(measurements{3})) = 0;
    time_measurement = sonarStruct.timestamps(1:length(measured_z)); 




    signal1 = measurements{1};
    signal2 = measurements{2};
    signal3 =  Z_intg(1:length(measured_z),:);
    noisySignal1 =  signal1+ sigma* randn(size(signal1));
    noisySignal2 =  signal2+ sigma* randn(size(signal2));
    noisySignal3 =  signal3+ sigma* randn(size(signal3));

    
    % Compute noise
    noise1 = noisySignal1 - signal1;
    noise2 = noisySignal2 - signal2;
    noise3 = noisySignal3 - signal3;
    
    % Calculate powers
    signalPower1 = mean(signal1.^2);
    signalPower2 = mean(signal2.^2);
    signalPower3 = mean(signal3.^2);

    noisePower1 = mean(noise1.^2);
    noisePower2 = mean(noise2.^2);
    noisePower3 = mean(noise3.^2);
    
    % Calculate SNR in dB
    snr_dB1 = 10 * log10(signalPower1 / noisePower1);
    snr_dB2 = 10 * log10(signalPower2 / noisePower2);
    snr_dB3 = 10 * log10(signalPower3 / noisePower3);



    measurementsStruct = struct( ...
        'timestamps', time_measurement, ...
        'x_positions', noisySignal1,...
        'y_positions', noisySignal2,...
        'z_positions',noisySignal3,...
        'depth', measurements{3}... %ALREADY CONTAINS NOISE FROM SONAR
    );
    %------------------Init SDP class------------------
    x_0 = [0 0 5 0 0 0];
    objSDP = classSDP_GD(A, B, dt, inputs, x_0, measurements);
    
    nx = size(objSDP.A,1);
    ny = 1;
    ntheta = nx*ny;
    

    %------------------Set SDP Noise ------------------

    mu_theta0 = ones(ntheta,1)*0.1;
    mu_eta = ones(ntheta,1)*0.01;
    sigma_v = sigma; %output noise
    
    sigma_wx0       = diag(ones(nx,1)*0.01); %process noise
    sigma_wx        = sigma_wx0*0.01;    

    sigma_theta0    = diag(ones(ntheta,1))*1;
    sigma_eta       = diag(ones(ntheta,1))*0.01; %theta noise*
    
    objSDP.setNoise(mu_theta0, mu_eta, sigma_v, sigma_wx0, sigma_wx, sigma_theta0, sigma_eta);
    


    % % -----------------perform SDP-GD ------------------

    objSDP.performSDP_GD(); %performs C for all 3 measurements
    objSDP.mergeC(); %merge to 1 C
    estimated_C_theta = objSDP.estimated_C_theta;
    nominalX                   =objSDP.nominalX{3};
    timeElapse                   =objSDP.timeElapse;
    % to do: maybe save the whole object itself
    save(CThetaFilePath', 'estimated_C_theta',  'nominalX', 'timeElapse');


    
    % -----------------perform Kalman------------------
    KalmanStruct = PerformKalman( obj_ID, inputs, measurementsStruct, sigma);
    save(SonarMapFilePath, 'KalmanStruct');

end