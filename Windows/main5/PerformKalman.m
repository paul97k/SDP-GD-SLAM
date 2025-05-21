function KalmanStruct = PerformKalman(obj_ID, inputs, measurementsStruct)

    % -------------------Init Kalman Class-----------------------
    objKal = classKalman(obj_ID.identification);
    
    
    % -------------------Set input and measurement-----------------------
    % objKal.setInputAndMeasurement();
    % input = objKal.u;
    % time = objKal.timeId;
    % z_meas_val = objKal.z_meas_val ; 
    
    Outputs = [measurementsStruct.x_positions measurementsStruct.y_positions measurementsStruct.z_positions];
    

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
    tic
    objKal.PerformEKF(inputs, Outputs, paramaters)
    x_estimates = objKal.x_estimates;       
    timeElapseKalman = toc;
    
    % -------------------Plot result-----------------------

 
    map_By_Kalman  = objKal.CreateKalmanMAP(measurementsStruct.depth, x_estimates);
    Z_est = objKal.CutAndGetNearestDataFromMap(x_estimates, map_By_Kalman);

    
    KalmanStruct = struct( ...
        'timestamps', measurementsStruct.timestamps, ...
        'x', x_estimates(1,:), ...
        'y', x_estimates(2,:), ...
        'z', x_estimates(3,:), ...
        'z_depth', Z_est, ...
        'vx', x_estimates(4,:), ...
        'vy', x_estimates(5,:), ...
        'vz', x_estimates(6,:), ...
        'map_By_Kalman', map_By_Kalman, ...
        'timeElapseKalman', timeElapseKalman ...
    );
    
        % 'vz', x_estimates(6,:) ...
    
end