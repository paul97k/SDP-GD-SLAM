%% Perform Kalman
clc
% close all
clear


% -------------------Init Data folder and location -----------------------
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
    "S0.22_D10_R0.5_D12.mat"]; %#8

%%
clc
% -------------------Init Identified system ---------------------middleIdx--
% obj_ID_ = load('identifiedSystem_Kalman.mat', 'obj');
obj_ID_ = load('identifiedSystem.mat', 'obj');
obj_ID = obj_ID_.obj;
Ts = obj_ID.downSampleFreq;


% -------------------Init Data to perform Kalman on-----------------------
objData = DataLoader(dataFolder_Straight_differentBottoms, Straight_differentBottoms(2)); %folder and fileName
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
clc
% close all

endIdx= length(x_estimates);

skip_i = 2;
skip_j = 20;

map_By_Kalman  = objKal.CreateKalmanMAP(x_estimates, skip_i, skip_j);
[timeGT,timeLoc,...
                  map_pointsGT, ...
                  vxGT, vyGT, ...
                  X_intg, Y_intg, ...
                  xGT, yGT,  Z_AUV,...
                  Z_est, Z_nearest, Z_GT] = objKal.CutAndGetNearestDataFromMap(skip_i, skip_j, x_estimates, map_By_Kalman);
objKal.plotResults(...
                timeGT,timeLoc,...
                z_meas_val, x_estimates,  ...
                  map_By_Kalman, map_pointsGT, ...
                  vxGT, vyGT, ...
                  X_intg, Y_intg, ...
                  xGT, yGT,  Z_AUV,...
                  Z_est, Z_nearest, Z_GT)

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

%% test Kalman Map on different set 
objData = DataLoader(dataFolder_Going_straight, StraightControllerFile(1)); %folder and fileName
objDataTest.downSampleFreq = Ts;
objDataTest.computeNED_AddNoise(true, 0.02);
objDataTest.DownsampleData();
objDataTest.computeXZsonar(true);

% -------------------Init Kalman Class-----------------------
objKalTest = classKalman(objDataTest, obj_ID.identification);

% -------------------Set input and measurement-----------------------
objKalTest.setInputAndMeasurement();
input = objKalTest.u;
time = objKalTest.timeId;
z_meas_valTest = objKalTest.z_meas_val ; 

% -------------------perform EKF using parameters from the previous Kalman filter-----------------------
objKalTest.PerformEKF(input, z_meas_valTest, paramaters)
x_estimatesTest = objKalTest.x_estimates;      

% -------------------Plot result-----------------------

[timeGT,timeLoc,...
                  map_pointsGT, ...
                  vxGT, vyGT, ...
                  X_intg, Y_intg, ...
                  xGT, yGT,  Z_AUV,...
                  Z_est, Z_nearest, Z_GT] = objKalTest.CutAndGetNearestDataFromMap(skip_i, skip_j, x_estimatesTest, map_By_Kalman);

objKalTest.plotResults(...
                timeGT,timeLoc,...
                z_meas_val, x_estimatesTest,  ...
                  map_By_Kalman, map_pointsGT, ...
                  vxGT, vyGT, ...
                  X_intg, Y_intg, ...
                  xGT, yGT,  Z_AUV,...
                  Z_est, Z_nearest, Z_GT)