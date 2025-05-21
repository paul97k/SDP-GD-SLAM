
clc;
clear;
close all

dataFolder_Chirp = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Chirp/';
ChirpFiles = ["chirp0.001-1x.mat", "chirp0.0001-1xyz.mat", "chirp0.0001-1xyz+1.mat"];

dataFolder_Going_straight = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
StraightControllerFile = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat"];

dataFolder_Identification = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Identification';
InputFile = ["u1_0.05.mat", "u1_0.5.mat", "u6_0.5.mat", "u7_0.5.mat"]; %#4

dataFolder_Step_Response = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Step_Response';
StepResponeFile = ["SR0.01.mat", "SR0.1.mat", "SR0.2.mat", "SR0.5.mat", "SR0.7.mat", "SR1.0.mat", "SR1.2.mat"]; %#7

dataFolder_Straight_differentBottoms = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/SDP_GD/';
Straight_differentBottoms = ["S0.22_D10_R0.5_D0_withoutAnything.mat", ...
    "S0.22_D10_R0.5_D0.mat", "S0.22_D10_R0.5_D1.mat", "S0.22_D10_R0.5_D2.mat", "S0.22_D10_R0.5_D3.mat" , ... 
    "S0.22_D10_R0.5_D4.mat", "S0.22_D10_R0.5_D6.mat", "S0.22_D10_R0.5_D8.mat", "S0.22_D10_R0.5_D10.mat", ...
    "S0.22_D10_R0.5_D12.mat"]; %#8
%% For SDP-GD
clc
close all
% obj = DataLoader(dataFolder_Going_straight,StraightControllerFile(1));
identificationDatapath = dataFolder_Chirp + ChirpFiles(3);
obj = DataLoader(identificationDatapath);
% obj.downSampleFreq = 0.5;
obj.downSampleFreq =6;
trusterInput = false;


obj.computeNED_AddNoise(false);
obj.DownsampleData()

maxItt = 300;

% 
% obj.computeXZsonar(true);
% length(obj.identification.Input)

if trusterInput
    obj.identifySystem3(); %using thruster input
    saveName = "trusterInput_identifiedSystem_Ts"+obj.downSampleFreq+ ".mat";
else  
    obj.identifySystem(); %using linear reference input
    saveName = "C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code\identified_systems\" + "identifiedSystem_Ts"+obj.downSampleFreq+ ".mat";  
end
save(saveName, 'obj');
%%

if trusterInput 
    figure()
    plot(obj.identification.Input(:,1))
    hold on
    plot(obj.identification.Input(:,2))
    plot(obj.identification.Input(:,3))
else


end




%%  Model validation 
% clc
load('identifiedSystem2.mat', 'obj');
 % obj.identification.sys_ss
 % close all
 clc
% Ts = obj.downSampleFreq;
Ts = 20;

% loadName ="C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/identified_systems" +  "identifiedSystem_Ts"+Ts+ ".mat";  
% loadName ="trusterInput_identifiedSystem_Ts"+Ts+ ".mat"; 
% objDate = load(saveName, 'obj');
obj = objDate.obj;
% objVal = DataLoader(dataFolder_Chirp,ChirpFiles(2));
objVal = DataLoader(dataFolder_Straight_differentBottoms+ Straight_differentBottoms(1));
objVal.downSampleFreq = Ts;

objVal.computeNED_AddNoise(false);
objVal.DownsampleData();

V_x_new = objVal.NED.V_x;
V_y_new =  objVal.NED.V_y;
V_z_new =  objVal.NED.V_z;
t=  objVal.imu.timestamps;

V_x_new = V_x_new(:);
V_y_new = V_y_new(:);
V_z_new = V_z_new(:);


X_intg = objVal.positionIntegrated.X_intg; % X position
Y_intg = objVal.positionIntegrated.Y_intg; % Y position
Z_intg = objVal.positionIntegrated.Z_intg; % Y position

if trusterInput
    Input = [obj.thrusters.input.thruster0 obj.thrusters.input.thruster1 obj.thrusters.input.thruster2];
        figure()
    plot(objVal.identification.Input(:,1))
    hold on
    plot(objVal.identification.Input(:,2))
    plot(objVal.identification.Input(:,3))
else
    zeroInput = objVal.CMD.cmd_linear_x*0 ;
    Input = [zeroInput zeroInput zeroInput objVal.CMD.cmd_linear_x  objVal.CMD.cmd_linear_y objVal.CMD.cmd_linear_z];
    figure()
    plot(Input(:,4))
    hold on
    plot(Input(:,5))
    plot(Input(:,6))
end

Output = [X_intg Y_intg Z_intg V_x_new V_y_new V_z_new];
% 
if size(Input,1) > size(Output,1)
    Input = Input(1:size(Output,1),:);
elseif size(Input,1) < size(Output,1)
    Output = Output(1:size(Input,1),:);
end



data = iddata(Output, Input, 1/Ts);
%

T = objVal.CMD.cmd_timestamps;

t_sim = (min(T):1/Ts:max(T))'; % Resample time vector with step Ts
t_sim = t_sim(1:length(data.InputData));


%%

figure;
[ymod,fit,ic] =compare(data, obj.identification.sys_ss);
compare(data, obj.identification.sys_ss)
title('Model Validation');
disp(['fit: ', num2str(fit')]);

% Compute the error
error = Output - ymod.OutputData;
disp(['Mean Absolute Error: ', num2str(mean(abs(error)))]);
disp(['Root Mean Square Error (RMSE): ', num2str(sqrt(mean(error.^2)))]);





fprintf('Size of data.InputData: %d x %d\n', size(data.InputData,1), size(data.InputData,2));
fprintf('Size of T: %d x %d\n', size(t_sim,1), size(t_sim,2));

% Simulate the system with the correct time vector
y_sim = lsim(obj.identification.sys_ss, data.InputData, t_sim );
% Plot results


error = data.OutputData - y_sim; % Compute residual error

rmse = sqrt(mean(error.^2));
mae = mean(abs(error));
max_error = max(abs(error));

% Display Results
fprintf('\nModel Validation Statistics:\n');
fprintf('RMSE: %.4f\n', rmse);
fprintf('MAE: %.4f\n', mae);
fprintf('Max Absolute Error: %.4f\n', max_error);

num_outputs = size(data.OutputData, 2); % Number of output variables
correlation_matrix = zeros(num_outputs, 1); % Store correlation values

for i = 1:num_outputs
    corr_matrix = corrcoef(data.OutputData(:, i), y_sim(:, i)); % Compute correlation
    correlation_matrix(i) = corr_matrix(1,2); % Extract correlation coefficient
end

% Display results
for i = 1:num_outputs
    fprintf('Correlation for Output %d: %.4f\n', i, correlation_matrix(i));
end
%% Kalman Identification

clc;
clear;
close all

dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/data/npz';
dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Identification';

InputFiles = ["u1_0.05.mat", "u1_0.5.mat", "u6_0.5.mat", "u7_0.5.mat"];

dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Chirp';
InputFiles = ["chirp0.001-1x.mat", "chirp0.0001-1xyz.mat", "chirp0.0001-1xyz+1.mat"];


% dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
% InputFiles = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat"];


fileName = InputFiles(1);


obj = DataLoader(dataFolder,fileName);
obj.downSampleFreq = 10;
obj.computeNED_AddNoise(false);
obj.DownsampleData()

x_0 = [0; 0; 0];
% 
x_0 = [0; 0; 0];
APar = [    0.0001    1.0887   -0.2821   -0.6347    0.1519    0.6320;
    0.0000   -0.0044   -0.0028   -0.0005   -0.0037    0.0013;
   -0.0000   -0.0085   -0.0041   -0.0021    0.0000    0.0149;
    0.0000   -0.0120    0.0027   -0.0011    0.0087    0.0390;
    0.0000    0.0017    0.0018    0.0004    0.0024    0.0002;
    0.0000   -0.0065   -0.0026   -0.0010   -0.0014    0.0093]'*1000;
BPar =     [0.4449   0 0 0 0 0];

% [row, col, APar] = find(APar);
% APar = APar';
APar = APar(:)';
% size(APar)

% APar = zeros(1,36);
% BPar = zeros(1,6);
maxItt = 150;
obj.identifySystem2(x_0,APar, BPar, maxItt);
obj.computeXZsonar();
length(obj.identification.Input)
% save('identifiedSystem_Kalman.mat', 'obj');




%%  Model validation Kalman
load('identifiedSystem_Kalman.mat', 'obj');
obj.identification.sys_ss
% load('identifiedSystem.u7_0.5.mat', 'obj');
dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
StraightControllerFile = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat"];
% 
% dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Step_Response';
% StepResponeFile = ["SR0.01.mat", "SR0.1.mat", "SR0.2.mat", "SR0.5.mat", "SR0.7.mat", "SR1.0.mat", "SR1.2.mat"]; %#7
% % 
% dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Identification';
% InputFile = [ "u7_0.5.mat"]; %#4
% 
% dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Chirp';
% InputFile = ["chirp0.001-1x.mat", "chirp0.0001-1xyz.mat", "chirp0.0001-1xyz+1.mat"];
% 



Ts = obj.downSampleFreq;

fileName = StraightControllerFile(1);
objVal = DataLoader(dataFolder, fileName);
objVal.downSampleFreq = Ts;
objVal.computeNED_AddNoise(true, 0.02);
objVal.DownsampleData();
objVal.computeNED_AddNoise(true);

% 
% input = [objVal.CMD.cmd_linear_x objVal.CMD.cmd_linear_y objVal.CMD.cmd_linear_z ];
% 
% output = [objVal.NED.V_x objVal.NED.V_y objVal.NED.V_z ];
            V_x_new = objVal.NED.V_x;
            V_y_new =  objVal.NED.V_y;
            t=  objVal.imu.timestamps;
        
            V_x_new = V_x_new(:);
            V_y_new = V_y_new(:);

            % t = obj.identification.timestamps;
            X_intg = cumtrapz(t, V_x_new); % X position
            Y_intg = cumtrapz(t, V_y_new); % Y position
% input = [objVal.CMD.cmd_linear_x*0  objVal.CMD.cmd_linear_y*0 objVal.CMD.cmd_linear_x  objVal.CMD.cmd_linear_y];
% output = [X_intg Y_intg V_x_new V_y_new];

% 
Input = [objVal.CMD.cmd_linear_x objVal.CMD.cmd_linear_y objVal.CMD.cmd_linear_z ...
         objVal.CMD.cmd_angular_x objVal.CMD.cmd_angular_y objVal.CMD.cmd_angular_z];

Output = [objVal.NED.V_x objVal.NED.V_y objVal.NED.V_z ...
          objVal.imu.rolls objVal.imu.pitches objVal.imu.yaws];
size(Input,1)
size(Output,1)
if size(Input,1) > size(Output,1)
    Input = Input(1:size(Output,1),:);
elseif size(Input,1) < size(Output,1)
    Output = Output(1:size(Input,1),:);
end
size(Input,1)
size(Output,1)


data = iddata(Output, Input, 1/Ts);
figure;
[ymod,fit,ic] =compare(data, obj.identification.sys_ss);
compare(data, obj.identification.sys_ss)
title('Model Validation');


T = objVal.CMD.cmd_timestamps;

t_sim = (min(T):1/Ts:max(T))'; % Resample time vector with step Ts
t_sim = t_sim(1:length(data.InputData));

fprintf('Size of data.InputData: %d x %d\n', size(data.InputData,1), size(data.InputData,2));
fprintf('Size of T: %d x %d\n', size(t_sim,1), size(t_sim,2));
%
% Simulate the system with the correct time vector
y_sim = lsim(obj.identification.sys_ss, data.InputData, t_sim );
% % Plot results
% figure;
% hold on;
% plot(t_sim, input, 'LineWidth', 1.5); % Simulated output
% % plot(t_sim, obj.identification.Output, '--', 'LineWidth', 1.5); % Actual data for comparison
% plot(t_sim, y_sim, 'LineWidth', 1.5); % Simulated output
% xlabel('Time (s)');
% ylabel('System Output');
% % title('Comparison of Simulated and Measured Output');
% % legend('Simulated x', 'Simulated y' ,'Simulated z');
% title('Input to the system');
% legend('Input x', 'Input y' ,'Input z');
% % % Compute Error Metrics

error = data.OutputData - y_sim; % Compute residual error

rmse = sqrt(mean(error.^2));
mae = mean(abs(error));
max_error = max(abs(error));

% Display Results
fprintf('\nModel Validation Statistics:\n');
fprintf('RMSE: %.4f\n', rmse);
fprintf('MAE: %.4f\n', mae);
fprintf('Max Absolute Error: %.4f\n', max_error);

num_outputs = size(data.OutputData, 2); % Number of output variables
correlation_matrix = zeros(num_outputs, 1); % Store correlation values

for i = 1:num_outputs
    corr_matrix = corrcoef(data.OutputData(:, i), y_sim(:, i)); % Compute correlation
    correlation_matrix(i) = corr_matrix(1,2); % Extract correlation coefficient
end

% Display results
for i = 1:num_outputs
    fprintf('Correlation for Output %d: %.4f\n', i, correlation_matrix(i));
end
%% 
clc
close all
load('identifiedSystem.mat', 'obj');
% Save object data
% Extract system parameters
A = obj.identification.A
B = obj.identification.B
C = obj.identification.C
D = obj.identification.D
% A =[0 0 0;0 0 0; 0 0 0];
% B =[0.8905 0 0;
%     0  0 0; 
%     0 0 0];

u = obj.identification.Input; % Input matrix
T = obj.identification.timestamps; % Time vector
Ts = 1/obj.downSampleFreq ; % Sample time


% Define state-space model
sys = ss(A, B, C, D, Ts, 'StateName', {'X', 'Y', 'Z','phi', 'theta', 'psi' }, 'InputName', 'Force');
% sys = ss(A, B, C, D, Ts, 'StateName', {'X', 'Y', 'Z'}, 'InputName', 'Force');



T_resampled = (min(T):Ts:max(T))'; % Resample time vector with step Ts
u_resampled = interp1(T, u, T_resampled, 'linear'); % Interpolate input to match new time

[y, T_out, x] = lsim(sys, u_resampled, T_resampled);
output_measured = obj.identification.Output(:,1);
output_t_measured = obj.identification.timestamps;
figure;

% plot(T_out, y(:,1), 'LineWidth', 2);
hold on
% plot(output_t_measured, output_measured, 'LineWidth', 2);
plot(T_out, u_resampled(:,1), 'LineWidth', 2);
plot(T_out, u_resampled(:,2), '--','LineWidth', 2);
plot(T_out, u_resampled(:,3), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Amplitude');
title('System Input');
% legend(' estimate output', 'measeured output')
legend('x reference', 'y reference', 'z reference')
grid on;






%% Perform Kalman
clc
close all
clear
obj_ID_ = load('identifiedSystem_Kalman.mat', 'obj');
obj_ID = obj_ID_.obj;
% load('identifiedSystem.u7_0.5.mat', 'obj');
dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
StraightControllerFile = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat"];

dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Step_Response';
StepResponeFile = ["SR0.01.mat", "SR0.1.mat", "SR0.2.mat", "SR0.5.mat", "SR0.7.mat", "SR1.0.mat", "SR1.2.mat"]; %#7

% dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Identification';
InputFile = ["u1_0.05.mat", "u1_0.5.mat", "u6_0.5.mat", "u7_0.5.mat"]; %#4
Ts = obj_ID.downSampleFreq;

fileName = StepResponeFile(5);
objKal = DataLoader(dataFolder, fileName);
objKal.downSampleFreq = Ts;
objKal.computeNED_AddNoise(true, 0.02);
objKal.DownsampleData()

% input = [objKal.CMD.cmd_linear_x objKal.CMD.cmd_linear_y objKal.CMD.cmd_linear_z];
% time = objKal.CMD.cmd_timestamps;
% output = [objKal.NED.V_x objKal.NED.V_y objKal.NED.V_z];



id_time = objKal.CMD.cmd_timestamps;
V_x_new = objKal.NED.V_x;
V_y_new =  objKal.NED.V_y;
V_z_new =  objKal.NED.V_z;

rolls = objKal.imu.rolls;
pitches =  objKal.imu.pitches;
yaws =  objKal.imu.yaws;

% Ensure column vectors
id_time = id_time(:);
V_x_new = V_x_new(:);
V_y_new = V_y_new(:);
V_z_new = V_z_new(:);


Input = [objKal.CMD.cmd_linear_x  objKal.CMD.cmd_linear_y objKal.CMD.cmd_linear_z ...
    objKal.CMD.cmd_angular_x objKal.CMD.cmd_angular_y objKal.CMD.cmd_angular_z];
time = id_time;
% Output = [V_x_new V_y_new V_z_new rolls pitches yaws];
Output = [V_x_new V_y_new V_z_new rolls pitches yaws];

% 
% size(input,1)
% size(time)
% size(output,1)

if size(Input,1) > size(Output,1)
    Input = Input(1:size(Output,1),:);
    time = time(1:size(Output,1));
elseif size(Input,1) < size(Output,1)
    Output = Output(1:size(Input,1),:);
end
% size(input,1)
% size(output,1)

u = Input;
z_meas_val = Output;

Q = diag([0.001,0.001,0.001,0.001,0.001,0.01 ,0.01,0.01,0.01]); % Process noise
R = diag([0.0001,0.002,0.005 ,0.1,0.02,0.5]);  % Measurement noise
x_0 = zeros(9,1);
P = eye(size(Q,1));    
obj_ID.PerformEKF(P, Q, R, u, z_meas_val, x_0)



% Covariance Matrix (Initial Uncertainty)
       
dt = obj_ID.identification.Ts;
A = obj_ID.identification.A;
B = obj_ID.identification.B;


Vx = z_meas_val(:,1);  % X velocity
Vy = z_meas_val(:,2);  % Y velocity
t = time;

% Integrate velocity to get position
X_intg = cumtrapz(t, Vx); % X position
Y_intg = cumtrapz(t, Vy); % Y position
                    

x_estimates = obj_ID.EKF.x_estimates;
l_xest= length(x_estimates);

timeGT = objKal.positionGT.timestamps(1:l_xest,:);
xGT = objKal.positionGT.x_positions(1:l_xest,:);
yGT = objKal.positionGT.y_positions(1:l_xest,:);
vxGT = objKal.velocityGT.x_vel(1:l_xest,:); 
vyGT = objKal.velocityGT.y_vel(1:l_xest,:);



figure;
set(gcf, 'Position', [0, 0, 2000, 1000]); % (x, y, width, height)
% plot position
    subplot(3,2,1);
    plot(timeGT, xGT, 'b', 'LineWidth', 1.5); hold on;
    plot(time, X_intg, 'g', 'LineWidth', 1.5); hold on;
    plot(time, x_estimates(1,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time Step(k*dt)'); ylabel('X position');
    legend('True Position','inegrate Position', 'EKF Estimate');
    title('Position Estimation in X');
    
    subplot(3,2,2);
    plot(timeGT, yGT, 'b', 'LineWidth', 1.5); hold on;
    plot(time, Y_intg, 'g', 'LineWidth', 1.5); hold on;
    plot(time, x_estimates(2,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time Step(k*dt)'); ylabel('Y position');
    legend('True Position','inegrate Position', 'EKF Estimate');
    title('Position Estimation in Y');

% plot error
    subplot(3,2,3);
    plot(timeGT, abs(xGT-X_intg), 'g', 'LineWidth', 1.5); hold on;
    plot(timeGT, abs(xGT-x_estimates(1,:)') , 'r--', 'LineWidth', 1.5);
    xlabel('|Error Step|'); ylabel('X position');
    legend('Error inegrate Position', 'Error  EKF Estimate');
    title('Absolute Error in X');
    
    subplot(3,2,4);
    plot(timeGT, abs(yGT-Y_intg), 'g', 'LineWidth', 1.5); hold on;
    plot(timeGT, abs(yGT-x_estimates(2,:)'), 'r--', 'LineWidth', 1.5);
    xlabel('Time Step(k*dt)'); ylabel('Y position');
    legend('Error inegrate Position', 'Error EKF Estimate');
    title('Absolute  Error in Y');

% plot velocity  
    subplot(3,2,5);
    plot(timeGT, vxGT, 'b', 'LineWidth', 1.5); hold on;
    plot(timeGT, z_meas_val(:,1), 'g', 'LineWidth', 1.5); hold on;
    plot(timeGT, x_estimates(4,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time Step(k*dt)'); ylabel('X velocity');
    legend('GT Velocity', 'meas Velocity','EKF Estimate');
    title('Velocity Estimation in X');
    % xlim([50,100])
    
    subplot(3,2,6);
    plot(timeGT, vyGT, 'b', 'LineWidth', 1.5); hold on;
    plot(timeGT, z_meas_val(:,2), 'g', 'LineWidth', 1.5); hold on;
    plot(timeGT, x_estimates(5,:), 'r--', 'LineWidth', 1.5);
    xlabel('Time Step(k*dt)'); ylabel('Y velocity');
    legend('GT Velocity', 'meas Velocity', 'EKF Estimate');
    title('Velocity Estimation in Y');
    % xlim([50,100])
%% create slam Kalman Map

clc
close all

objKal.computeXZsonar();
skip_i = 2;
skip_j = 20;

map_points  = objKal.CreateKalmanMAP(x_estimates, skip_i, skip_j);
x_estimatesGTXY = x_estimates;
x_estimatesGTXY(1,:) =xGT';
x_estimatesGTXY(2,:) =yGT';
map_pointsGT  = objKal.CreateKalmanMAP(x_estimatesGTXY, skip_i, skip_j);
%% test Kalman Map
close all
clc
Z_est =     findClosestZ(x_estimates(1,:)   ,x_estimates(2,:)   , map_points);
Z_nearest = findClosestZ(X_intg             , Y_intg            , map_points);
Z_GT =      findClosestZ(xGT                ,yGT                , map_pointsGT);

Z_AUV = 0;

% Plot the results
figure;
scatter3(map_points(1,:), map_points(2,:), map_points(3,:), 5, 'b', 'filled'); 
hold on;
scatter3(map_pointsGT(1,:), map_pointsGT(2,:), map_pointsGT(3,:), 5, 'r', 'filled'); 

scatter3(x_estimates(1,:),x_estimates(2,:), Z_est, 20, 'y', 'filled'); % Red points for matched Y, Z
scatter3(x_estimates(1,:),x_estimates(2,:), zeros(size(x_estimates(2,:))), 20, 'y', 'filled'); % Red points for matched Y, Z

scatter3(X_intg, Y_intg, Z_nearest, 20, 'g', 'filled'); % Red points for matched Y, Z
scatter3(X_intg, Y_intg,  zeros(size(Y_intg)), 20, 'g', 'filled'); % Red points for matched Y, Z

scatter3(xGT, yGT, Z_GT, 20, 'r', 'filled'); % Red points for matched Y, Z
scatter3(xGT, yGT, zeros(size(yGT)), 20, 'r', 'filled'); %
% Red points for matched Y, Z
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Closest Y and Z points from Map');
grid on;
legend('Map Points', 'Postion using EKF', '.', 'Postion using integration', '.', 'Ground truth Postion', '.');


%% test Kalman Map on different set 
close all

dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Going_straight';
StraightControllerFile = ["SC11.mat","SC12.mat","SC13.mat","SC22.mat","SC23.mat"];

dataFolder = 'C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code/controller_mattfile2/Step_Response';
StepResponeFile = ["SR0.01.mat", "SR0.1.mat", "SR0.2.mat", "SR0.5.mat", "SR0.7.mat", "SR1.0.mat", "SR1.2.mat"]; %#7

fileName = StepResponeFile(6);
objVal = DataLoader(dataFolder, fileName);
objVal.downSampleFreq = Ts;

objVal.computeNED_AddNoise(true);
objVal.DownsampleData();
maxXMap =  max(map_points(1,:));


X_intg = objVal.positionIntegrated.X_intg;
Y_intg = objVal.positionIntegrated.Y_intg;
xGT = objVal.positionGT.x_positions;
yGT = objVal.positionGT.y_positions;



clc
Z_nearest = findClosestZ(X_intg             , Y_intg            , map_points);

Z_AUV = 0;
x_estimatesGTXY = zeros(9,length(xGT));
x_estimatesGTXY(1,:) =xGT';
x_estimatesGTXY(2,:) =yGT';

objVal.computeXZsonar();
map_pointsGT  = objVal.CreateKalmanMAP(x_estimatesGTXY, skip_i, skip_j);


Z_nearest_ = findClosestZ(X_intg             , Y_intg            , map_points);
Z_GT =      findClosestZ(xGT                ,yGT                , map_pointsGT);



% Plot the results
figure;
scatter3(map_points(1,:), map_points(2,:), map_points(3,:), 5, 'b', 'filled'); 
hold on;
scatter3(map_pointsGT(1,:), map_pointsGT(2,:), map_pointsGT(3,:), 5, 'r', 'filled'); 
scatter3(X_intg, Y_intg, Z_nearest, 20, 'g', 'filled'); % Red points for matched Y, Z
scatter3(X_intg, Y_intg,  zeros(size(Y_intg)), 20, 'g', 'filled'); % Red points for matched Y, Z

scatter3(xGT, yGT, Z_GT, 20, 'r', 'filled'); % Red points for matched Y, Z
scatter3(xGT, yGT, zeros(size(yGT)), 20, 'r', 'filled'); %
Red points for matched Y, Z
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Closest Y and Z points from Map');
grid on;
legend('Map Points', 'Postion using EKF', '.', 'Postion using integration', '.', 'Ground truth Postion', '.');
%%


function [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = simulate_lifted_system(...
    A_1, A_2, B_1, B_2, C, inputs, mu_x0, sigma_wx0, sigma_theta0, sigma_wx, sigma_v, sigma_eta, mu_theta0, mu_eta, nT, nx, nu, A_1_part)

    % Initialize A and B matrices
    A_matrices = repmat(A_2, 1, 1, nT);
    B_matrices = repmat(B_2, 1, 1, nT);

    % Replace first A_1_part entries with A_1 and B_1
    if A_1_part > 0
        A_matrices(:, :, 1:A_1_part) = repmat(A_1, 1, 1, A_1_part);
        B_matrices(:, :, 1:A_1_part) = repmat(B_1, 1, 1, A_1_part);
    end

    % Create repeated C matrices
    C_matrices = repmat(C, 1, 1, nT);

    % Process and measurement noise covariances
    process_noise_cov = repmat(sigma_wx, 1, 1, nT);
    measurement_noise_cov = repmat(sigma_v, 1, 1, nT);
    eta_noise_cov = repmat(sigma_eta, 1, 1, nT);

    % Call create_lifted_process_observation_model function
    [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
        create_lifted_process_observation_model(A_matrices, B_matrices, C_matrices, inputs, ...
        process_noise_cov, measurement_noise_cov, eta_noise_cov, ...
        mu_x0, sigma_wx0, sigma_theta0, sigma_eta, mu_theta0, mu_eta);
end





