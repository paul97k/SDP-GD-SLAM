%% Load Simulate Data
clear all
close all
clc
load('simulate_lifted_system.mat');
load('map_estimate_c.mat');
%%
simulate_data = load('simulate_lifted_system.mat');
map_data = load('map_estimate_c.mat');
% Load simulate_lifted_system.mat
simulate_data = load('simulate_lifted_system.mat');

A_1 = simulate_data.A_1;
A_2 = simulate_data.A_2;
B_1 = simulate_data.B_1;
B_2 = simulate_data.B_2;
C = simulate_data.C;
input_signal = simulate_data.input_signal;
mu_x0 = simulate_data.mu_x0;
sigma_wx0 = simulate_data.sigma_wx0;
sigma_theta0 = simulate_data.sigma_theta0;
sigma_wx = simulate_data.sigma_wx;
sigma_v = simulate_data.sigma_v;
sigma_eta = simulate_data.sigma_eta;
mu_theta0 = simulate_data.mu_theta0;
mu_eta = simulate_data.mu_eta;
nT = simulate_data.nT;
nx = simulate_data.nx;
nu = simulate_data.nu;
A_1_part = simulate_data.A_1_part;

%% Example Initialization:

experimentNum = 10;
totalHorizon = 4;
timeStep = 0.1;
estimateDataLen = (totalHorizon/timeStep)+1;
estimateDataLen = str2double(num2str(estimateDataLen, '%.0f'));
constVec = [5, 1.5, 2; 3, 4.5, 3.5];
coefVec = [0.6, 0.4, 0.025; 1.5, 22, 1];
modelExists = true;

matrixAk = [2 0 0; ...
            0 3 0; ...
            0 0 4];
matrixAk = A_2


vecBk = [0; 1; 1];
tfProcessModelDenomCoef = [];
tfProcessModelNomCoef = [];

initX = [1; 0.5; 2];
initCovX = [2.5e-5; 1.5e-4; 1e-4];

timeK = (0.1:timeStep:totalHorizon)';
nominalU = 3.5+cos(2.*timeK);

covW = [1.8e-4; 2.2e-4; 1e-4];
covV = [3e-4; 2e-4];

covC = [0.2; 0.3; 0.4; 0.25; 0.35; 0.45];
desiredSnrC = 30;
desiredSnr = 30;

%% Create Dataset:
createDataset(timeStep, estimateDataLen, constVec, coefVec, covC, ...
    modelExists, matrixAk, vecBk, tfProcessModelDenomCoef, tfProcessModelNomCoef, ...
    experimentNum, initX, nominalU, covW, covV, desiredSnr, desiredSnrC);

%% Set EstimateBias to False:
estimateBias = false;

%% Load Created Data:
filename = ['Instant_', sprintf('%d_samples_%d_snr.mat', estimateDataLen, desiredSnr)];
load(filename);

%% Run mapEstimateC:
[optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigma_v, sigma_wx, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC);
%%





clc




simulate_data = load('simulate_lifted_system.mat');
A_1 = simulate_data.A_1;
A_2 = simulate_data.A_2;
B_1 = simulate_data.B_1;
B_2 = simulate_data.B_2;
C = simulate_data.C;
input_signal = simulate_data.input_signal;
mu_x0 = simulate_data.mu_x0;
sigma_wx0 = simulate_data.sigma_wx0;
sigma_theta0 = simulate_data.sigma_theta0;
sigma_wx = simulate_data.sigma_wx;
sigma_v = simulate_data.sigma_v;
sigma_eta = simulate_data.sigma_eta;
mu_theta0 = simulate_data.mu_theta0;
mu_eta = simulate_data.mu_eta;
nT = simulate_data.nT;
nx = simulate_data.nx;
nu = simulate_data.nu;
A_1_part = simulate_data.A_1_part;




% Load map_estimate_c.mat
map_data = load('map_estimate_c.mat');

% Access the variables
lifted_A = map_data.lifted_A;
nT = map_data.nT;
estimate_bias = map_data.estimate_bias;
sigma_v = map_data.sigma_v;
sigma_wx = map_data.sigma_wx;
lifted_u = map_data.lifted_u;
measured_y = map_data.measured_y;
mu_theta = map_data.mu_theta;
sigma_w_theta = map_data.sigma_w_theta;
size_ck = map_data.size_ck;
desired_snr = map_data.desired_snr;
matrix_scale = map_data.matrix_scale;




% Define Variables
matrix_s = sdpvar(double(nT) / double(size_ck(1)), double(nT) / double(size_ck(1)), 'symmetric');

gamma = sdpvar(1);
beta = sdpvar(1);
vec_theta = sdpvar(size_ck(1) * size_ck(2) * nT, 1);

% Bias variable (if applicable)
if estimate_bias
    bias = sdpvar(size_ck(1), 1);
    vec_bias = repmat(bias, nT, 1);
else
    vec_bias = zeros(size_ck(1) * nT, 1);
end


disp(size(nominalVecC)); % Size of nominalVecC
disp(size(matrixDiff)); % Size of matrixDiff
disp(size(sigmaDeltaC)); % Size of sigmaDeltaC
disp(size(randn(size(nominalVecC, 1), 1))); % Size of the random noise


% Construct C_theta (implement your custom function here)
C_theta = construct_C_theta(vec_theta, size_ck, nT);

disp(class(C_theta)); % Should be 'sdpvar' or 'double'
% Constraints
pwr_s = double((desired_snr - 15) / 5);

sigma_v = double(sigma_v);

% Block Matrix 1



% Ensure all components are numeric or sdpvar
top_left = (10^pwr_s) * (sigma_v - matrix_s); % Example: numeric or sdpvar
top_right = C_theta; % Example: should be sdpvar
bottom_left = C_theta'; % Example: should be sdpvar
bottom_right = -(10^(-pwr_s)) * inv(sigma_wx); % Example: numeric or sdpvar

% Check if all components are valid types
assert(isa(top_left, 'double') || isa(top_left, 'sdpvar'), 'top_left must be numeric or sdpvar');
assert(isa(top_right, 'double') || isa(top_right, 'sdpvar'), 'top_right must be numeric or sdpvar');
assert(isa(bottom_left, 'double') || isa(bottom_left, 'sdpvar'), 'bottom_left must be numeric or sdpvar');
assert(isa(bottom_right, 'double') || isa(bottom_right, 'sdpvar'), 'bottom_right must be numeric or sdpvar');

% Construct the block matrix
block_matrix1 = [top_left, top_right; bottom_left, bottom_right];
constraints = [matrix_s >= 0, block_matrix1 <= 0];

% Block Matrix 2
top_left = -matrix_s;
top_right = measured_y - C_theta * lifted_u;
bottom_left = top_right';
bottom_right = gamma;

block_matrix2 = [top_left, top_right; bottom_left, bottom_right];
constraints = [constraints, block_matrix2 <= 0];

% Block Matrix 3
top_left = -sigma_w_theta;
top_right = vec_theta - mu_theta;
bottom_left = top_right';
bottom_right = beta;

block_matrix3 = [top_left, top_right; bottom_left, bottom_right];
constraints = [constraints, block_matrix3 <= 0, gamma >= 0, beta >= 0];

% Objective Function
cost = trace(matrix_s - eye(size(matrix_s))) + gamma + beta;

% Solve the Problem
options = sdpsettings('solver', 'mosek', 'verbose', 1);
optimize(constraints, cost, options);

% Retrieve Results
optimal_matrix_s = value(matrix_s);
optimal_vec_theta = value(vec_theta);
optimal_value_gamma = value(gamma);
optimal_value_cost = value(cost);
% optimal_bias = value(bias) if estimate_bias else zeros(size_ck(1), 1);


function C_theta = construct_C_theta(vec_theta, size_ck, nT)
    nC0 = size_ck(1);
    nC1 = size_ck(2);
    idx = 1; % MATLAB indexing starts at 1
    C_theta_blocks = cell(nT, nT); % Use a cell array for dynamic block creation

    for i = 1:nT
        for j = 1:nT
            if i == j
                % Extract elements for the diagonal block from vec_theta
                block_size = nC0 * nC1;
                block = reshape(vec_theta(idx:idx + block_size - 1), nC0, nC1);
                C_theta_blocks{i, j} = block;

                % Update the index
                idx = idx + block_size;
            else
                % Off-diagonal blocks are zeros
                if isa(vec_theta, 'sdpvar')
                    C_theta_blocks{i, j} = sdpvar(nC0, nC1); % Zero block for sdpvar
                else
                    C_theta_blocks{i, j} = zeros(nC0, nC1); % Numeric zero block
                end
            end
        end
    end

    % Combine blocks into a single matrix
    C_theta = cell2mat(C_theta_blocks);
end
