%% Example:
clear all
close all
clc

experimentNum = 10;
totalHorizon = 100;
timeStep = 0.1;
estimateDataLen = (totalHorizon/timeStep)+1;
estimateDataLen = str2double(num2str(estimateDataLen, '%.0f'));
constVec = [5, 1.5, 2; 3, 4.5, 3.5];
coefVec = [0.6, 0.4, 0.025; 1.5, 22, 1];
modelExists = true;

matrixAk = [0.7 0.25 0; ...
            0 0.5 0; ...
            0 0.25 0.7];

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
%%
clc
createDataset(timeStep, estimateDataLen, constVec, coefVec, covC, ...
        modelExists, matrixAk, vecBk, tfProcessModelDenomCoef, tfProcessModelNomCoef, ...
        experimentNum, initX, nominalU, covW, covV, desiredSnr, desiredSnrC);





%% Set EstimateBias to False:
clc
% clear
load('Instant_1001_samples_30_snr.mat');

find_min_max_nonzero(matrixA)

%% Load Created Data:
clc
desiredSnrC = 0;
desiredSnr = 0;
estimateBias = false;
filename = ['Instant_', sprintf('%d_samples_%d_snr.mat', estimateDataLen, desiredSnr)];
load(filename);
matrixScale = 1;

% 
% You can now see how the input to the following function look like:
[optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC)



%% 9. Reconstruct Parameters

% 
nominalVecC = repmat(C, 1, nT); 
nominalVecC = reshape(nominalVecC', [], 1);
nominalVecC = double(nominalVecC);



estimateBias = false;
sigmaV = sigma_v;
sigmaX= sigma_wx;
nominalX= lifted_A*lifted_u; 
measuredY= measured_y; 
sigmaC= sigma_w_theta; %sigmaw_theta
sizeCk= size(C); 
desiredSnr= 10; 
matrixScale= 1 ; 
trueVecC= [] ; 

[initialVecVar, optimal_vec_theta, predicted_y, residuals] = reconstruct_and_evaluate(...
    optimalVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y, ...
    sigma_v, sigma_wx, mu_theta, sigma_w_theta, @map_cost_func);



analyze_and_visualize_comparison(initialVecVar, optimalVecVar, sizeCk, nT, ...
    estimate_bias, nominalX, measured_y, sigma_v, sigma_wx, mu_theta, ...
    sigma_w_theta, @map_cost_func);





function analyze_and_visualize_comparison(initialVecVar, optimalVecVar, sizeCk, nT, ...
    estimate_bias, nominalX, measured_y, sigma_v, sigma_wx, mu_theta, ...
    sigma_w_theta, map_cost_func)

    % 1. Reconstruct Parameters for SDP
    [sdp_vec_theta, sdp_predicted_y, sdp_residuals] = reconstruct_and_evaluate(...
        initialVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y, ...
        sigma_v, sigma_wx, mu_theta, sigma_w_theta, map_cost_func);

    % 2. Reconstruct Parameters for GD
    [gd_vec_theta, gd_predicted_y, gd_residuals] = reconstruct_and_evaluate(...
        optimalVecVar, sizeCk, nT, estimate_bias, nominalX, measured_y, ...
        sigma_v, sigma_wx, mu_theta, sigma_w_theta, map_cost_func);

    % 3. Visualization: Measured vs. Predicted Outputs
    figure;
    plot(measured_y, 'o-', 'DisplayName', 'Measured Output');
    hold on;
    plot(sdp_predicted_y, 'x-', 'DisplayName', 'SDP Predicted Output');
    plot(gd_predicted_y, '*-', 'DisplayName', 'GD Predicted Output');
    xlabel('Time Step');
    ylabel('Output');
    title('Measured vs. Predicted Output (SDP vs. GD)');
    legend;
    grid on;

    % 4. Visualization: Residuals
    figure;
    plot(sdp_residuals, '.-', 'DisplayName', 'SDP Residuals');
    hold on;
    plot(gd_residuals, 's-', 'DisplayName', 'GD Residuals');
    xlabel('Time Step');
    ylabel('Residual');
    title('Error Over Time (SDP vs. GD)');
    legend;
    grid on;

    % 5. Residual Statistics
    sdp_residual_mean = mean(sdp_residuals);
    sdp_residual_variance = var(sdp_residuals);
    gd_residual_mean = mean(gd_residuals);
    gd_residual_variance = var(gd_residuals);

    disp(['SDP Residual Mean: ', num2str(sdp_residual_mean)]);
    disp(['SDP Residual Variance: ', num2str(sdp_residual_variance)]);
    disp(['GD Residual Mean: ', num2str(gd_residual_mean)]);
    disp(['GD Residual Variance: ', num2str(gd_residual_variance)]);

    % 6. Compare Parameters (if needed)
    % You can extend this part if comparing the parameters between SDP and GD is required.
end


function [optimal_vec_theta, predicted_y, residuals] = reconstruct_and_evaluate(...
    optimal_vec_var, size_ck, nT, estimate_bias, nominalX, measured_y, ...
    sigma_v, sigma_wx, mu_theta, sigma_w_theta, map_cost_func)
    
    % Extract sizes
    vec_theta_size = prod(size_ck) * nT;
    optimal_vec_theta = reshape(optimal_vec_var(1:vec_theta_size), [], 1);
    
    % Reconstruct `C_theta`
    estimated_C_theta = construct_C_theta(optimal_vec_theta, size_ck, nT);
    
    % Bias estimation
    if estimate_bias
        optimal_bias = optimal_vec_var(vec_theta_size + (1:size_ck(1)));
        vec_bias = repmat(optimal_bias, nT, 1);
    else
        vec_bias = zeros(size_ck(1) * nT, 1);
    end
    
    % Predicted output and residuals
    predicted_y = estimated_C_theta * nominalX + vec_bias;
    residuals = measured_y - predicted_y;
    

end


function C_theta = construct_C_theta(vec_theta, size_ck, nT)
    nC0 = size_ck(1);
    nC1 = size_ck(2);
    C_theta_blocks = cell(nT, nT);
    
    idx = 1; % Start index for extracting elements
    
    for i = 1:nT
        for j = 1:nT
            if i == j
                % Extract diagonal block
                block_size = nC0 * nC1;
                block_elements = vec_theta(idx:idx + block_size - 1);
                block = reshape(block_elements, nC0, nC1);
                idx = idx + block_size;
            else
                % Off-diagonal blocks are zeros
                block = zeros(nC0, nC1);
            end
            C_theta_blocks{i, j} = block;
        end
    end
    
    % Construct block matrix
    C_theta = cell2mat(C_theta_blocks);
end


function find_min_max_nonzero(matrix)

    % Ensure we are working with absolute values
    abs_elements = abs(matrix(:));
    
    % Exclude zeros for the lowest element
    non_zero_elements = abs_elements(abs_elements > 0);

    % Find the lowest element (smallest non-zero absolute value)
    lowest_element = min(non_zero_elements);

    % Find the highest element (largest absolute value)
    highest_element = max(abs_elements);

    % Display results
    disp(['Lowest element: ', num2str(lowest_element)]);
    disp(['Highest element: ', num2str(highest_element)]);
end
