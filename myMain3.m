
% close all
clear
clc
load('identifiedSystem.mat', 'obj');


% close all
A_ = obj.identification.A; 
B_ = obj.identification.B;
dt = obj.identification.Ts;


process_noise_var = obj.identification.sys_ss.NoiseVariance;

inputs = obj.identification.Input;
Vx = obj.identification.Output(:,1);  % X velocity
Vy = obj.identification.Output(:,2);  % Y velocity
t = obj.identification.timestamps;
X_intg = cumtrapz(t, Vx); % X position
Y_intg = cumtrapz(t, Vy); % Y position

middleIdx = (size(obj.sonarPositions.z_positions,2))/2;
measured_z = obj.sonarPositions.z_positions(:,middleIdx);

measurment = [X_intg Y_intg measured_z];
% measurment = [ X_intg Y_intg];
% measurment = [measured_z];
% 


cutIdx = 56;
measurment = measurment(1:cutIdx,:);


inputs = inputs(1:cutIdx,:);
measured_yT = measurment';
measurments = measured_yT(:);

A = [
    1 0 dt 0;
    0 1 0 dt;
    0 0 0 0;
    0 0 0 0];
B = [0;0 ; 0.55 ; 0.05];



nT = length(inputs);
nx = size(A,1);
nu = size(B,2);
ny = size(measurment,2);
ntheta = ny*nx;
% ntheta = nnz(C);

x_0 = [0 0 0.55 0];

if size(measurment,2)==1 %Z
    C = [0.0446   -0.0791   -8.7852   -0.0024];
    sigma_v = 0.001;
elseif size(measurment,2)==2 %XY
    C = [0.8755   -0.0118    0.9944   -0.0011;
        0.7704   -0.0210    0.9836   -0.0017];
    sigma_v = diag([0.001 0.001]);
else        %XYZ
    % C = [0.8755   -0.0118    0.9944   -0.0011;
    %     0.7704   -0.0210    0.9836   -0.0017;
    %     0.8541   -0.0152   0        0];
    regT = 0.01;
    C = [0.8755   -0.0118    0.9944   -0.0011;
        0.7704   -0.0210    0.9836   -0.0017;
        0.0446   -0.0791   -8.7852   -0.0024];
    sigma_v = diag([0.001 0.001 0.001]);
end

%still to do

mu_x0 = x_0;
mu_theta0 = ones(ntheta,1)*0.2;
mu_eta = ones(ntheta,1) * 0.3;


sigma_wx0       = diag(ones(nx,1)*0.1); %process noise from identification 
sigma_wx        = sigma_wx0;    
sigma_theta0    = diag(ones(ntheta,1)*0.1);
sigma_eta       = sigma_theta0; %theta noise*

process_noise_cov = repmat(sigma_wx, 1, 1, nT);
measurement_noise_cov = repmat(sigma_v, 1, 1, nT);
eta_noise_cov = repmat(sigma_eta, 1, 1, nT);

% 7. MAP Estimation
size_ck = size(C);
desired_snr = 20;
matrix_scale = 0;
true_vec_theta = [];
estimate_bias = false;

A_matrices = repmat(A, 1, 1, nT);
B_matrices = repmat(B, 1, 1, nT);
C_matrices = repmat(C, 1, 1, nT);


% 6. Lifted System Simulation

[lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
    create_lifted_process_observation_model(A_matrices, B_matrices, C_matrices, inputs, ...
    process_noise_cov, measurement_noise_cov, eta_noise_cov, ...
    mu_x0, sigma_wx0, sigma_theta0, sigma_eta, mu_theta0, mu_eta);


nominalVecC = repmat(C, 1, nT); 
nominalVecC = reshape(nominalVecC', [], 1);
nominalVecC = double(nominalVecC);


sigmaV = sigma_v;
sigmaC= sigma_w_theta; %sigmaw_theta should not be zero
sigmaX= sigma_wx;
estimateBias = false;

nominalX= lifted_A*lifted_u;
measuredY= measurments; 
sizeCk= size(C); 
desiredSnr= 10; 
matrixScale= 1; 
trueVecC= [] ; 
measuredY = double(measuredY);
measuredY(measuredY > 1000) = 5;
measuredY(measuredY < -1000) = -5;
%
% [initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
%     nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC);
%
[~, gd_predicted_y, residuals_gd, estimated_C_theta] = reconstruct_and_evaluate(... 
    optimalVecVar, sizeCk, nT, estimate_bias, nominalX, measuredY);
%
        % Cz = load('myMain3_estimated_C_theta_Z.mat', 'estimated_C_theta');
        % for i = 3:3:size(estimated_C_theta,1)
        %     estimated_C_theta(i,:) = Cz.estimated_C_theta(i/3,:);
        % end
        % 
        % % Predicted output and residuals
        % gd_predicted_y2 = estimated_C_theta * nominalX;



% clc
nT = size(gd_predicted_y,1)/ny;

%
measurment_normal = reshape(measuredY,    ny, nT)';
gd_predicted_y_norm =    reshape(gd_predicted_y, ny, nT)';
X_norm =            reshape(nominalX,       nx, nT)';


if ny ==3
    % gd_predicted_y_norm2 =    reshape(gd_predicted_y2, ny, nT)';
    plot_comparison(X_norm, measurment_normal, gd_predicted_y_norm)
    % plot_comparison(X_norm, measurment_normal, gd_predicted_y_norm2)
elseif ny==2
    plot_comparison_two(X_norm, measurment_normal, gd_predicted_y_norm)
else
    save('myMain3_estimated_C_theta_Z.mat', 'estimated_C_theta');
    plot_comparison_one(X_norm, measurment_normal, gd_predicted_y)
end








%%
clc
measurment_normal = reshape(measuredY,    ny, nT)';
X_norm =            reshape(nominalX,       nx, nT)';

Cx = load('myMain3_estimated_C_theta_X.mat', 'estimated_C_theta');
Cy = load('myMain3_estimated_C_theta_Y.mat', 'estimated_C_theta');
Cz = load('myMain3_estimated_C_theta_Z.mat', 'estimated_C_theta');
estimated_C_theta_NEW = zeros(size(Cx.estimated_C_theta,1)*3,size(Cx.estimated_C_theta,2));
for i = 1:size(Cx.estimated_C_theta,1)
    
    x = Cx.estimated_C_theta(i,:);
    y = Cy.estimated_C_theta(i,:);
    z = Cz.estimated_C_theta(i,:);
    estimated_C_theta_NEW(i*3-2:i*3,:)=[x; y; z]; 
end

% Predicted output and residuals
gd_predicted_y2 = estimated_C_theta_NEW * nominalX;


gd_predicted_y_norm2 =    reshape(gd_predicted_y2, ny, nT)';

plot_comparison(X_norm, measurment_normal, gd_predicted_y_norm2)












function plot_comparison_one(X_norm, measurment_normal, gd_predicted_y)
    figure('Position', [-1920, 0, 1920, 1000]); % [left, bottom, width, height]
    hold on;
    grid on;
    plot(measurment_normal, 'r--', 'LineWidth', 2, 'DisplayName', 'Measured z');
    plot(gd_predicted_y, 'b.', 'LineWidth', 2, 'DisplayName', 'Predicted z');
    xlabel('Step', 'FontSize', 12);
    ylabel('timestep', 'FontSize', 12);
    title('Only 1 axe', 'FontSize', 14);
    legend('measurment_normal', 'gd_predicted_y');
    set(gca, 'FontSize', 12);
    hold off;
end


function plot_comparison_two(X_norm, measurment_normal, gd_predicted_y)
    % Create figure with a specific position and size
    figure('Position', [-1920, 0, 1920, 1000]); % Adjust for screen placement

    % First subplot: Estimated Vx and Vy
    subplot(2,1,1);
    hold on;
    grid on;
    plot(X_norm(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Estimated Vx');
    plot(X_norm(:,4), 'r-', 'LineWidth', 2, 'DisplayName', 'Estimated Vy');
    xlabel('Time Step', 'FontSize', 12);
    ylabel('Signal Value', 'FontSize', 12);
    title('Comparison of Predicted vs Measured Y', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;

    % Third subplot: Comparison of Predicted vs Measured XY
    subplot(2,1,2);
    hold on;
    grid on;
    plot(gd_predicted_y(:,1), gd_predicted_y(:,2), 'b.', 'LineWidth', 2, 'DisplayName', 'Predicted X Y');
    plot(measurment_normal(:,1), measurment_normal(:,2), 'r--', 'LineWidth', 2, 'DisplayName', 'Measured X Y');
    plot(X_norm(:,1), X_norm(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Nominal XY');
    xlabel('X', 'FontSize', 12);
    ylabel('Y', 'FontSize', 12);
    title('Comparison of Predicted vs Measured XY', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;
end


function plot_comparison(X_norm, measurment_normal, gd_predicted_y)
    % Create figure with a specific position and size
    figure('Position', [-1920, 0, 1920, 1000]); % Adjust for screen placement

    % First subplot: Estimated Vx and Vy
    subplot(3,1,1);
    hold on;
    grid on;
    plot(X_norm(:,3), 'b-', 'LineWidth', 2, 'DisplayName', 'Estimated Vx');
    plot(X_norm(:,4), 'r-', 'LineWidth', 2, 'DisplayName', 'Estimated Vy');
    xlabel('Time Step', 'FontSize', 12);
    ylabel('Signal Value', 'FontSize', 12);
    title('Comparison of Predicted vs Measured Y', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;

    % Second subplot: Measured vs Predicted z
    subplot(3,1,2);
    hold on;
    grid on;
    plot(measurment_normal(:,3), 'r--', 'LineWidth', 2, 'DisplayName', 'Measured z');
    plot(gd_predicted_y(:,3), 'b.', 'LineWidth', 2, 'DisplayName', 'Predicted z');
    xlabel('Step', 'FontSize', 12);
    ylabel('z', 'FontSize', 12);
    title('Z', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;

    % Third subplot: Comparison of Predicted vs Measured XY
    subplot(3,1,3);
    hold on;
    grid on;
    plot(gd_predicted_y(:,1), gd_predicted_y(:,2), 'b.', 'LineWidth', 2, 'DisplayName', 'Predicted X Y');
    plot(measurment_normal(:,1), measurment_normal(:,2), 'r--', 'LineWidth', 2, 'DisplayName', 'Measured X Y');
    plot(X_norm(:,1), X_norm(:,2), 'g-', 'LineWidth', 2, 'DisplayName', 'Nominal XY');
    xlabel('X', 'FontSize', 12);
    ylabel('Y', 'FontSize', 12);
    title('Comparison of Predicted vs Measured XY', 'FontSize', 14);
    legend('Location', 'best');
    set(gca, 'FontSize', 12);
    hold off;
end

function [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
    create_lifted_process_observation_model(A_matrices, B_matrices, C_matrices, ...
                                            inputs, ...
                                            process_noise_cov, measurement_noise_cov, eta_noise_cov, ...
                                            mu_x0, sigma_wx0, sigma_theta0, sigma_eta, mu_theta0, mu_eta)

    % Extract dimensions
    [nx, ~, nT] = size(A_matrices);  % State dimension and time horizon
    [~, nu, ~] = size(B_matrices);   % Input dimension
    [ny, ~, ~] = size(C_matrices);   % Measurement dimension
    [nTheta, ~] = size(sigma_theta0); % Theta dimension

    % Construct the lifted transition matrix A
    lifted_A = zeros(nT * nx, nT * nx);
    for i = 1:nT
        lifted_A((i-1) * nx + (1:nx), (i-1) * nx + (1:nx)) = eye(nx);
        if i > 1
            for j = 1:(i-1)
                lifted_A((i-1) * nx + (1:nx), (j-1) * nx + (1:nx)) = A_matrices(:, :, i-1)^(i-j);
            end
        end
    end

    % Initialize lifted_u
    lifted_u = zeros(nx + (nT-1) * nx, 1);
    lifted_u(1:nx, 1) = mu_x0; % Add initial state mean

    % Add input terms Bk * uk
    for i = 1:nT-1
        lifted_u(nx + (i-1) * nx + (1:nx), 1) = B_matrices(:, :, i) * inputs(i, :, 1)';
    end

    % Construct the lifted observation matrix C
    lifted_C = zeros(nT * ny, nT * nx);
    for i = 1:nT
        lifted_C((i-1) * ny + (1:ny), (i-1) * nx + (1:nx)) = C_matrices(:, :, i);
    end

    % Construct the lifted process noise covariance matrix
    sigma_wx = zeros(nT * nx, nT * nx);
    for i = 1:nT
        sigma_wx((i-1) * nx + (1:nx), (i-1) * nx + (1:nx)) = process_noise_cov(:, :, i);
    end
    sigma_wx(1:nx, 1:nx) = sigma_wx0; % Add initial state covariance

    % Construct the lifted measurement noise covariance matrix
    sigma_v = zeros(nT * ny, nT * ny);
    for i = 1:nT
        sigma_v((i-1) * ny + (1:ny), (i-1) * ny + (1:ny)) = measurement_noise_cov(:, :, i);
    end

    % Construct the lifted theta noise covariance matrix
    sigma_eta_lifted = zeros(nT * nTheta, nT * nTheta);
    for i = 1:nT
        sigma_eta_lifted((i-1) * nTheta + (1:nTheta), (i-1) * nTheta + (1:nTheta)) = eta_noise_cov(:, :, i);
    end
    sigma_eta_lifted(1:nTheta, 1:nTheta) = sigma_theta0; % Add initial theta covariance

    % Construct D matrix
    D = zeros(nT * nTheta, nT * nTheta);
    for i = 1:nT
        for j = 1:i
            D((i-1) * nTheta + (1:nTheta), (j-1) * nTheta + (1:nTheta)) = eye(nTheta);
        end
    end

    % Compute Σ_w_theta = D Σ_eta D^T
    sigma_w_theta = D * sigma_eta_lifted * D';

    % Construct mu_theta
    mu_theta = zeros(nT * nTheta, 1);
    mu_theta(1:nTheta, :) = mu_theta0; % Add initial state

    % Add input terms
    prev_mu_theta = zeros(nTheta, 1);
    for i = 1:nT-1
        prev_mu_theta = prev_mu_theta + mu_eta;
        mu_theta((i-1) * nTheta + (1:nTheta), :) = mu_theta0 + prev_mu_theta;
    end
end





function [optimal_vec_theta, predicted_y, residuals, estimated_C_theta] = reconstruct_and_evaluate(...
    optimal_vec_var, size_ck, nT, estimate_bias, nominalX, measured_y)
    
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
