classdef classSDP_GD < handle
    properties
        fullPath

        A
        B
        dt
        inputs
        measurements
        estimated_C_theta_results
        nominalX
        nominalXdepth
        estimated_C_theta
        estimated_C_thetaDepth



        mu_theta0 
        mu_eta 
        sigma_v 
        sigma_wx0       
        sigma_wx        
        sigma_theta0    
        sigma_eta       
        x_0
        
        timeElapse

    end
    
    methods
        function obj = classSDP_GD(A, B, dt, inputs, x_0, measurements)
        
            if nargin < 6
                measurements = [];  % Default to empty if not provided
            end
        
            obj.A = A;
            obj.B = B;
            obj.dt = dt;
            obj.inputs = inputs;
            obj.x_0 = x_0;
            obj.measurements = measurements;
            obj.estimated_C_theta_results = cell(1, length(measurements)); % Works even if measurements is empty
        
        end


        function obj = setNoise(obj,mu_theta0, mu_eta, sigma_v, sigma_wx0, sigma_wx, sigma_theta0, sigma_eta)

            obj.mu_theta0 = mu_theta0;
            obj.mu_eta = mu_eta;
            obj.sigma_v = sigma_v;
            obj.sigma_wx0       = sigma_wx0   ;
            obj.sigma_wx        = sigma_wx    ;
            obj.sigma_theta0    = sigma_theta0;
            obj.sigma_eta       = sigma_eta   ;
            
        end


        % 
        % function obj = performSDP_GD(obj)
        %     tic;
        %     n = length(obj.measurements);
        % 
        %     % Preallocate cell arrays to store results
        %     estimated_C_theta_results = cell(1, n);
        %     nominalX_results = cell(1, n);
        % 
        %     % Parallel computation
        %     parfor i = 1:n
        %         [nominalX, estimated_C_theta] = estimateCTheta(obj, obj.measurements{i}, obj.inputs,obj.x_0, obj.dt, obj.A, obj.B);
        % 
        %         % Store results in separate arrays
        %         nominalX_results{i} = nominalX;
        %         estimated_C_theta_results{i} = estimated_C_theta;
        %     end
        % 
        %     % Assign results back to the object AFTER the loop
        %     obj.nominalX = nominalX_results;
        %     obj.estimated_C_theta_results = estimated_C_theta_results;
        %     obj.timeElapse = toc;
        % end


        function obj = performSDP_GD(obj)
            tic;
            n = 2;

            % Preallocate cell arrays to store results
            estimated_C_theta_results = cell(1, n);
            nominalX_results = cell(1, n);

            % Parallel computation
            parfor i = 1:n
                [nominalXTemp, estimated_C_theta] = estimateCTheta(obj, obj.measurements{i}, obj.inputs,obj.x_0, obj.dt, obj.A, obj.B);

                % Store results in separate arrays
                nominalX_results{i} = nominalXTemp;
                estimated_C_theta_results{i} = estimated_C_theta;
            end

            % Assign results back to the object AFTER the loop
            obj.nominalX = nominalX_results;
            obj.estimated_C_theta_results = estimated_C_theta_results;
            mergeC(obj);
            obj.nominalXdepth = obj.estimated_C_theta* nominalX_results{1};
            obj.estimated_C_thetaDepth = estimateCThetaDepth(obj, obj.measurements{3}, obj.nominalXdepth);

            obj.timeElapse = toc;
        end








        function obj = mergeC(obj)
        
            xC = obj.estimated_C_theta_results{1};
            yC = obj.estimated_C_theta_results{2};
            % zC = obj.estimated_C_theta_results{3};

            lengthC = 2;
            obj.estimated_C_theta = zeros(size(xC,1)*lengthC,size(xC,2));
            
            
            for i = 1:size(xC,1)
                
                x = xC(i,:);
                y = yC(i,:);
                % z = zC(i,:);
                obj.estimated_C_theta(i*lengthC-1:i*lengthC,:)=[x; y]; 
            end
            
        end

        function obj = print_diagC(obj)
            block_size = [3, 4]; % Each diagonal block is 3x4
            num_blocks = min(size(obj.estimated_C_theta) ./ block_size); % Number of diagonal blocks
            
            diagonal_blocks = cell(1, num_blocks); % Store extracted diagonal blocks
            
            for i = 1:num_blocks
            row_idx = (i-1)*block_size(1) + (1:block_size(1)); % Row range
            col_idx = (i-1)*block_size(2) + (1:block_size(2)); % Column range
            diagonal_blocks{i} = obj.estimated_C_theta(row_idx, col_idx);
            diagonal_blocks{i} 
            end
        end












        function estimated_C_theta= estimateCThetaDepth(obj, measurment, nominalX)
                
            % measurment = measurement(1:cutIdx,:);
            % 
            % inputs = inputs(1:cutIdx,:);

            A = eye(2);
            B = [0;0];
      
            measured_yT = measurment';
            measurments = measured_yT(:);
           
            nT = length(measurment);
            nx = size(A,1);
            nu = size(B,2);
            ny = size(measurment,2);
            ntheta = ny*nx;


            inputs= zeros(nT,nx);
            x_0 = [0;0];
            inputs= zeros(nT,nx);
            

            % 
            mu_x0 = x_0;
          %  ------------------create lifted model ------------------
            % 7. MAP Estimation


            process_noise_cov = repmat(obj.sigma_wx(1:2,1:2), 1, 1, nT);
            measurement_noise_cov = repmat(obj.sigma_v, 1, 1, nT);
            eta_noise_cov = repmat(obj.sigma_eta(1:2,1:2), 1, 1, nT);
            sigma_wx0 = obj.sigma_wx0(1:2,1:2);
            sigma_theta0 = obj.sigma_theta0(1:2,1:2);
            sigma_eta = obj.sigma_eta(1:2,1:2);
            mu_theta0= obj.mu_theta0(1:2);
            mu_eta= obj.mu_eta(1:2);


            
            C = [1   1 ];
            A_matrices = repmat(A, 1, 1, nT);
            B_matrices = repmat(B, 1, 1, nT);
            C_matrices = repmat(C, 1, 1, nT);
           
        
            [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
                create_lifted_process_observation_modelDepth(obj, A_matrices, B_matrices, C_matrices, inputs, ...
                process_noise_cov, measurement_noise_cov, eta_noise_cov, ...
                mu_x0, sigma_wx0, sigma_theta0, sigma_eta, mu_theta0, mu_eta);
            
            
            nominalVecC = repmat(C, 1, nT); 
            nominalVecC = reshape(nominalVecC', [], 1);
            nominalVecC = double(nominalVecC);
            %  ------------------perform SDP ------------------
            
            sigmaV = sigma_v;
            sigmaC= sigma_w_theta; %sigmaw_theta should not be zero
            sigmaX= sigma_wx;
            estimateBias = true;
        
            
            
            measuredY= measurments; 
            sizeCk= size(C); 
            desiredSnr= 30; 
            matrixScale= 1; 
            trueVecC= [] ; 
            measuredY = double(measuredY);
            measuredY(measuredY > 1000) = 5;
            measuredY(measuredY < -1000) = -5;
        
        
            [~, optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
            nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC);
        
            [~, ~, ~, estimated_C_theta] = reconstruct_and_evaluate(... 
                obj, optimalVecVar, sizeCk, nT, estimateBias, nominalX, measuredY);
        end


        
        function [nominalX, estimated_C_theta] = estimateCTheta(obj, measurment, inputs,x_0, dt, A ,B)
                
            % measurment = measurement(1:cutIdx,:);
            % 
            % inputs = inputs(1:cutIdx,:);
      
            measured_yT = measurment';
            measurments = measured_yT(:);
           
            nT = length(inputs);
            nx = size(A,1);
            nu = size(B,2);
            ny = size(measurment,2);
            ntheta = ny*nx;
            

            % 
            mu_x0 = x_0;
          %  ------------------create lifted model ------------------
            % 7. MAP Estimation


            process_noise_cov = repmat(obj.sigma_wx, 1, 1, nT);
            measurement_noise_cov = repmat(obj.sigma_v, 1, 1, nT);
            eta_noise_cov = repmat(obj.sigma_eta, 1, 1, nT);


            
            C = [1   1   1   1  1  1];
            A_matrices = repmat(A, 1, 1, nT);
            B_matrices = repmat(B, 1, 1, nT);
            C_matrices = repmat(C, 1, 1, nT);
           
        
            [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
                create_lifted_process_observation_model(obj, A_matrices, B_matrices, C_matrices, inputs, ...
                process_noise_cov, measurement_noise_cov, eta_noise_cov, ...
                mu_x0, obj.sigma_wx0, obj.sigma_theta0, obj.sigma_eta, obj.mu_theta0, obj.mu_eta);
            
            
            nominalVecC = repmat(C, 1, nT); 
            nominalVecC = reshape(nominalVecC', [], 1);
            nominalVecC = double(nominalVecC);
            %  ------------------perform SDP ------------------
            
            sigmaV = sigma_v;
            sigmaC= sigma_w_theta; %sigmaw_theta should not be zero
            sigmaX= sigma_wx;
            estimateBias = false;
        
            
            nominalX= lifted_A*lifted_u;
            measuredY= measurments; 
            sizeCk= size(C); 
            desiredSnr= 30; 
            matrixScale= 1; 
            trueVecC= [] ; 
            measuredY = double(measuredY);
            measuredY(measuredY > 1000) = 5;
            measuredY(measuredY < -1000) = -5;
        
        
            [~, optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
            nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC);
        
            [~, ~, ~, estimated_C_theta] = reconstruct_and_evaluate(... 
                obj, optimalVecVar, sizeCk, nT, estimateBias, nominalX, measuredY);
        end


        function [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
            create_lifted_process_observation_model(obj, A_matrices, B_matrices, C_matrices, ...
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
        

        function [lifted_A, lifted_C, lifted_u, sigma_wx, sigma_v, sigma_w_theta, mu_theta] = ...
            create_lifted_process_observation_modelDepth(obj, A_matrices, B_matrices, C_matrices, ...
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
                lifted_u(nx + (i-1) * nx + (1:nx), 1) = B_matrices(:, :, i) * inputs(i, 1)';
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
            obj, optimal_vec_var, size_ck, nT, estimate_bias, nominalX, measured_y)
            
            % Extract sizes
            vec_theta_size = prod(size_ck) * nT;
            optimal_vec_theta = reshape(optimal_vec_var(1:vec_theta_size), [], 1);
            
            % Reconstruct `C_theta`
            estimated_C_theta = construct_C_theta(obj, optimal_vec_theta, size_ck, nT);
               
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



        function C_theta = construct_C_theta(obj, vec_theta, size_ck, nT)
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





    end

    methods (Access = private)
        function [common_time, interp_signal1, interp_signal2, interp_signal3] = interpolate_signals(obj, signal1_time, signal1, signal2_time, signal2, signal3_time, signal3)
    
            % Find the longest time vector
            all_times = {signal1_time, signal2_time, signal3_time};
            [~, longest_idx] = max(cellfun(@length, all_times));
            common_time = all_times{longest_idx};
            
            % Ensure the time vector is sorted
            common_time = sort(unique(common_time));
            


            [signal1_time, unique_idx] = unique(signal1_time);
            signal1 = signal1(unique_idx);
            [signal2_time, unique_idx] = unique(signal2_time);
            signal2 = signal2(unique_idx);
            [signal3_time, unique_idx] = unique(signal3_time);
            signal3 = signal3(unique_idx);


            % Interpolate each signal to match the common time vector
            interp_signal1 = interp1(signal1_time, signal1, common_time, 'linear', 'extrap');
            interp_signal2 = interp1(signal2_time, signal2, common_time, 'linear', 'extrap');
            interp_signal3 = interp1(signal3_time, signal3, common_time, 'linear', 'extrap');
        end




    end

    
end
