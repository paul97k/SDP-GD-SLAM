% function [initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimate_batches(estimateBias, sigma_v_batches, sigma_wx_batches, ...
%     lifted_A_batches, lifted_u_batches, measuredY, C, sigma_w_theta_batches, sizeCk, desiredSnr, matrixScale, trueVecC, batchSize)
% % Calculate the number of batches
% 

function [nominalX, initialVecVar, optimalVecVar, optimalValueCost, exitFlag, output]...
            = mapEstimate_batches(filenameLiftedModelBatches, filenameLiftedModel, batchSize, estimateBias, desiredSnr, matrixScale, trueVecC)
    % Calculate the number of batches
    
    
    
    load(filenameLiftedModelBatches)
    
    sizeCk= size(C); 
    measuredY = measured_y;
    
    
    num_batches = floor(length(measuredY) / (batchSize*sizeCk(1))) + (mod(length(measuredY), batchSize*sizeCk(1)) > 0);
    
    % Preallocate arrays for results
    all_optimalVecC = cell(num_batches, 1);
    all_optimalBias = cell(num_batches, 1);
    all_optimalValueCost = cell(num_batches, 1);
    
    % Display number of batches
    disp(["num_batches = ", num_batches]);

    start_idx = 1;
    end_idx =  sizeCk(1);

    % Parallel loop over batches
    for i = 1:num_batches
        % Display progress every 10 iterations
        % if mod(i, 10) == 0
        %     disp(['Processing batch ', num2str(i), ' of ', num2str(num_batches)]);
        % end

        if iscell(sigma_v_batches)
            % Extract batch data from cells
            sigmaV_batch = sigma_v_batches{i};
            sigmaWX_batch = sigma_wx_batches{i};
            lifted_A_batch = lifted_A_batches{i};
            lifted_u_batch = lifted_u_batches{i};
            sigmaWTheta_batch = sigma_w_theta_batches{i};
            
            % Compute nominal state
            nominalX = lifted_A_batch * lifted_u_batch;
            % Calculate batch size dynamically from cell
            
            batchSize = size(lifted_A_batches{i}, 1) / 3;  
        elseif isnumeric(sigma_v_batches)
            % Extract batch data from numeric arrays
            sigmaV_batch = sigma_v_batches(i, :, :); % Extract the i-th 30x30 matrix
            sigmaWX_batch = sigma_wx_batches(i, :, :);
            lifted_A_batch = lifted_A_batches(i, :, :);
            lifted_u_batch = lifted_u_batches(i, :, :);
            sigmaWTheta_batch = sigma_w_theta_batches(i, :, :);
            
            % Squeeze to remove singleton dimensions (optional)
            sigmaV_batch = squeeze(sigmaV_batch);
            sigmaWX_batch = squeeze(sigmaWX_batch);
            lifted_A_batch = squeeze(lifted_A_batch);
            lifted_u_batch = squeeze(lifted_u_batch);
            sigmaWTheta_batch = squeeze(sigmaWTheta_batch);
            nominalX = lifted_A_batch * lifted_u_batch';
            
    
            % Calculate batch size dynamically from numeric arrays
            batchSize = size(lifted_A_batch, 1) / 3;  
        else
            error('Unsupported data type for batch variables');
        end


        % Determine start and end indices for the current batch
        % start_idx = (i - 1) * batchSize + 1;
        % end_idx = min(i * batchSize, length(measuredY));

        measuredY_batch = measuredY(start_idx:end_idx);
        

  
        
        % Construct nominalVecC
        nominalVecC = repmat(C, 1, batchSize); 
        nominalVecC = reshape(nominalVecC', [], 1);
        nominalVecC = double(nominalVecC);



        % Compute intermediate variables
        invSigmaWX = inv(sigmaWX_batch);

        % Call the MAP estimation helper function for the batch
        [optimalMatrixS, optimalMatrixC, optimalVecC, optimalValueGamma, ...
            optimalValueCost, optimalBias] = mapUpboundEstimateC(estimateBias, ...
            sigmaV_batch, invSigmaWX, nominalX, measuredY_batch, nominalVecC, ...
            sigmaWTheta_batch, sizeCk, desiredSnr, matrixScale);

        % Store results in preallocated arrays
        all_optimalVecC{i} = optimalVecC;
        all_optimalBias{i} = optimalBias;
        all_optimalValueCost{i} = optimalValueCost;
        
        
        start_idx = end_idx +1;
        end_idx = min((start_idx+sizeCk(1)-1), length(measuredY));
    end

    % Combine results from all batches
    all_optimalVecC = vertcat(all_optimalVecC{:});
    all_optimalBias = vertcat(all_optimalBias{:});
    all_optimalValueCost = vertcat(all_optimalValueCost{:});

    % Combine initial estimates
    % initialVecVar = [all_optimalVecC; 0 ;0 ;0];
    initialVecVar = all_optimalVecC;



    load(filenameLiftedModel)

    estimateBias = false;
    sigmaV = sigma_v;
    sigmaX= sigma_wx;
    nominalX= lifted_A*lifted_u; 
    measuredY= measured_y; 
    sigmaC= sigma_w_theta; %sigmaw_theta
    sizeCk= size(C);
    desiredSnr= 30; 
    matrixScale= 1; 
    trueVecC= [] ; 

    nominalVecC = repmat(C, 1, nT); 
    nominalVecC = reshape(nominalVecC', [], 1);
    nominalVecC = double(nominalVecC);
    
    measuredY = double(measuredY);



    % Use combined estimates for final optimization
    [optimalVecVar, optimalValueCost, exitFlag, output, gradient, ...
        hessian] = mapFminuncEstimateC(estimateBias, initialVecVar, sigmaV, ...
        sigmaX, nominalX, measuredY, nominalVecC, sigmaC, sizeCk);

    % Display final results
    % disp('Optimal Variable Vector:');
    % disp(optimalVecVar);


    disp('Exit Flag:');
    disp(exitFlag);

end


