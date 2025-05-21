function nominalX = f_MyMain4_createNominalX(A,B,inputs, nT, mu_x0)
    A_matrices = repmat(A, 1, 1, nT);
    B_matrices = repmat(B, 1, 1, nT);
                    % Extract dimensions
    [nx, ~, nT] = size(A_matrices);  % State dimension and time horizon
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

    nominalX= lifted_A*lifted_u;
end