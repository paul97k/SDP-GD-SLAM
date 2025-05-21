function logDetValue = safeLogdet(matrix, method)
    if strcmp(method, 'chol')
        try
            R = chol(matrix);
            logDetValue = 2 * sum(log(diag(R)));
        catch
            logDetValue = -Inf; % Handle singular or invalid matrices
        end
    else
        logDetValue = log(det(matrix));
    end
end
