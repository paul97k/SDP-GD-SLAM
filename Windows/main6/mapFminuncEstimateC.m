function [optimalVecVar, optimalValCost, exitFlag, output, gradient, hessian] = mapFminuncEstimateC(...
    estimateBias, initialVecVar, sigmaV, sigmaX, nominalX, measuredY, nominalVecC, sigmaC, sizeCk)
% Finds a locally optimal C matrix from Maximum A Posteriori approach using 
% Quasi-Newton Algorithm with a given gradient.
%  ----------------
%  @author: Sasan Vakili
%

% warning('off','all')

% Dimensions:
dataLen = length(measuredY)/sizeCk(1);
varLen = sizeCk(1)*sizeCk(2)*dataLen+sizeCk(1);

% try
%     options = optimoptions(@fminunc, 'Algorithm','quasi-newton', ...
%         'SpecifyObjectiveGradient', true, 'checkGradients', false, ...
%         'MaxIterations', 5000, 'Display', 'iter');
% 
%     problem.options = options;
%     problem.x0 = initialVecVar;
%     % problem.x0 = ones([3,1]);
%     problem.objective = @(vecVar)mapCostEstimateC(vecVar, estimateBias, sigmaV, sigmaX, ...
%     nominalX, measuredY, nominalVecC, sigmaC, sizeCk);
%     problem.solver = 'fminunc';
%     [optimalVecVar, optimalValCost, exitFlag, output, gradient, hessian] = fminunc(problem);
% catch
% 
%     disp('........................................')
% 
%     optimalVecVar = nan(varLen,1);
%     optimalValCost = nan(1);
%     exitFlag = nan(1);
%     output = nan(1);
%     gradient = nan(1);
%     hessian = nan(1);
% end

options = optimoptions(@fminunc, 'Algorithm', 'quasi-newton', ...
    'SpecifyObjectiveGradient', true, ...
    'MaxIterations', 1000, 'Display', 'off');


problem.options = options;
problem.x0 = initialVecVar;
problem.objective = @(vecVar) mapCostEstimateC(vecVar, estimateBias, sigmaV, sigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk);
problem.solver = 'fminunc';


% Explicitly check gradients before optimization
checkGradients(@(vecVar) mapCostEstimateC(vecVar, estimateBias, sigmaV, sigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk), initialVecVar);

try
    [optimalVecVar, optimalValCost, exitFlag, output, gradient, hessian] = fminunc(problem);

catch
    optimalVecVar = nan(varLen, 1);
    optimalValCost = nan;
    exitFlag = nan;
    output = nan;
    gradient = nan;
    hessian = nan;
end
