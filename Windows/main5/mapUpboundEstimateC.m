function [optimalMatrixS, optimalMatrixC, optimalVecC, optimalValueGamma, ...
    optimalValueCost, optimalBias] = mapUpboundEstimateC(estimateBias, sigmaV, invSigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale)
% Finds the near optimal C matrix from Maximum A Posteriori approach using SDP method.
%  ----------------
%  @author: Sasan Vakili
%

% nominalVecCk = mat2cell(reshape(reshape(nominalVecC,[],estimateDataLen), sizeCk(2), [])', sizeCk(1)*ones(1,estimateDataLen));
% nominalMatrixC = blkdiag(nominalVecCk{:});


yalmip('clear')

% Dimensions:
dataLen = length(measuredY)/sizeCk(1);

% Decision Variables:
matrixS = sdpvar(size(sigmaV,1), size(sigmaV,2));
gamma = sdpvar(1);
beta = sdpvar(1);
vecC = sdpvar(sizeCk(1)*sizeCk(2)*dataLen,1);



vecCk = mat2cell(reshape(reshape(vecC,[],dataLen), sizeCk(2), [])', sizeCk(1)*ones(1,dataLen));
matrixC = blkdiag(vecCk{:});
if estimateBias == true
    bias = sdpvar(sizeCk(1),1);
    vecBias = repmat(bias, dataLen, 1);
else
    vecBias = zeros(sizeCk(1)*dataLen,1);
end

% These are just to avoid Computational Round off:
matrixS = 0.5*(matrixS+matrixS');
pwrS = (desiredSnr - 15)/5;


if issparse(measuredY)
    measuredY = full(double(measuredY));
end
if issparse(nominalX)
    nominalX = full(double(nominalX));
end


% Constraints:
constraint1 = matrixS >= 1e-12;
constraint2 = gamma >= 0;
constraint3 = [(10^pwrS)*(sigmaV-matrixS), matrixC; matrixC', -(10^(-pwrS))*(invSigmaX)] <= 0;
constraint4 = [-matrixS, (measuredY-matrixC*nominalX-vecBias); ...
    (measuredY-matrixC*nominalX-vecBias)', -gamma] <= 0;
constraint5 = beta >= 0;

constraint6 = [-sigmaC, (vecC-nominalVecC); (vecC-nominalVecC)', -beta] <= 0;
constraint = constraint1 + constraint2 + constraint3 + constraint4 + constraint5 + constraint6;



% Objective:
% cost = trace(matrixS-eye(size(sigmaV,1), size(sigmaV,2))) + gamma + betta;
cost = trace(matrixScale*matrixS*(matrixScale') - eye(size(sigmaV,1), size(sigmaV,2))) + gamma + beta;




% Optimization Solution:
try
    solution = optimize(constraint, cost, sdpsettings('solver','mosek', 'verbose', 0));
    
    % Numerical Values
    optimalMatrixS = value(matrixS);
    optimalMatrixC = value(matrixC);
    optimalVecC = value(vecC);
    optimalValueGamma = value(gamma);
    optimalValueCost = value(cost);
    if estimateBias == true
        optimalBias = value(bias);
    else
        optimalBias = zeros(sizeCk(1),1);
    end
catch
    optimalMatrixS = nan(size(sigmaV,1), size(sigmaV,2));
    optimalMatrixC = nan(sizeCk(1)*dataLen, sizeCk(2)*dataLen);
    optimalVecC = nan(sizeCk(1)*sizeCk(2)*dataLen,1);
    optimalValueGamma = nan(1);
    optimalValueCost = nan(1);
    optimalBias = nan(sizeCk(1),1);
end


% outline  setup
% SDP-GD- add words
% write papre name
% method: explain what y is
