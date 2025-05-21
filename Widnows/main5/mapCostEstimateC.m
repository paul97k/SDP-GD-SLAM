function [currentCost, gradientCost] = mapCostEstimateC(vecVar, estimateBias, sigmaV, sigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk)
% Optimization cost of the locally optimal C matrix from Maximum A Posteriori approach.
%  ----------------
%  @author: Sasan Vakili
%

% Dimensions:
dataLen = length(measuredY)/sizeCk(1);

% Parameters:
% invSigmaC = inv(sigmaC);

% Variables:
vecC = vecVar(1:sizeCk(1)*sizeCk(2)*dataLen,:);
vecCk = mat2cell(reshape(reshape(vecC,[],dataLen), sizeCk(2), [])', sizeCk(1)*ones(1,dataLen));
matrixC = blkdiag(vecCk{:});
sigmaY = matrixC*sigmaX*matrixC'+sigmaV;
% invSigmaY = inv(sigmaY);
if estimateBias == true
    bias = vecVar(sizeCk(1)*sizeCk(2)*dataLen+1:sizeCk(1)*sizeCk(2)*dataLen+sizeCk(1),:);
    vecBias = repmat(bias, dataLen, 1);
else
    vecBias = zeros(sizeCk(1)*dataLen,1);
end

% Cost:
currentCost = safeLogdet(sigmaY, 'chol') ...
    + (((measuredY-matrixC*nominalX-vecBias)')/sigmaY)*(measuredY-matrixC*nominalX-vecBias) ...
    + (((vecC-nominalVecC)')/sigmaC)*(vecC-nominalVecC);

% Gradient:
if nargout > 1
    gradientCostMatrixC = 2*(sigmaY\matrixC*sigmaX) ...
    - 2*(sigmaY\(measuredY-matrixC*nominalX-vecBias))*(((measuredY-matrixC*nominalX-vecBias)')/sigmaY) ...
    *(matrixC*sigmaX) - 2*(sigmaY\(measuredY-matrixC*nominalX-vecBias))*(nominalX');

    gradientCostVecC = zeros(sizeCk(1)*sizeCk(2)*dataLen,1);
    for i = 1:sizeCk(1)*sizeCk(2)*dataLen
        gradientVecC = zeros(sizeCk(1)*sizeCk(2)*dataLen,1);
        gradientVecC(i) = 1;
        gradientVecCk = mat2cell(reshape(reshape(gradientVecC,[],dataLen), ...
            sizeCk(2), [])', sizeCk(1)*ones(1,dataLen));
        gradientMatrixC = blkdiag(gradientVecCk{:});
        gradientCostVecC(i) = trace((gradientCostMatrixC')*gradientMatrixC);
    end
    gradientCostVecC = gradientCostVecC + 2*(sigmaC\(vecC-nominalVecC));
    
    if estimateBias == true
        gradientCostVecBias = -2*(sigmaY\(measuredY-matrixC*nominalX-vecBias));
        gradientCostBias = zeros(sizeCk(1),1);
        for i = 1:sizeCk(1)
        gradientVecBias = zeros(sizeCk(1)*dataLen,1);
        gradientVecBias(i:sizeCk(1):sizeCk(1)*dataLen) = 1;
        gradientCostBias(i) = (gradientCostVecBias')*gradientVecBias;
        end
    else
        gradientCostBias = zeros(sizeCk(1),1);
    end

    gradientCost = [gradientCostVecC; gradientCostBias];
end
