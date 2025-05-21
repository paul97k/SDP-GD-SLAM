function [sigmaW, sigmaV, nominalVecBU, matrixA, experimentCase] = ...
    generateMeasurements(modelExists, matrixAk, vecBk, ...
    tfProcessModelDenomCoef, tfProcessModelNomCoef, estimateDataLen, ...
    experimentNum, initX, nominalU, covW, covV, trueMatrixC)
%  Generating a N-D state space data set for a time-invariant A 
%  and a time-varying C for a 1-D obseravtion with:
%  - tfProcessModelDenomCoef: The Transfer function denominator coefficients;
%  - tfProcessModelNomCoef: The Transfer function numerator coefficients;
%  - timeStep: The step size;
%  - horizonLen: The horizon length for data generation;
%  - experimentNum: The number of different expriments;
%  - initX: The true initial state;
%  - initCovX: The initial state covariance;
%  - nominalU: The nominal input to the system;
%  - covW: The vector of process model uncertainty covariances;
%  - covV: The measurements covariance.
%  The generated number of data is calculated based on horizonLen and
%  timeStep. New data is collected for each expriment.
%  ----------------
%  @author: Sasan Vakili
%

if modelExists == false
    % [A, B] = modelExample(tfProcessModelDenomCoef, tfProcessModelNomCoef);
    [A, B] = tf2ss(tfProcessModelNomCoef,tfProcessModelDenomCoef);
else
    A = matrixAk;
    B = vecBk;
end

% estimateDataLen = (totalHorizon/timeStep)+1;

vecA = zeros(estimateDataLen*size(A,1), size(A,2));
matrixA = zeros(estimateDataLen*size(A,1), estimateDataLen*size(A,2));
multiplyA = eye(size(A,1), size(A,2));
for i = 0:estimateDataLen-1
    vecA(i*size(A,1)+1:(i+1)*size(A,1),:) = multiplyA;
%     vecA(i*size(A,1)+1:(i+1)*size(A,1),:) = A^i;
    matrixA((estimateDataLen-i-1)*size(A,1)+1:estimateDataLen*size(A,1),...
        (estimateDataLen-i-1)*size(A,2)+1:((estimateDataLen-i-1)+1)*size(A,2)) = vecA(1:((i+1)*size(A,1)),:);
    multiplyA = A*multiplyA;
end

vecU = nominalU.*ones(estimateDataLen-1,1);

vecB = repmat(B, 1, estimateDataLen-1);
vecBU = vecB.*vecU';
nominalVecBU = [initX; reshape(vecBU,[],1)];

kSigmaV = covV.*eye(length(covV));
kSigmaV = repmat({kSigmaV}, 1, (estimateDataLen));
sigmaV = blkdiag(kSigmaV{:});

% initSigmaW = initCovX.*eye(length(initCovX));
kSigmaW = covW.*eye(length(covW));
kSigmaW = repmat({kSigmaW}, 1, (estimateDataLen));
sigmaW = blkdiag(kSigmaW{:});
% kSigmaW = repmat({kSigmaW}, 1, (estimateDataLen-1));
% sigmaW = blkdiag(initSigmaW, kSigmaW{:});
   
nominalX = matrixA*nominalVecBU;

experimentCase = struct;
for iterNum = 1:experimentNum
    
    rng(2023*iterNum);
    expRandomGen = rng;
    rng(expRandomGen);
 
    trueX = nominalX + matrixA*(sqrt(diag(sigmaW)).*randn(length(nominalX),1));
    experimentCase.(sprintf('id_%d', iterNum)).trueX = trueX;

    trueY = trueMatrixC*trueX;
    experimentCase.(sprintf('id_%d', iterNum)).trueY = trueY;

    measuredY = trueY + (sqrt(diag(sigmaV)).*randn(length(trueY),1));
    experimentCase.(sprintf('id_%d', iterNum)).measuredY = measuredY;
%     if iterNum == 1
%         invarX = trueX;
%         invarY = trueY;
%     end
%     invarMeasuredY = invarY + (sqrt(sigmaV).*randn(length(trueY),1));
%     experimentCase.(sprintf('id_%d', iterNum)).invarX = invarX;
%     experimentCase.(sprintf('id_%d', iterNum)).invarY = invarY;
%     experimentCase.(sprintf('id_%d', iterNum)).invarMeasuredY = invarMeasuredY;
end