function [nominalMatrixC, nominalVecC, trueMatrixC, trueVecC, sigmaC, sizeCk] = ...
    generateMatrixC(constVec, coefVec, timeStep, estimateDataLen, covC)
%  Generating a time-varying C for a 1-D obseravtion with:
%  - constVec: The constant vector for creating C;
%  - coefVec: The coefficient vector for creating C, the first elemnt is 
%    the coefficient of the exponential term of C and the rest are 
%    the coefficient of the trigonometric terms of C;
%  - timeStep: The step size;
%  - horizonLen: The horizon length for data generation;
%  The generated number of data is calculated based on horizonLen and
%  timeStep. New data is collected for each expriment.
%  Note:        length(coefVec)=length(constVec)+1
%               C1 = const1+exp(-coef1*timeK)*cos(coef2*timeK) 
%               C2 = const2+exp(-coef1*timeK)*sin(coef3*timeK)
%               C3 = const2+exp(-coef1*timeK)*cos(coef2*timeK) 
%               C4 = const2+exp(-coef1*timeK)*cos(coef2*timeK) 
%               C5 = const2+exp(-coef1*timeK)*cos(coef2*timeK) 

%               C6 = const2+exp(-coef1*timeK)*cos(coef2*timeK)
%               C7 = const2+exp(-coef1*timeK)*cos(coef2*timeK)
%               C8 = const2+exp(-coef1*timeK)*cos(coef2*timeK)
%               C9 = const2+exp(-coef1*timeK)*cos(coef2*timeK) 
%               C10 = const2+exp(-coef1*timeK)*cos(coef2*timeK) 
%  ----------------
%  @author: Sasan Vakili

totalHorizon = (estimateDataLen-1)*timeStep;

timeK = (0:timeStep:totalHorizon)';

C1 = constVec(1,1)+exp(-coefVec(1,1)*timeK).*cos(coefVec(1,2).*timeK);
C2 = constVec(1,2)+exp(-coefVec(1,1)*timeK).*sin(coefVec(1,3).*timeK);
C3 = repmat(constVec(1,3), length(timeK), 1);
% C3 = constVec(1,3) + exp(coefVec(1,1).*timeK)./(1+exp(coefVec(1,1).*timeK));

% C4 = constVec(1,1)+exp(-coefVec(1,1)*timeK).*cos(coefVec(1,2).*timeK);
% C5 = constVec(1,2)+exp(-coefVec(1,1)*timeK).*sin(coefVec(1,3).*timeK);
% C6 = repmat(constVec(1,3), length(timeK), 1);
% C4 = constVec(2,1) + (2/pi)*asin(sin((2*pi/coefVec(2,1)).*timeK));
% C5 = constVec(2,2) - (2/pi)*acos(cos((2*pi/coefVec(2,1)).*timeK));
% 
% 
% C4 = constVec(2,1) + (2/pi)*asin(sin((2*pi/coefVec(2,1)).*timeK));

% C4 = constVec(2,1) + exp(coefVec(2,1).*timeK)./(1+exp(coefVec(2,1).*timeK));
% C5 = constVec(2,2) + sign(cos((2*pi/coefVec(2,2)).*timeK));
% C6 = repmat(constVec(2,3), length(timeK), 1);
C4 = constVec(1,1)+exp(-coefVec(1,1)*timeK).*cos(coefVec(1,2).*timeK);
C5 = constVec(1,2)+exp(-coefVec(1,1)*timeK).*sin(coefVec(1,3).*timeK);
C6 = repmat(constVec(1,3), length(timeK), 1);




nominalVecC = [C1, C2, C3, C4, C5, C6];

sizeCk = [2, 3];
nominalVecC = reshape(nominalVecC', [], 1);
tempNominalVecC = mat2cell(reshape(reshape(nominalVecC,[],estimateDataLen), ...
    sizeCk(2), [])', sizeCk(1)*ones(1,estimateDataLen));
nominalMatrixC = blkdiag(tempNominalVecC{:});

% covVecC = repmat(covC, sizeCk(1)*sizeCk(2)*dataLen/length(covC), 1);
% sigmaDeltaC = diag(covVecC);

kSigmaDeltaC = covC.*eye(length(covC));
kSigmaDeltaC = repmat({kSigmaDeltaC}, 1, (estimateDataLen));
sigmaDeltaC = blkdiag(kSigmaDeltaC{:});

matrixI = eye(sizeCk(1)*sizeCk(2));
matrixDiff = tril(repmat(matrixI, estimateDataLen));

rng(2022);
expRandomGen = rng;
rng(expRandomGen);

freq = 10;
% trueVecC = nominalVecC + matrixDiff*(sqrt(diag(sigmaDeltaC)).*randn(size(nominalVecC,1),1));
noisyVecC = nominalVecC + matrixDiff*(sqrt(diag(sigmaDeltaC)).*randn(size(nominalVecC,1),1));
trueVecC = [];
for i=1:6
    filteredVecC = lowpass(noisyVecC(i:6:end), 4, freq);
    trueVecC = [trueVecC, filteredVecC];
end
trueVecC = reshape(trueVecC', [], 1);
tempTrueVecC = mat2cell(reshape(reshape(trueVecC,[],estimateDataLen), ...
    sizeCk(2), [])', sizeCk(1)*ones(1,estimateDataLen));
trueMatrixC = blkdiag(tempTrueVecC{:});

sigmaC = matrixDiff*sigmaDeltaC*(matrixDiff');