function [initialVecVar,optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC)

% 
% function [initialVecVar] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
%     nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC)
% Finds the Best C matrix from Maximum A Posteriori approach in two steps:
% 1. Sloves SDP for near optimal C
% 2. Uses the solution from SDP to find the locally optimal C using Gradient
%    Descent method.
%  ----------------
%  @author: Sasan Vakili
%

invSigmaX = inv(sigmaX);
[optimalMatrixS, optimalMatrixC, optimalVecC, optimalValueGamma, ...
    optimalValueCost, optimalBias] = mapUpboundEstimateC(estimateBias, ...
    sigmaV, invSigmaX, nominalX, measuredY, nominalVecC, sigmaC, sizeCk, ...
    desiredSnr, matrixScale);

initialVecVar = [optimalVecC; optimalBias];

% 
[optimalVecVar, optimalValueCost, exitFlag, output, gradient, ...
    hessian] = mapFminuncEstimateC(estimateBias, initialVecVar, sigmaV, ...
    sigmaX, nominalX, measuredY, nominalVecC, sigmaC, sizeCk);
disp('Exit Flag:');
disp(exitFlag);
% initialVecVar
% optimalVecVar


% % Display results
% disp('Optimal Variable Vector:');
% disp(optimalVecVar);
% 
% disp('Optimal Cost Value:');
% disp(optimalValueCost);
% 

% 
% disp('Output Details:');
% disp(output);