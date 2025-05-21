%% Example:
clear all
close all
clc

experimentNum = 10;
totalHorizon = 100;
timeStep = 0.1;
estimateDataLen = (totalHorizon/timeStep)+1;
estimateDataLen = str2double(num2str(estimateDataLen, '%.0f'));
constVec = [5, 1.5, 2; 3, 4.5, 3.5];
coefVec = [0.6, 0.4, 0.025; 1.5, 22, 1];
modelExists = true;

matrixAk = [0.7 0.25 0; ...
            0 0.5 0; ...
            0 0.25 0.7];

vecBk = [0; 1; 1];
tfProcessModelDenomCoef = [];
tfProcessModelNomCoef = [];

initX = [1; 0.5; 2];
initCovX = [2.5e-5; 1.5e-4; 1e-4];

timeK = (0.1:timeStep:totalHorizon)';
nominalU = 3.5+cos(2.*timeK);

covW = [1.8e-4; 2.2e-4; 1e-4];
covV = [3e-4; 2e-4];

covC = [0.2; 0.3; 0.4; 0.25; 0.35; 0.45];
desiredSnrC = 30;
desiredSnr = 30;

createDataset(timeStep, estimateDataLen, constVec, coefVec, covC, ...
        modelExists, matrixAk, vecBk, tfProcessModelDenomCoef, tfProcessModelNomCoef, ...
        experimentNum, initX, nominalU, covW, covV, desiredSnr, desiredSnrC);
%% Set EstimateBias to False:
estimateBias = false;
% Load Created Data:
filename = ['Instant_', sprintf('%d_samples_%d_snr.mat', estimateDataLen, desiredSnr)];
load(filename);
matrixScale =1;
% 
% You can now see how the input to the following function look like:
[optimalVecVar, optimalValueCost, exitFlag, output] = mapEstimateC(estimateBias, sigmaV, sigmaX, ...
    nominalX, measuredY, nominalVecC, sigmaC, sizeCk, desiredSnr, matrixScale, trueVecC)