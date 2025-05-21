function filename = createDataset(timeStep, estimateDataLen, constVec, coefVec, covC, ...
    modelExists, matrixAk, vecBk, tfProcessModelDenomCoef, tfProcessModelNomCoef, ...
    experimentNum, initX, nominalU, covW, covV, desiredSnr, desiredSnrC)
%% Dataset Creation
disp('Creating dataset...');

[nominalMatrixC, nominalVecC, trueMatrixC, trueVecC, sigmaC, sizeCk] = ...
    generateMatrixC(constVec, coefVec, timeStep, estimateDataLen, covC);

alphaC = zeros(6,1);
for i=1:6
    alphaC(i) = 10^(-(desiredSnrC - snr(nominalVecC(i:6:end), trueVecC(i:6:end)-nominalVecC(i:6:end)))/20);
end
alphaC = alphaC.^2;
covC = alphaC.*covC;
[nominalMatrixC, nominalVecC, trueMatrixC, trueVecC, sigmaC, sizeCk] = ...
    generateMatrixC(constVec, coefVec, timeStep, estimateDataLen, covC);
tunedSnrC = zeros(6,1);
for i=1:6
    tunedSnrC(i) = snr(nominalVecC(i:6:end), trueVecC(i:6:end)-nominalVecC(i:6:end));
end

tempExperimentNum = 1;

[~, ~, nominalVecBU, matrixA, experimentCase] = ...
    generateMeasurements(modelExists, matrixAk, vecBk, ...
    tfProcessModelDenomCoef, tfProcessModelNomCoef, estimateDataLen, ...
    tempExperimentNum, initX, nominalU, covW, covV, trueMatrixC);

measuredY = experimentCase.id_1.measuredY;
nominalX = matrixA*nominalVecBU; %AU
nominalY = trueMatrixC*nominalX; %CAU
alpha = 10^(-(desiredSnr - snr(nominalY, measuredY-nominalY))/20);
% alpha = 10^(-(desiredSnr - snr(nominalY(1:2:end), measuredY(1:2:end)-nominalY(1:2:end)))/20);
% beta = 10^(-(desiredSnr - snr(nominalY(2:2:end), measuredY(2:2:end)-nominalY(2:2:end)))/20);
covW = (alpha^2).*covW;
covV = (alpha^2).*covV;
[sigmaW, sigmaV, nominalVecBU, matrixA, experimentCase] = ...
    generateMeasurements(modelExists, matrixAk, vecBk, ...
    tfProcessModelDenomCoef, tfProcessModelNomCoef, estimateDataLen, ...
    experimentNum, initX, nominalU, covW, covV, trueMatrixC);

nominalX = matrixA*nominalVecBU;
sigmaX = matrixA*sigmaW*(matrixA');
nominalY = trueMatrixC*nominalX;
measuredY = experimentCase.id_1.measuredY;
tunedSNR = snr(nominalY, measuredY-nominalY);

% estimateDataLen = (totalHorizon/timeStep)+1;
filename = ['Instant_', sprintf('%d_samples_%d_snr.mat', estimateDataLen, desiredSnr)];


save(filename, 'timeStep', 'estimateDataLen', 'nominalMatrixC', ...
    'nominalVecC', 'trueMatrixC', 'trueVecC', 'sigmaC', 'sizeCk', ...
    'experimentCase', 'matrixA', 'sigmaW', 'nominalX', 'sigmaX', 'sigmaV', 'covW', ...
    'covV', 'tunedSNR', 'tunedSnrC', 'nominalY', 'measuredY');
disp('Done. ');

disp(['Done.  ', filename]);

