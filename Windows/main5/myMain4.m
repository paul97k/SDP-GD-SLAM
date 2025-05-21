
clear
clc
close all
MyMain4_variables; % get all variables
% Ts = 2;
% cutIdx = 60;
% trusterInput = false;
% trusterInput = true;

Ts = 20;
cutIdx = 100;
trusterInput = true;

TestSavedOutputMap = true;
% TestSavedOutputMap = false;

if trusterInput 
    identifiedSystemName = dataFolder_identifiedSystems+"trusterInput_identifiedSystem_Ts"+Ts+ ".mat";
    dataFolder_OutputMapSDP = dataFolder_SDP_CThetaThrusters;
    dataFolder_OutputMapSonarBased = dataFolder_SonarBasedSlamThrusters;
else
    identifiedSystemName = dataFolder_identifiedSystems+"identifiedSystem_Ts"+Ts+ ".mat";
    dataFolder_OutputMapSDP = dataFolder_SDP_CTheta;
    dataFolder_OutputMapSonarBased = dataFolder_SonarBasedSlam;
end

% identifiedSystemName = 'identifiedSystem2.mat';
obj_ID_ = load(identifiedSystemName, 'obj');
obj_ID = obj_ID_.obj;




Ts = obj_ID.downSampleFreq;

% trainFiles = Straight_differentBottomsNoise;
% testFiles = Straight_differentBottomsNoise;
trainFiles = Straight_differentBottomsTest;
testFiles = Straight_differentBottomsTest;


folderDataPath = dataFolder_Straight_differentBottoms;



itt = 0;
for fileNumberTrain =length(trainFiles)-1:length(trainFiles)-1
% for fileNumberTrain =1:1
    %   
        DataFileName    = trainFiles(fileNumberTrain);
        OutputMapsDataFileName    = DataFileName+'.nT_'+cutIdx+'.Ts_'+Ts+'.mat';
    if ~TestSavedOutputMap
        f_MyMain4_create_outputMaps(obj_ID, cutIdx, folderDataPath, dataFolder_OutputMapSDP, dataFolder_OutputMapSonarBased, DataFileName, trusterInput)
    else


    
        SDPOutputMapFilePath   = dataFolder_OutputMapSDP       + OutputMapsDataFileName;
        SonarOutputMapFilePath = dataFolder_OutputMapSonarBased   + OutputMapsDataFileName;
    
        StartfileNumberTest = getStartFileNumber(OutputMapsDataFileName, testFiles);
    
    
        if isempty(StartfileNumberTest)
            continue
        end

        
    
        for testFilesI = 0:1
    
            TestDataFileName = testFiles(StartfileNumberTest+testFilesI);
            % if ~contains(OutputMapsDataFileName, TestDataFileName)
            %     continue
            % end

            itt = itt+1; 

            disp("Figure: " +itt)
            disp("Use output map: " +OutputMapsDataFileName)
            disp("Data to test  : " +TestDataFileName)

            f_MyMain4_Test_outputMaps(obj_ID, cutIdx, SDPOutputMapFilePath, SonarOutputMapFilePath, folderDataPath, OutputMapsDataFileName, TestDataFileName,  itt, trusterInput)
            
    
        end
    end
end


function StartfileNumberTest = getStartFileNumber(MapDataFileName, testFiles)

    if contains(MapDataFileName, 'Noise')
        fileNameTokens = regexp(MapDataFileName, '(.*?)_Noise', 'tokens');
    elseif contains(MapDataFileName, 'Test')
        fileNameTokens = regexp(MapDataFileName, '(.*?)_Test', 'tokens');
    else
        error('Filename must contain either "Noise" or "Test".');
    end

    if isempty(fileNameTokens)
        error('No matching tokens found in the filename.');
    end

    baseFileName = fileNameTokens{1}{1};
    StartfileNumberTest = find(contains(testFiles, baseFileName), 1);

    if isempty(StartfileNumberTest)
        error('No matching file found in testFiles for base name: %s', baseFileName);
    end
end
