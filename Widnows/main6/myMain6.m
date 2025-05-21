
clear
clc
% close all
cd('C:\Users\paulk\iCloudDrive\Uni\Thesis\Codes\sasan code\main6')

MyMain_variables; % get all variables
% Ts = 2;
% cutIdx = 60;
trusterInput = false;
trusterInput = true;


% Ts = 5;
% cutIdx = 60;



% Ts =6 ;
% cutIdx = 60;


Ts = 2;
cutIdx = 59;

plot1 = true;
plot2 = true;
% 
% plot1 = false; % hide to hide errors
plot2 = false;

trainModel = true;
trainModel = false;
% 
MapIsTest = false;
% MapIsTest = true;
% 




if trusterInput 
    identifiedSystemName = dataFolder_identifiedSystems+"trusterInput_identifiedSystem_Ts"+Ts+ ".mat";
    dataFolder_OutputMapSDP = dataFolder_SDP_CThetaThrusters;
    dataFolder_OutputMapSDPMain4 = dataFolder_SDP_CThetaThrustersMain4;
    dataFolder_OutputMapSonarBased = dataFolder_SonarBasedSlamThrusters;
else
    identifiedSystemName = dataFolder_identifiedSystems+"identifiedSystem_Ts"+Ts+ ".mat";
    dataFolder_OutputMapSDP = dataFolder_SDP_CTheta;
    dataFolder_OutputMapSDPMain4 = dataFolder_SDP_CThetaMain4;
    dataFolder_OutputMapSonarBased = dataFolder_SonarBasedSlam;
end

% identifiedSystemName = 'identifiedSystem2.mat';
obj_ID_ = load(identifiedSystemName, 'obj');
obj_ID = obj_ID_.obj;




Ts = obj_ID.downSampleFreq;

% trainFiles = Straight_differentBottomsNoise;
testFiles = Straight_differentBottomsNoise;
trainFiles = Straight_differentBottomsTest;
% testFiles = Straight_differentBottomsTest;


folderDataPath = dataFolder_Straight_differentBottoms;



% for fileNumberTrain =1:length(trainFiles)
% for fileNumberTrain =length(trainFiles):length(trainFiles)
for fileNumberTrain =19:19
    %   
        DataFileName    = trainFiles(fileNumberTrain);
        OutputMapsDataFileName    = DataFileName+'.nT_'+cutIdx+'.Ts_'+Ts+'.mat';
        SDPOutputMapFilePathMain4   = dataFolder_OutputMapSDPMain4       + OutputMapsDataFileName;

        


    if trainModel
        f_MyMain6_create_outputMaps(obj_ID, cutIdx, folderDataPath, dataFolder_OutputMapSDP, ...
            dataFolder_OutputMapSonarBased, DataFileName, trusterInput, SDPOutputMapFilePathMain4)
    else


    
        SDPOutputMapFilePath   = dataFolder_OutputMapSDP       + OutputMapsDataFileName;
        SonarOutputMapFilePath = dataFolder_OutputMapSonarBased   + OutputMapsDataFileName;
    
        [StartfileNumberTest, TestFileExistOrNot] = getStartFileNumber(OutputMapsDataFileName, testFiles);
        
        if ~TestFileExistOrNot
            continue
        end
        
    
        if isempty(StartfileNumberTest)
            continue
        end

        
    
        for testFilesI = 0:1
            % close all

            % set test data, if it contains noise in the name then testdata
            % is equal to the first or second test data starting with the
            % same disturbance level. 
            % If the outputmap contains Test1, then the next file is
            % used, which is Test2 and vice versa
            if contains(testFiles(1), "Noise")
                TestDataFileName = testFiles(StartfileNumberTest+testFilesI);
            elseif contains(OutputMapsDataFileName, "Test1")
                TestDataFileName = testFiles(StartfileNumberTest+testFilesI);
            else
                TestDataFileName = testFiles(StartfileNumberTest-testFilesI);
            end

            % optional: Only test  noise1 data, so that the caculateMean
            % will calculate only noise 1 files.
            if ~contains(TestDataFileName, "Noise1")
                continue
            end

            if contains(TestDataFileName, "Noise")
                fileNameTokens = regexp(TestDataFileName, '(.*?)_Noise', 'tokens');
            else
                fileNameTokens = regexp(TestDataFileName, '(.*?)_Test', 'tokens');
            end

            % fileNameTokens = regexp(TestDataFileName, '(.*?)_Test', 'tokens');
            stringBeforeTest = fileNameTokens{1}{1};
            if ~MapIsTest
                if contains(OutputMapsDataFileName, TestDataFileName) || ~contains(OutputMapsDataFileName, stringBeforeTest)
                    continue
                end            
            else
                if ~contains(OutputMapsDataFileName, TestDataFileName) %check test = train data
                    continue
                end            
            end
            if testFilesI==0
                itt = fileNumberTrain*2-1;
            else
                itt = fileNumberTrain*2;
            end

            disp("Figure: " +itt)
            disp("Use output map: " +OutputMapsDataFileName)
            disp("Data to test  : " +TestDataFileName)

            f_MyMain6_Test_outputMaps(obj_ID, cutIdx, SDPOutputMapFilePath, ...
                SonarOutputMapFilePath, folderDataPath, OutputMapsDataFileName, ...
                TestDataFileName,  itt, trusterInput, plot1, plot2)
            
    
        end
    end
end


function [StartfileNumberTest, TestFileExistOrNot] = getStartFileNumber(MapDataFileName, testFiles)

    if contains(MapDataFileName, 'Noise')
        fileNameTokens = regexp(MapDataFileName, '(.*?)_Noise', 'tokens');
    elseif contains(MapDataFileName, 'Test')
        if contains(testFiles(1), 'Noise')
            fileNameTokens = regexp(MapDataFileName, '(.*?)_Test', 'tokens');
        else
            fileNameTokens = regexp(MapDataFileName, '(.*?).nT', 'tokens');
        end
    else
        error('Filename must contain either "Noise" or "Test".');
    end

    if isempty(fileNameTokens)
        error('No matching tokens found in the filename.');
    end

    baseFileName = fileNameTokens{1}{1};

    StartfileNumberTest = find(contains(testFiles, baseFileName), 1);
    TestFileExistOrNot = true;

    if isempty(StartfileNumberTest)
        TestFileExistOrNot = false;
    end
end
