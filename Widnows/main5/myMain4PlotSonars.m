
clear
clc
close all
obj_ID_ = load('identifiedSystem.mat', 'obj');


MyMain4_variables; % get all variables

TestSavedOutputMap = true;


if TestSavedOutputMap
    runSDP = false;
    runSonarSLAM = false;
else
    runSDP = true;
    runSonarSLAM = true;
end
cutIdx = 60;


%---------------------------------------------------------
%set system as identified system
obj_ID = obj_ID_.obj;
A= obj_ID.identification.A; 
B = obj_ID.identification.B;
dt = obj_ID.identification.dt;
Ts = obj_ID.downSampleFreq;


figureNumber = 0;
trainFiles = Straight_differentBottomsTest;
testFiles = Straight_differentBottomsTest;

% for fileNumber = 2:10
idx1 = find(Straight_differentBottomsTest == "S0.22_D10_R0.5_D10_Test1");
idx2 = find(Straight_differentBottomsTest == "S0.22_D10_R0.5_D10_Test2");
idx3 = find(Straight_differentBottomsTest == "S0.22_D10_R0.5_D10_Test3");
idx4= find(Straight_differentBottomsTest == "S0.22_D10_R0.5_D12_Test1");

testFIleNumbers =[idx1 idx2 idx3 idx4];
itt = 0; 
for fileNumberTrain = 1:length(trainFiles)
% for i = 1:length(testFIleNumbers)
%     fileNumberTrain = testFIleNumbers(i);

    TrainedDataFileName = trainFiles(fileNumberTrain)+'.nT_'+cutIdx+'.Ts_'+Ts+'.mat';
    TrainCThetaFilePath  = dataFolder_SDP_CTheta + TrainedDataFileName; %get previously saved C theta
    TrainSonarMapFilePath    = dataFolder_SonarBasedSlam + TrainedDataFileName; %get previously saved C theta


    TestDataFileName = testFiles(fileNumberTrain)+'.mat';


    TestDatafilePath =   dataFolder_Straight_differentBottoms+TestDataFileName;
            
    

    objData = DataLoader(TestDatafilePath);
    objData.downSampleFreq = 2;
    objData.computeNED_AddNoise(true);
    objData.computeXZsonar(false);

    timestampsUn = objData.sonarPositions4.timestamps0; 
    z_positionsUn = objData.sonarPositions4.ranges0; 
    

    itt = itt+1;


    % if  fileNumberTrain==idx2
    %     itt=1;
    % elseif fileNumberTrain==idx3
    %     itt=3;
    % end

    if contains(TestDataFileName, "Test3")
        timestampsUn=timestampsUn-5;
        itt=3;
    end

   if itt == 1
       figure
       plot(timestampsUn, z_positionsUn)
       hold on
       legends =[TestDataFileName];
       legend(legends)
   elseif itt == 2
       plot(timestampsUn, z_positionsUn)
       legends =[legends;TestDataFileName];
       legend(legends)
       xlim([0,50])
   elseif itt == 3
       plot(timestampsUn, z_positionsUn);
       legends =[legends;TestDataFileName];
       legend(legends)
       xlim([0,50])

       itt = 0;   
   
  elseif itt == 4
       plot(timestampsUn, z_positionsUn);
       legends =[legends;TestDataFileName];
       legend(legends)
       xlim([0,50])
       itt = 0;   
   end
   



end