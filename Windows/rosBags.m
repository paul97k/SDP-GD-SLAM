clc;
clear



file = 'rosbag2_2025_01_28-16_48_44';

file1 = 'rosbag2_2025_01_29-11_00_35';
file2 = 'rosbag2_2025_01_29-11_03_29';
file3 = 'rosbag2_2025_01_29-11_06_17';
folderPath = fullfile(pwd, 'data', 'ros2Bags', file1);
bagReader = ros2bagreader(folderPath);


% Select the bag for LaserScan messages
bagSel = select(bagReader, "Topic", "/multi_beam_lidar");
msgs = readMessages(bagSel);
%%
% Initialize arrays for storing converted Cartesian coordinates
y_all = [];
z_all = [];
ranges_all = [];

for i = 1:length(msgs)
    % i
    ranges = msgs{i}.ranges;
    % ranges_all = [ranges_all, ranges];
    angle_min = msgs{i}.angle_min;
    angle_max = msgs{i}.angle_max;
    angle_increment = msgs{i}.angle_increment;

    % Calculate angles for each scan
    angles = angle_min:angle_increment:angle_max;
    angles = angles(1:length(ranges)); % Ensure the angles array is not longer than ranges

    % Convert polar coordinates to Cartesian
    y = ranges .* cos(angles');
    z = ranges .* sin(angles');

    % Store the data
    y_all = [y_all, y];
    z_all = [z_all, z];
end
%%
figure;
hold on;
for i = 1:100
    plot3( 1:3085, y_all(i, :), z_all(i, :), 'LineWidth', 1.5); % z-axis is time
end
xlabel('Time');
ylabel('Y');
zlabel('Z');
title('3D Plot of X and Y Over Time');
grid on;
view(3); % 3D view
hold off;
%%
figure;
mesh(repmat(1:3085, 100, 1), y_all, z_all); % Time is Z-axis
xlabel('Time');
ylabel('Y');
zlabel('Z');
title('3D Mesh Plot of X and Y');
grid on;
view(3);
%%
clc;
clear;
% close all
% Define the bag file path

file1 = 'rosbag2_2025_01_29-11_00_35';


file3 = 'rosbag2_2025_01_29-15_34_13';
file4 = 'rosbag2_2025_01_29-15_37_17';
file5 = 'rosbag2_2025_01_29-15_40_07';
file6 = 'rosbag2_2025_01_29-20_03_17';
file7 = 'rosbag2_2025_01_30-12_52_26';
file8  = 'rosbag2_2025_01_30-13_12_05';
file9  = 'rosbag2_2025_01_30-15_15_16';
file10  = 'rosbag2_2025_01_30-15_30_40';

folderPath = fullfile(pwd, 'data', 'ros2Bags', file9);
bagReader = ros2bagreader(folderPath);

bagSel = select(bagReader, "Topic", "/multi_beam_lidar");
msgs = readMessages(bagSel);

maxRange = msgs{1}.range_max; % Adjust based on Lidar range
slamAlg = lidarSLAM(20, maxRange); % Grid resolution, max range

slamAlg.LoopClosureThreshold = 1000000000000000;
slamAlg.LoopClosureSearchRadius = 0.01; % Search radius for loop closure

angle_min = msgs{1}.angle_min;
angle_max = msgs{1}.angle_max;
angle_increment = msgs{1}.angle_increment;

% Calculate angles for each scan
angles = angle_min:angle_increment:angle_max;
angles = angles(1:length(msgs{1}.ranges)); 

%
clc
pc_all = [];

% for i = 299:1:length(msgs)+1
% for i = 1:10:length(msgs)
for i = 0:10:length(msgs)+1
% parfor i = 1000:1500
    if i ==0
        i=1;
    end
    ranges = msgs{i}.ranges;
    if length(unique(ranges)) >1
        disp('No duplicates in ranges');
    end

    % y = ranges .* cos(angles');
    % z = ranges .* sin(angles');


    if max(ranges) == Inf
        disp(['Skipping iteration ', num2str(i), ' due to Inf in ranges']);
        continue;
    end
    scan = lidarScan(ranges(50), angles(50));
    % ranges_trans = ranges .* sin(angles');
    % ranges_trans = ranges_trans - 2*max(ranges_trans);
    % % [ranges_trans, ranges]
    % scan = lidarScan(ranges_trans, angles);
    % addScan(slamAlg, scan);
    y = scan.Cartesian(:,1);
    z = scan.Cartesian(:,2);

    % Create a point cloud from the current scan (assuming X=0 for 2D case)
    ptCloud = pointCloud([ones(length(y),1)*(i/length(msgs))*100, y, z]);

    
    pc_all = [pc_all; ptCloud.Location]; % Accumulate points over scans

    if mod(i, 200) == 0
        disp(i);
    end
end 
%%
clc
close all
size_r = 9;
start_idx = -4;
end_idx = 4;
for i = 0:10:length(pc_all)+1
% for i = 0:10
    if i ==0
        i=1;
    end
    

    if start_idx <1
        ranges = vecnorm(pc_all(1:end_idx, 2:3), 2, 2);
    elseif end_idx >length(pc_all)
        ranges = vecnorm(pc_all(start_idx:length(pc_all), 2:3), 2, 2);
    else
        ranges = vecnorm(pc_all(start_idx:end_idx, 2:3), 2, 2);
    end
    numel_ = length(ranges);

    angles = linspace(angle_min, angle_max, numel_);

    


    % pc_all(i:i+size_r,2:3);
    

    ranges_trans = -ranges .* sin(angles');
    ranges_trans = ranges_trans - 2*(ranges_trans(floor(numel_/2)));
    % [ranges_trans, ranges]
    scan = lidarScan(ranges_trans, angles);
    addScan(slamAlg, scan);

    start_idx = start_idx+1;
    end_idx = end_idx+1;

    if mod(i, 200) == 0
    disp(i);
    end

end
    

% Create the final point cloud object
mapPointCloud = pointCloud(pc_all);

% Visualize the point cloud
figure;
pcshow(mapPointCloud);
title('Accumulated Point Cloud from Laser Scans');
xlabel('X'); ylabel('Y'); zlabel('Z');



% Generate the optimized map
[scans, optimizedPoses] = scansAndPoses(slamAlg);
%

% Visualize the optimized trajectory
figure;
show(slamAlg);
% plot(optimizedPoses(:,1),optimizedPoses(:,2))
title('SLAM Optimized Trajectory and Map');
xlabel("X [meters]")
ylabel("Y [meters]")


%%
clc
% Register successive point clouds using ICP
pcRef = mapPointCloud; % First point cloud as reference
for i = 2:length(msgs)
    pcCurrent = pointCloud([zeros(length(y),1), y, z]); % Convert to pointCloud
    tform = pcregistericp(pcCurrent, pcRef, 'Metric', 'pointToPlane'); % ICP
    
    % Transform the point cloud
    pcCurrent = pctransform(pcCurrent, tform);
    
    % Merge into a global map
    pcRef = pcmerge(pcRef, pcCurrent, 0.01);
end

% Visualize the final SLAM map
figure;
pcshow(pcRef);
title('ICP-Based SLAM Map');

% 
% %% not able to process
% /mavros/rc/override
% /mavros/state
% /mavros/setpoint_position/global
% /orb_slam2_stereo_node/status
% /motion