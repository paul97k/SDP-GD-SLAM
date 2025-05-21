clc;
clear all;
addpath('helpers');
close all


% 
% 
% % Specify the directory
% directory = 'data\lifted_model\discrete\';
% 
% % Get a list of all files in the directory
% fileList = dir(directory);
% 
% % Loop through the list and print filenames
% fprintf('Files in %s:\n', directory);
% for i = 1:length(fileList)
%     if ~fileList(i).isdir % Skip directories
%         fprintf('%s\n', fileList(i).name);
%     end
% end
% 
% 
% Initialization


bag_names = get_bag_name();


bag_name = bag_names{37}
%%
clc
scaleFactor = 1;
nT = 100;
scaled = false;
batchSize = 1;

number_var = 10;
increment = 0.02;
start_noise = 0.02;
variance_noise_vec = start_noise:increment:(start_noise + increment * (number_var - 1));
mean_noise = 0;


bagStart = 12;
for filenname_i = 1:2
    
    for bag_id = 12:24
        if filenname_i ==1
            filename = getFileNameLiftedModel(bag_id, nT, scaled, scaleFactor);
        else
            filename = getFileNameLiftedModel_batches(bag_id, nT, scaled, scaleFactor, batchSize);
        end
        
        data = load(filename);
        for var_i = 1:number_var
            variance_noise = variance_noise_vec(var_i);
            addGaussianNoiseAndSave(filename, mean_noise, variance_noise);
        end
    end
end

function addGaussianNoiseAndSave(filename, mean_noise, variance_noise)
    % Load the data from the file
    data = load(filename);

    
    

    % Ensure measured_y exists in the data structure
    if isfield(data, 'measured_y')
        % Generate Gaussian noise
        noise = mean_noise + sqrt(variance_noise) * randn(size(data.measured_y));
        
        % Add noise to measured_y
        
        % plot(data.measured_y )
        % hold on

        data.measured_y = data.measured_y + noise;
        % 
        % plot(data.measured_y )
        % legend('no noise', 'added noise')

        % Update the filename to reflect the noise parameters
        % new_filename = strrep(filename, 'lidar_0_0', sprintf('lidar_%d_%d', mean_noise, variance_noise));
        new_filename = strrep(filename, 'lidar_0_0', sprintf('lidar_%d_%.2f', mean_noise, variance_noise));


        % Save the modified data with the new measured_y to the updated filename
        save(new_filename, '-struct', 'data');
        fprintf('Modified data saved to: %s\n', new_filename);
    else
        error('The field "measured_y" does not exist in the loaded data.');
    end
end


function filename = getFileNameLiftedModel_batches(bag_id, nT, scaled, scaleFactor, Batchsize)
    bag_names = get_bag_name();
    

    % Validate the bag_id
    bag_name = bag_names{bag_id};
    % if isKey(bag_names, bag_id)
    %     bag_name = bag_names(bag_id);
    % else
    %     error('Invalid bag_id: %d. Please choose a valid ID.', bag_id);
    % end
    
    % Construct the filename
    if scaled
        filename = sprintf('%s_nT_%d_scaled_%d.mat', bag_name, nT, scaleFactor);
    else
        filename = sprintf('%s_nT_%d_Batchsize_%d.mat', bag_name, nT, Batchsize);
    end


    % Prepend the folder path
    folder_path = 'data/lifted_model/Batches';
    filename = fullfile(folder_path, filename);

    % disp(['load Lifted Model from ', filename])
    
end


