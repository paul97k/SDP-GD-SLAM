

function filename = getFileNameLiftedModel(bag_id, nT, scaled, scaleFactor)
    bag_names = get_bag_name();
    

    % % Validate the bag_id
    % if isKey(bag_names, bag_id)
    %     bag_name = bag_names(bag_id);
    % else
    %     error('Invalid bag_id: %d. Please choose a valid ID.', bag_id);
    % end

    
    bag_name = bag_names{bag_id};

    % Construct the filename
    if scaled
        filename = sprintf('%s_nT_%d_scaled_%d.mat', bag_name, nT, scaleFactor);
    else
        filename = sprintf('%s_nT_%d.mat', bag_name, nT);
    end


    % Prepend the folder path
    folder_path = 'data/lifted_model/discrete';
    filename = fullfile(folder_path, filename);

    % disp(['load Lifted Model from ', filename])
    
end