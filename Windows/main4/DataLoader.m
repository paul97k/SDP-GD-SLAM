classdef DataLoader < handle
    properties
        fullPath


        sonarRange_Angle % Sonar data
        sonarPositions % Sonar data
        sonarPositions4 % Sonar data
        imu   % IMU sensor data
        dvl   % Doppler Velocity Log (DVL) data

        thrusters % Thruster inputs & outputs
        eulerGT % Euler ground truth (GT)
        positionGT % Position ground truth (GT)
        velocityGT % velocity ground truth (GT)

        positionIntegrated

        %identification
        identification
        
        % New properties for NED velocity
        NED

        EKF
        downSampleFreq

        CMD
        hyrdro


    end
    
    methods
        % Constructor to load the data
        % function obj = DataLoader(dataFolder, fileName)
        function obj = DataLoader(fileName)
            % if nargin < 2
            %     fileName = 'processed_data.mat'; % Default file name
            % end
            % obj.fullPath = fullfile(dataFolder, fileName);
            obj.fullPath = fileName;
            
            if exist(obj.fullPath, 'file')
                data = load(obj.fullPath);
                
                % **Sonar Data**
                obj.sonarRange_Angle = struct( ...
                    'angles', data.angles, ...
                    'ranges', data.ranges, ...
                    'timestamps', data.timestamps_sonar ...
                );

                if isfield(data, 'range_0')
                    obj.sonarPositions4 = struct( ...
                        'ranges0', data.range_0, ...
                        'timestamps0', data.time_0, ...
                        'ranges1', data.range_1, ...
                        'timestamps1', data.time_1, ...
                        'ranges2', data.range_2, ...
                        'timestamps2', data.time_2, ...
                        'ranges3', data.range_3, ...
                        'timestamps3', data.time_3...
                    );
                end

                 % **IMU Data**
                obj.imu = struct( ...
                    'angular_velocities', data.angular_velocities, ...
                    'linear_accelerations', data.linear_accelerations, ...
                    'rolls', data.yaws, ... % Fix variable name mix-up
                    'pitches', data.rolls, ...
                    'yaws', data.pitches, ...
                    'timestamps', data.timestamps_imu ...
                );

                % **DVL (Doppler Velocity Log) Data**
                obj.dvl = struct( ...
                    'vx', data.vz, ... % changed frames also. x=z, y=y, z=x
                    'vy', data.vy, ...
                    'vz', data.vx, ...
                    'timestamps', data.timestamps_dvl ...
                );

                % 
                % disp('Output Thruster Data Sizes:');
                % disp(['thruster0_in_t: ', mat2str(size(data.thruster0_in_t))]);
                % disp(['thruster1_in_t: ', mat2str(size(data.thruster1_in_t))]);
                % disp(['thruster2_in_t: ', mat2str(size(data.thruster2_in_t))]);
                % % 
                % disp(['thruster0_in: ', mat2str(size(data.thruster0_in))]);
                % disp(['thruster1_in: ', mat2str(size(data.thruster1_in))]);
                % disp(['thruster2_in: ', mat2str(size(data.thruster2_in))]);

                % 
                % disp('          out         ')
                % disp(['thruster0_out_t: ', mat2str(size(data.thruster0_out_t))]);
                % disp(['thruster1_out_t: ', mat2str(size(data.thruster1_out_t))]);
                % disp(['thruster2_out_t: ', mat2str(size(data.thruster2_out_t))]);
                % 
                % disp(['thruster0_out: ', mat2str(size(data.thruster0_out))]);
                % disp(['thruster1_out: ', mat2str(size(data.thruster1_out))]);
                % disp(['thruster2_out: ', mat2str(size(data.thruster2_out))]);

                % [data.thruster0_in_t, data.thruster0_in, ...
                %  data.thruster1_in_t, data.thruster1_in, ...
                %  data.thruster2_in_t, data.thruster2_in] = ...
                %  obj.align_signals(data.thruster0_in_t, data.thruster0_in, ...
                %                    data.thruster1_in_t, data.thruster1_in, ...
                %                    data.thruster2_in_t, data.thruster2_in);

                 %    [data.thruster0_out_t, data.thruster0_out, ...
                 % data.thruster1_out_t, data.thruster1_out, ...
                 % data.thruster2_out_t, data.thruster2_out] = ...
                 % obj.align_signals(data.thruster0_out_t, data.thruster0_out, ...
                 %                   data.thruster1_out_t, data.thruster1_out, ...
                 %                   data.thruster2_out_t, data.thruster2_out);

                [common_time, data.thruster0_in, data.thruster1_in, data.thruster2_in] = ...
                    interpolate_signals(obj, data.thruster0_in_t, data.thruster0_in, ...
                                            data.thruster1_in_t, data.thruster1_in, ...
                                            data.thruster2_in_t, data.thruster2_in);
                data.thruster0_in_t = common_time;
                data.thruster1_in_t = common_time;
                data.thruster2_in_t = common_time;


                [common_time, data.thruster0_out, data.thruster1_out, data.thruster2_out] = ...
                    interpolate_signals(obj, data.thruster0_out_t, data.thruster0_out, ...
                                            data.thruster1_out_t, data.thruster1_out, ...
                                            data.thruster2_out_t, data.thruster2_out);
                data.thruster0_out_t = common_time;
                data.thruster1_out_t = common_time;
                data.thruster2_out_t = common_time;

                % disp('------')
                % disp(['thruster0_in_t: ', mat2str(size(data.thruster0_in_t))]);
                % disp(['thruster1_in_t: ', mat2str(size(data.thruster1_in_t))]);
                % disp(['thruster2_in_t: ', mat2str(size(data.thruster2_in_t))]);
                % % 
                % disp(['thruster0_in: ', mat2str(size(data.thruster0_in))]);
                % disp(['thruster1_in: ', mat2str(size(data.thruster1_in))]);
                % disp(['thruster2_in: ', mat2str(size(data.thruster2_in))]);

                % 
                % disp('          out         ')
                % disp(['thruster0_out_t: ', mat2str(size(data.thruster0_out_t))]);
                % disp(['thruster1_out_t: ', mat2str(size(data.thruster1_out_t))]);
                % disp(['thruster2_out_t: ', mat2str(size(data.thruster2_out_t))]);
                % 
                % disp(['thruster0_out: ', mat2str(size(data.thruster0_out))]);
                % disp(['thruster1_out: ', mat2str(size(data.thruster1_out))]);
                % disp(['thruster2_out: ', mat2str(size(data.thruster2_out))]);


                % **Thruster Data (Inputs & Outputs)**
                obj.thrusters = struct( ...
                    'input', struct( ...
                        'thruster0', data.thruster0_in, ...
                        'thruster1', data.thruster1_in, ...
                        'thruster2', data.thruster2_in, ...
                        'timestamps', [data.thruster0_in_t; data.thruster1_in_t; data.thruster2_in_t] ...
                    ), ...
                    'output', struct( ...
                        'thruster0', data.thruster0_out, ...
                        'thruster1', data.thruster1_out, ...
                        'thruster2', data.thruster2_out, ...
                        'timestamps', [data.thruster0_out_t; data.thruster1_out_t; data.thruster2_out_t] ...
                    ) ...
                );

                % **Euler Angles Ground Truth (GT)**
                obj.eulerGT = struct( ...
                    'roll', data.roll_anglesGT, ...
                    'pitch', data.pitch_anglesGT, ...
                    'yaw', data.yaw_anglesGT, ...
                    'timestamps', data.timestamps_eulerG ...
                );

                % **Position Ground Truth (GT)**
                obj.positionGT = struct( ...
                    'x_positions', data.x_positionsGT, ...
                    'y_positions', data.y_positionsGT, ...
                    'z_positions', data.z_positionsGT, ...
                    'timestamps', data.timestamps_pose ...
                );



                xGT =  obj.positionGT.x_positions;
                yGT =  obj.positionGT.y_positions;
                zGT =  obj.positionGT.y_positions;

                timeGT =  obj.positionGT.timestamps;
                dt = mean(diff(timeGT));
                
                VxGT = diff(xGT) / dt; % X velocity
                VyGT = diff(yGT) / dt; % Y velocity
                VzGT = diff(zGT) / dt; % Y velocity
                
                % To match the length with the original data, append a value
                VxGT = [VxGT VxGT(end)];
                VyGT = [VyGT VyGT(end)];
                VzGT = [VzGT VzGT(end)];
                
                % **Velocity Ground Truth (GT)**
                obj.velocityGT = struct( ...
                    'x_vel', VxGT, ...
                    'y_vel', VyGT, .....
                    'z_vel', VzGT, .....
                    'timestamps',timeGT ...
                );

                if isfield(data, 'cmd_linear_x')
                    obj.CMD = struct( ...
                        'cmd_linear_x', data.cmd_linear_x, ...
                        'cmd_linear_y', data.cmd_linear_y, ...
                        'cmd_linear_z', data.cmd_linear_z, ... 
                        'cmd_angular_x', data.cmd_angular_x, ...
                        'cmd_angular_y', data.cmd_angular_y, ...
                        'cmd_angular_z', data.cmd_angular_z, ...
                        'cmd_timestamps', data.cmd_timestamps ...
                    );
                else
                    disp('cmd_linear_x does NOT exist in data.');
                end

                if isfield(data, 'hyrdro_linear_x')
                    obj.hyrdro = struct( ...
                        'hyrdro_linear_x', data.hyrdro_linear_x, ...
                        'hyrdro_linear_y', data.hyrdro_linear_y, ...
                        'hyrdro_linear_z', data.hyrdro_linear_z, ... 
                        'hyrdro_angular_x', data.hyrdro_angular_x, ...
                        'hyrdro_angular_y', data.hyrdro_angular_y, ...
                        'hyrdro_angular_z', data.hyrdro_angular_z, ...
                        'hyrdro_timestamps', data.hyrdro_timestamps ...
                    );
                else
                    disp('hyrdro_linear_x does NOT exist in data.');
                end


                % disp('Data successfully loaded.');
            else
                error('File %s not found in folder %s', fileName);
            end
        end


        % 
        % function mapReturn = CreateKalmanMAP(obj, x_estimates, skip_i, skip_j)
        %     % Function to generate a 3D sonar map for ICP using Kalman filter estimates
        %     %
        %     % Parameters:
        %     % - obj: The object containing sonar and ground truth data
        %     % - x_estimates: Kalman filter estimated positions [2 x N]
        %     % - skip_i: Step size for time indexing (to reduce processing)
        %     % - skip_j: Step size for sonar indexing (to reduce noise)
        %     %
        %     % Returns:
        %     % - map: A 3×M matrix containing the generated 3D points for ICP
        % 
        % 
        % 
        %     % Extract sonar positions
        %     time_sonar = obj.sonarPositions.timestamps;
        %     y_sonar = obj.sonarPositions.y_positions;
        %     z_sonar = obj.sonarPositions.z_positions;
        % 
        %     % Initialize the map storage
        %     map = [];
        % 
        % 
        %     iFactor = length(y_sonar)/length(x_estimates);
        % 
        %     for i = 1:skip_i:length(y_sonar)
        %         xEst_i = max([1,round(i/iFactor)]);
        % 
        %         % Select every `skip_j`-th sonar point
        %         y_subset = y_sonar(i, 1:skip_j:end);
        %         z_subset = z_sonar(i, 1:skip_j:end);
        %         % x_estimates(:, xEst_i)
        %         % Compute 3D points in world coordinates
        %         x_values = x_estimates(1, xEst_i) * ones(size(y_subset));
        %         y_values = y_subset + x_estimates(2, xEst_i);
        %         z_values = z_subset;
        % 
        %         % Append new points to the map
        %         map = [map, [x_values; y_values; z_values]];
        %     end
        %     mapReturn = map;
        % end
        % 


        function obj = DownsampleData(obj)
            % 

            % computeXZsonar(obj)
            % x_estimatesGTXY = zeros(9,length(obj.positionGT.x_positions)); % the other 7 are just so that the function can work properly
            % x_estimatesGTXY(1,:) =obj.positionGT.x_positions';
            % x_estimatesGTXY(2,:) =obj.positionGT.y_positions';
            % 
            % skip_i = 2;
            % skip_j = 20;
            % map_pointsGT  = CreateKalmanMAP(obj, x_estimatesGTXY, skip_i, skip_j);
            % 
            % 
            % figure
            % scatter3(map_pointsGT(1,:), map_pointsGT(2,:), map_pointsGT(3,:), 5, 'r', 'filled'); 
            % 
            % hold on
            % scatter3(obj.positionGT.x_positions, obj.positionGT.y_positions, 5, 'r', 'filled'); 
            % 
            % 
            % 
            % [~, obj.sonarRange_Angle.angles] =  f_downsample(obj, obj.sonarRange_Angle.timestamps, obj.sonarRange_Angle.angles);
            % [obj.sonarRange_Angle.timestamps, ...
            %     obj.sonarRange_Angle.ranges] =  f_downsample(obj, obj.sonarRange_Angle.timestamps, obj.sonarRange_Angle.ranges);
            % 
            % computeXZsonar(obj)


            
            [~, a] =   f_downsample(obj, obj.imu.timestamps, obj.imu.angular_velocities(:,1));
            [~, b] =   f_downsample(obj, obj.imu.timestamps, obj.imu.angular_velocities(:,2));
            [~, c] =   f_downsample(obj, obj.imu.timestamps, obj.imu.angular_velocities(:,3));
            obj.imu.angular_velocitie= [a b c];
            
            [~, a] = f_downsample(obj, obj.imu.timestamps, obj.imu.linear_accelerations(:,1));
            [~, b] = f_downsample(obj, obj.imu.timestamps, obj.imu.linear_accelerations(:,2));
            [~, c] = f_downsample(obj, obj.imu.timestamps, obj.imu.linear_accelerations(:,3));
            obj.imu.linear_accelerations= [a b c];


            [~, obj.imu.rolls] =                f_downsample(obj, obj.imu.timestamps, obj.imu.rolls);
            [~, obj.imu.pitches] =              f_downsample(obj, obj.imu.timestamps, obj.imu.pitches);
            [obj.imu.timestamps, obj.imu.yaws] =f_downsample(obj, obj.imu.timestamps, obj.imu.yaws);

            [~, obj.dvl.vx] =  f_downsample(obj, obj.dvl.timestamps, obj.dvl.vx);
            [~, obj.dvl.vy] =  f_downsample(obj, obj.dvl.timestamps, obj.dvl.vy);
            [obj.dvl.timestamps, obj.dvl.vz] =  f_downsample(obj, obj.dvl.timestamps, obj.dvl.vz);

            
            [a, obj.thrusters.input.thruster0] =  f_downsample(obj, obj.thrusters.input.timestamps(1,:), obj.thrusters.input.thruster0);
            [b, obj.thrusters.input.thruster1] =  f_downsample(obj, obj.thrusters.input.timestamps(2,:), obj.thrusters.input.thruster1);
            [c, obj.thrusters.input.thruster2] =  f_downsample(obj, obj.thrusters.input.timestamps(3,:), obj.thrusters.input.thruster2);

            obj.thrusters.input.timestamps= [a b c]';
            % 
            [a, obj.thrusters.output.thruster0] =  f_downsample(obj, obj.thrusters.output.timestamps(1,:), obj.thrusters.output.thruster0);
            [b, obj.thrusters.output.thruster1] =  f_downsample(obj, obj.thrusters.output.timestamps(2,:), obj.thrusters.output.thruster1);
            [c, obj.thrusters.output.thruster2] =  f_downsample(obj, obj.thrusters.output.timestamps(3,:), obj.thrusters.output.thruster2);
            obj.thrusters.output.timestamps= [a b c]';


            [~, obj.eulerGT.roll] =  f_downsample(obj, obj.eulerGT.timestamps, obj.eulerGT.roll);
            [~, obj.eulerGT.pitch] =  f_downsample(obj, obj.eulerGT.timestamps, obj.eulerGT.pitch);
            [obj.eulerGT.timestamps, obj.eulerGT.yaw] =  f_downsample(obj, obj.eulerGT.timestamps, obj.eulerGT.yaw);

            % figure
            % plot(obj.sonarPositions4.timestamps0, obj.sonarPositions4.ranges0)
            % 
            if isfield(obj.sonarPositions4, 'ranges0')
                [obj.sonarPositions4.timestamps0, obj.sonarPositions4.ranges0] =  f_downsample(obj, obj.sonarPositions4.timestamps0, obj.sonarPositions4.ranges0);
                [obj.sonarPositions4.timestamps1, obj.sonarPositions4.ranges1] =  f_downsample(obj, obj.sonarPositions4.timestamps1, obj.sonarPositions4.ranges1);
                [obj.sonarPositions4.timestamps2, obj.sonarPositions4.ranges2] =  f_downsample(obj, obj.sonarPositions4.timestamps2, obj.sonarPositions4.ranges2);
                [obj.sonarPositions4.timestamps3, obj.sonarPositions4.ranges3] =  f_downsample(obj, obj.sonarPositions4.timestamps3, obj.sonarPositions4.ranges3);
            end
            % hold on
            % plot(obj.sonarPositions4.timestamps0, obj.sonarPositions4.ranges0)

            % [~, obj.positionGT.x_positions] =  f_downsample(obj, obj.positionGT.timestamps, obj.positionGT.x_positions);
            % [~, obj.positionGT.y_positions] =  f_downsample(obj, obj.positionGT.timestamps, obj.positionGT.y_positions);
            % [obj.positionGT.timestamps, obj.positionGT.z_positions] =  f_downsample(obj, obj.positionGT.timestamps, obj.positionGT.z_positions);


            % computeXZsonar(obj)
            % x_estimatesGTXY = zeros(9,length(obj.positionGT.x_positions)); % the other 7 are just so that the function can work properly
            % x_estimatesGTXY(1,:) =obj.positionGT.x_positions';
            % x_estimatesGTXY(2,:) =obj.positionGT.y_positions';

            % map_pointsGT  = CreateKalmanMAP(obj, x_estimatesGTXY, skip_i, skip_j);
            % scatter3(map_pointsGT(1,:), map_pointsGT(2,:), map_pointsGT(3,:), 5, 'g', 'filled'); 
            % 
            % scatter3(obj.positionGT.x_positions, obj.positionGT.y_positions, 5, 'g', 'filled'); 
            % 
            % legend("before", "after")



            % 
            % [~, obj.velocityGT.x_vel] =  f_downsample(obj, obj.velocityGT.timestamps, obj.velocityGT.x_vel);
            % [~, obj.velocityGT.y_vel] =  f_downsample(obj, obj.velocityGT.timestamps, obj.velocityGT.y_vel);
            % [obj.velocityGT.timestamps, obj.velocityGT.z_vel] =  f_downsample(obj, obj.velocityGT.timestamps, obj.velocityGT.z_vel);

            [~, obj.CMD.cmd_linear_x] =  f_downsample(obj, obj.CMD.cmd_timestamps, obj.CMD.cmd_linear_x);
            [~, obj.CMD.cmd_linear_y] =  f_downsample(obj, obj.CMD.cmd_timestamps, obj.CMD.cmd_linear_y);
            [~, obj.CMD.cmd_linear_z] =  f_downsample(obj, obj.CMD.cmd_timestamps, obj.CMD.cmd_linear_z);
            [~, obj.CMD.cmd_angular_x] =  f_downsample(obj, obj.CMD.cmd_timestamps, obj.CMD.cmd_angular_x);
            [~, obj.CMD.cmd_angular_y] =  f_downsample(obj, obj.CMD.cmd_timestamps, obj.CMD.cmd_angular_y);
            [obj.CMD.cmd_timestamps, obj.CMD.cmd_angular_z] =  f_downsample(obj, obj.CMD.cmd_timestamps, obj.CMD.cmd_angular_z);
            if size(obj.hyrdro.hyrdro_timestamps,1)>0
                [~, obj.hyrdro.hyrdro_linear_x] =  f_downsample(obj, obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_linear_x);
                [~, obj.hyrdro.hyrdro_linear_y] =  f_downsample(obj, obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_linear_y);
                [~, obj.hyrdro.hyrdro_linear_z] =  f_downsample(obj, obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_linear_z);
                [~, obj.hyrdro.hyrdro_angular_x] =  f_downsample(obj, obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_angular_x);
                [~, obj.hyrdro.hyrdro_angular_y] =  f_downsample(obj, obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_angular_y);
                [obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_angular_z] =  f_downsample(obj, obj.hyrdro.hyrdro_timestamps, obj.hyrdro.hyrdro_angular_z);
            end



            [~, obj.positionIntegrated.X_intg] =  f_downsample(obj, obj.positionIntegrated.timestamps, obj.positionIntegrated.X_intg);
            [~ , obj.positionIntegrated.Y_intg] =  f_downsample(obj, obj.positionIntegrated.timestamps, obj.positionIntegrated.Y_intg);
            [obj.positionIntegrated.timestamps, obj.positionIntegrated.Z_intg] =  f_downsample(obj, obj.positionIntegrated.timestamps, obj.positionIntegrated.Z_intg);
            
            
            [~, obj.NED.V_x] =  f_downsample(obj, obj.NED.timestamps, obj.NED.V_x);
            [~, obj.NED.V_y] =  f_downsample(obj, obj.NED.timestamps, obj.NED.V_y);
            [obj.NED.timestamps, obj.NED.V_z] =  f_downsample(obj, obj.NED.timestamps, obj.NED.V_z);

        end

        function [timeSignal1_new, signal1_new] = f_downsample(obj, timeSignal1, signal1)
                % Ensure timeSignal1 is a column vector
                ogS = timeSignal1;
                ogT = signal1;
                if size(timeSignal1,1) ==size(signal1,2)
                    signal1 = signal1';
                end


                    
                % Ensure unique time points to prevent interpolation errors
                [timeSignal1, unique_idx] = unique(timeSignal1);
                if size(signal1,2)< size(unique_idx,1)
                    signal1 = signal1(unique_idx,:);
                else
                    signal1 = signal1(:,unique_idx);
                end


                % Define new time grid based on downsampling frequency
                skip_ix = 1 / obj.downSampleFreq;
                timeSignal1_new = (timeSignal1(1):skip_ix:timeSignal1(end))'; % Ensure column vector
        

                % Perform interpolation
                signal1_new = interp1(timeSignal1, signal1, timeSignal1_new, 'linear', 'extrap');
        
        end



    


        % Method to compute NED coordinates
        function obj = computeNED_AddNoise(obj, addNoise, noise_std)
            % Convert angles to radians
            if nargin < 3
                noise_std = 0.01; % Default noise standard deviation
            end

            % % Ensure unique time points to prevent interpolation errors
            % [timeSignal1, unique_idx] = unique(timeSignal1);
            % if size(signal1,2)< size(unique_idx,1)
            %     signal1 = signal1(unique_idx,:);
            % else
            %     signal1 = signal1(:,unique_idx);
            % end


            [~, obj.imu.rolls, obj.dvl.vx] = interpolate_signals2Input(obj, obj.imu.timestamps, obj.imu.rolls,    obj.dvl.timestamps, obj.dvl.vx);
            [~, obj.imu.pitches, obj.dvl.vy] = interpolate_signals2Input(obj, obj.imu.timestamps, obj.imu.pitches,  obj.dvl.timestamps, obj.dvl.vy);
            [commonTime, obj.imu.yaws, obj.dvl.vz] = interpolate_signals2Input(obj, obj.imu.timestamps, obj.imu.yaws,     obj.dvl.timestamps, obj.dvl.vz);
            obj.imu.timestamps = commonTime;
            obj.dvl.timestamps = commonTime';

            rolls_rad = deg2rad(obj.imu.rolls);
            pitches_rad = deg2rad(obj.imu.pitches);
            yaws_rad = deg2rad(obj.imu.yaws);

            % Preallocate velocity in NED frame
            V_NED = zeros(length(obj.dvl.vx), 3);


            for i = 1:length(obj.dvl.vx)
                % Extract current roll, pitch, yaw
                phi = rolls_rad(i);
                theta = pitches_rad(i);
                psi = yaws_rad(i);

                % Compute rotation matrix from body to NED
                R = [ cos(psi)*cos(theta),  sin(psi)*cos(theta), -sin(theta);
                     -cos(psi)*sin(phi)*sin(theta) - sin(psi)*cos(phi), -sin(psi)*sin(phi)*sin(theta) + cos(psi)*cos(phi), -sin(phi)*cos(theta);
                     -cos(psi)*cos(phi)*sin(theta) + sin(psi)*sin(phi), -sin(psi)*cos(phi)*sin(theta) - cos(psi)*sin(phi), cos(phi)*cos(theta) ];

                % Transform velocity from body to NED frame
                V_body = [obj.dvl.vx(i); obj.dvl.vy(i); obj.dvl.vz(i)];
                V_NED(i, :) = (R * V_body)'; % Transpose to get row vector
            end

            z_meas = [V_NED(:,1) V_NED(:,2) V_NED(:,3)];


            obj.positionIntegrated.X_intg = cumtrapz(commonTime, V_NED(:,1)); % X position
            obj.positionIntegrated.Y_intg = cumtrapz(commonTime, V_NED(:,2)); % Y position
            obj.positionIntegrated.Z_intg = cumtrapz(commonTime, V_NED(:,3)); % Y position
            obj.positionIntegrated.timestamps = commonTime; % 
            % Add measurement noise

                
                % cov_meas = zeros(2, 2); % Preallocate a 4x1 vector for covariance results
                % 
                % for i = 1:2
                %     for j = 1:2
                %         cov_meas(i,j) = cov(z_meas(i, :)- z_noise(j, :));
                %     end
                % end
                
                % z_meas = z_noise;
            % Store computed velocities in NED frame
            if addNoise
                bias = 0.00;                
                noise = noise_std * randn(size(z_meas));
                
                z_noise = z_meas + noise +bias;
                obj.NED.V_x = z_noise(:,1);
                obj.NED.V_y = z_noise(:,2);
                obj.NED.V_z = z_noise(:,3);
                obj.NED.timestamps = commonTime;
                % disp('NED coordinate transformation completed and noise added.');
            else
                obj.NED.V_x = z_meas(:,1);
                obj.NED.V_y = z_meas(:,2);
                obj.NED.V_z = z_meas(:,3);
                obj.NED.timestamps = commonTime;
            disp('NED coordinate transformation completed and NO noise added!!!!!');
            end

        end


        function obj = computeXZsonar(obj, downSample)

            ranges = obj.sonarRange_Angle.ranges; % 793x396
            angles = obj.sonarRange_Angle.angles; % 793x396
            timestamps = obj.sonarRange_Angle.timestamps;
            
            % Initialize storage matrices
            all_z = []; % Stores all x-coordinates
            all_y = []; % Stores all y-coordinates
            % figure
            for scan_idx = 1:length(timestamps)
                r = ranges(scan_idx, :); % Extract range values at scan index
                theta = angles(scan_idx, :); % Extract corresponding angles
                % Convert polar to Cartesian for visualization
                [z, y] = pol2cart(theta, r);
                
                all_z = [all_z; -z]; % Append x-coordinates as new row
                all_y = [all_y; y]; % Append y-coordinates as new row
            end
            if downSample
                [time_sonar, y_sonar, z_sonar] = obj.cut_downsample_sonar_data(timestamps, timestamps, all_y, all_z);
            else
                time_sonar  =timestamps;
                y_sonar =all_y;
                z_sonar = all_z;
            end
            obj.sonarPositions = struct( ...
                'y_positions', y_sonar, ...
                'z_positions', z_sonar, ...
                'timestamps',time_sonar ...
            );


            
        end



        function plotData(obj)
            % Plot body-frame velocities
            figure;
            
            % Plot North velocity (vx)
            subplot(3,1,1);
            plot(obj.timestamps_dvl, obj.vx, 'r', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('V_N (m/s)');
            title('North Velocity (vx) - Body Frame');
            grid on;
    
            % Plot East velocity (vy)
            subplot(3,1,2);
            plot(obj.timestamps_dvl, obj.vy, 'g', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('V_E (m/s)');
            title('East Velocity (vy) - Body Frame');
            grid on;
    
            % Plot Down velocity (vz)
            subplot(3,1,3);
            plot(obj.timestamps_dvl, obj.vz, 'b', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('V_D (m/s)');
            title('Down Velocity (vz) - Body Frame');
            grid on;
   
            % Plot NED velocities
            figure
    
            % Plot North velocity (V_N)
            subplot(3,1,1);
            plot(obj.timestamps_dvl, obj.V_x, 'r', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('V_N (m/s)');
            title('North Velocity (V_N) - NED Frame');
            grid on;
    
            % Plot East velocity (V_E)
            subplot(3,1,2);
            plot(obj.timestamps_dvl, obj.V_y, 'g', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('V_E (m/s)');
            title('East Velocity (V_E) - NED Frame');
            grid on;
    
            % Plot Down velocity (V_D)
            subplot(3,1,3);
            plot(obj.timestamps_dvl, obj.V_z, 'b', 'LineWidth', 1.5);
            xlabel('Time (s)');
            ylabel('V_D (m/s)');
            title('Down Velocity (V_D) - NED Frame');
            grid on;
    
            % Plot thruster input signals
            figure;
            plot(obj.thruster0_in_t, obj.thruster0_in + obj.thruster1_in, 'LineWidth', 1.5);
            xlabel('Sample Index');
            ylabel('Thruster Input');
            title('Thruster Input Signals');
            xlim([175,180]);
            legend('Thruster 0 + 1'); % You can add more plots if needed
            grid on;
    
            % Plot thruster output signals
            figure;
            plot(obj.thruster0_out_t, obj.thruster0_out + obj.thruster1_out, 'LineWidth', 1.5);
            xlabel('Sample Index');
            ylabel('Thruster Output');
            title('Thruster Output Signals');
            xlim([175,180]);
            legend('Thruster 0 + 1'); % You can add more plots if needed
            grid on;
            
            disp('Plots generated successfully.');
        end

        function identifySystem(obj)

            id_time = obj.CMD.cmd_timestamps;
            V_x_new = obj.NED.V_x;
            V_y_new =  obj.NED.V_y;
            V_z_new =  obj.NED.V_z;
        
            rolls = obj.imu.rolls;
            pitches =  obj.imu.pitches;
            yaws =  obj.imu.yaws;

            t=  obj.imu.timestamps;
        
            % Ensure column vectors
            id_time = id_time(:);
            V_x_new = V_x_new(:);
            V_y_new = V_y_new(:);
            V_z_new = V_z_new(:);
            
            rolls = rolls(:);
            pitches = pitches(:);
            yaws = yaws(:);

            % t = obj.identification.timestamps;
            X_intg = obj.positionIntegrated.X_intg; % X position
            Y_intg = obj.positionIntegrated.Y_intg; % Y position
            Z_intg = obj.positionIntegrated.Z_intg; % Y position

            % Input = [obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z ...
            %     obj.CMD.cmd_angular_x obj.CMD.cmd_angular_y obj.CMD.cmd_angular_z];
            % Output = [V_x_new V_y_new V_z_new rolls pitches yaws];


            % Input = [obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z];
            % Output = [V_x_new V_y_new V_z_new];
            zeroInput = obj.CMD.cmd_linear_x*0 ;
            Input = [zeroInput zeroInput zeroInput obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z];
            Output = [X_intg Y_intg Z_intg V_x_new V_y_new V_z_new];

            % cutIdx = 0.4*length(Input);
            % Input = Input(1:cutIdx,:);
            % Output = Output(1:cutIdx,:);

            dt = mean(diff(id_time));
            APar = zeros(1,9);
            BPar = zeros(1,9);
            params0 = [APar BPar];
                        
          
            size(params0)
    
            % Define input-output data for identification
            if size(Input,1) > size(Output,1) %the difference is usually only 1 
                Input = Input(1:size(Output,1),:);
                id_time = id_time(1:size(Output,1));

            elseif size(Input,1) < size(Output,1)
                Output = Output(1:size(Input,1),:);
            end
            Ts = 1/dt;
            data = iddata(Output, Input, dt);


        % modelOrder = 6;  %
        % opt = ssestOptions('Display', 'on');
        % opt = ssestOptions('Display', 'on');
        % sys_ss = ssest(data, modelOrder, 'Ts', dt, opt,'DisturbanceModel', 'none');

        % 

            % Define grey-box model
            sys_grey = idgrey(@parametric_model, params0, 'd', dt);

            opt = greyestOptions('InitialState', 'zero', 'Display', 'on');
            opt.SearchOptions.MaxIterations = 300;  % Set maximum number of iterations to 500
            % Estimate state-space model
            opt.SearchOptions.Tolerance = 1e-9;
            sys_ss = greyest(data, sys_grey, opt);





            % % Display identified state-space model
            % disp('Identified State-Space Model:');
            % disp(sys_ss);
            % 
            % Extract state-space matrices
            [A, B, C, D] = ssdata(sys_ss);
            % disp('Estimated State-Space Matrices:');
            % disp('A = '), disp(A);
            % disp('B = '), disp(B);
            % disp('C = '), disp(C);
            % disp('D = '), disp(D);

            % **Euler Angles Ground Truth (GT)**
            obj.identification = struct( ...
                'Input', Input, ...
                'Output', Output, ...
                'A', A, ...
                'B', B, ...
                'C', C, ...
                'D', D, ...
                'dt', dt, ...
                'sys_ss', sys_ss, ...
                'timestamps', id_time ...
            );

        figure;
        compare(data, sys_ss)
        [ymod,fit,ic] =compare(data, sys_ss);        
        % Compute the error
        error = Output - ymod.OutputData;
        disp(['fit: ', num2str(fit')]);
        disp(['Mean Absolute Error: ', num2str(mean(abs(error)))]);
        disp(['Root Mean Square Error (RMSE): ', num2str(sqrt(mean(error.^2)))]);

            % disp('System identification completed successfully.');
        end




        function identifySystem2(obj,APar, BPar, maxItt)

            id_time = obj.CMD.cmd_timestamps;
            V_x_new = obj.NED.V_x;
            V_y_new =  obj.NED.V_y;
            V_z_new =  obj.NED.V_z;

            rolls = obj.imu.rolls;
            pitches =  obj.imu.pitches;
            yaws =  obj.imu.yaws;

            t=  obj.imu.timestamps;

            % Ensure column vectors
            id_time = id_time(:);
            V_x_new = V_x_new(:);
            V_y_new = V_y_new(:);
            V_z_new = V_z_new(:);

            rolls = rolls(:);
            pitches = pitches(:);
            yaws = yaws(:);

            % t = obj.identification.timestamps;
            X_intg = obj.positionIntegrated.X_intg; % X position
            Y_intg = obj.positionIntegrated.Y_intg; % Y position

            Input = [obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z ...
                obj.CMD.cmd_angular_x obj.CMD.cmd_angular_y obj.CMD.cmd_angular_z];
            Output = [V_x_new V_y_new V_z_new rolls pitches yaws];


            % Input = [obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z];
            % Output = [V_x_new V_y_new V_z_new];

            % Input = [obj.CMD.cmd_linear_x*0  obj.CMD.cmd_linear_y*0 obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y];
            % Output = [X_intg Y_intg V_x_new V_y_new];

            dt = mean(diff(id_time));


            params0 = [APar BPar];


            size(params0)
            % Compute sampling time

            % Define input-output data for identification
            if size(Input,1) > size(Output,1) %the difference is usually only 1 
                Input = Input(1:size(Output,1),:);
                id_time = id_time(1:size(Output,1));

            elseif size(Input,1) < size(Output,1)
                Output = Output(1:size(Input,1),:);
            end
            data = iddata(Output, Input, dt);

            % Define grey-box model
            sys_grey = idgrey(@parametric_model2, params0, 'd', dt);

            opt = greyestOptions('InitialState', 'zero', 'Display', 'on');
            opt.SearchOptions.MaxIterations = maxItt;  % Set maximum number of iterations to 500
            % Estimate state-space model
            opt.SearchOptions.Tolerance = 1e-9;
            sys_ss = greyest(data, sys_grey, opt);

            % Display identified state-space model
            disp('Identified State-Space Model:');
            disp(sys_ss);

            % Extract state-space matrices
            [A, B, C, D] = ssdata(sys_ss);
            disp('Estimated State-Space Matrices:');
            disp('A = '), disp(A);
            disp('B = '), disp(B);
            disp('C = '), disp(C);
            disp('D = '), disp(D);

            % **Euler Angles Ground Truth (GT)**
            obj.identification = struct( ...
                'Input', Input, ...
                'Output', Output, ...
                'A', A, ...
                'B', B, ...
                'C', C, ...
                'D', D, ...
                'dt', dt, ...
                'sys_ss', sys_ss, ...
                'timestamps', id_time ...
            );


            disp('System identification completed successfully.');
        end


        function identifySystem3(obj,APar, BPar, maxItt)
            
            id_time = obj.CMD.cmd_timestamps;
            V_x_new = obj.NED.V_x;
            V_y_new =  obj.NED.V_y;
            V_z_new =  obj.NED.V_z;
        
            rolls = obj.imu.rolls;
            pitches =  obj.imu.pitches;
            yaws =  obj.imu.yaws;

            t=  obj.imu.timestamps;
        
            % Ensure column vectors
            id_time = id_time(:);
            V_x_new = V_x_new(:);
            V_y_new = V_y_new(:);
            V_z_new = V_z_new(:);
            
            rolls = rolls(:);
            pitches = pitches(:);
            yaws = yaws(:);

            % t = obj.identification.timestamps;
            X_intg = obj.positionIntegrated.X_intg; % X position
            Y_intg = obj.positionIntegrated.Y_intg; % Y position
            Z_intg = obj.positionIntegrated.Z_intg; % Y position

            % Input = [obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z ...
            %     obj.CMD.cmd_angular_x obj.CMD.cmd_angular_y obj.CMD.cmd_angular_z];
            % Output = [V_x_new V_y_new V_z_new rolls pitches yaws];


            % Input = [obj.CMD.cmd_linear_x  obj.CMD.cmd_linear_y obj.CMD.cmd_linear_z];
            % Output = [V_x_new V_y_new V_z_new];
            zeroInput = obj.CMD.cmd_linear_x*0 ;
            Input = [obj.thrusters.input.thruster0 obj.thrusters.input.thruster1 obj.thrusters.input.thruster2];
            Output = [X_intg Y_intg Z_intg V_x_new V_y_new V_z_new];
            
            % cutIdx = 0.4*length(Input);
            % Input = Input(1:cutIdx,:);
            % Output = Output(1:cutIdx,:);

            dt = mean(diff(id_time));
            APar = zeros(6,6);
            BPar = zeros(6,4);
            params0 = [APar BPar];
                        
          
            size(params0)
    
            % Define input-output data for identification
            if size(Input,1) > size(Output,1) %the difference is usually only 1 
                Input = Input(1:size(Output,1),:);
                id_time = id_time(1:size(Output,1));

            elseif size(Input,1) < size(Output,1)
                Output = Output(1:size(Input,1),:);
            end
            Ts = 1/dt;
            data = iddata(Output, Input, dt);


        % modelOrder = 6;  %
        % opt = ssestOptions('Display', 'on');
        % opt = ssestOptions('Display', 'on');
        % sys_ss = ssest(data, modelOrder, 'Ts', dt, opt,'DisturbanceModel', 'none');

        % 

            % Define grey-box model
            sys_grey = idgrey(@parametric_model3, params0, 'd', dt);

            opt = greyestOptions('InitialState', 'zero', 'Display', 'on');
            opt.SearchOptions.MaxIterations = 300;  % Set maximum number of iterations to 500
            % Estimate state-space model
            opt.SearchOptions.Tolerance = 1e-9;
            sys_ss = greyest(data, sys_grey, opt);





            % % Display identified state-space model
            % disp('Identified State-Space Model:');
            % disp(sys_ss);
            % 
            % Extract state-space matrices
            [A, B, C, D] = ssdata(sys_ss);
            % disp('Estimated State-Space Matrices:');
            % disp('A = '), disp(A);
            % disp('B = '), disp(B);
            % disp('C = '), disp(C);
            % disp('D = '), disp(D);

            % **Euler Angles Ground Truth (GT)**
            obj.identification = struct( ...
                'Input', Input, ...
                'Output', Output, ...
                'A', A, ...
                'B', B, ...
                'C', C, ...
                'D', D, ...
                'dt', dt, ...
                'sys_ss', sys_ss, ...
                'timestamps', id_time ...
            );

        figure;
        compare(data, sys_ss)
        [ymod,fit,ic] =compare(data, sys_ss);        
        % Compute the error
        error = Output - ymod.OutputData;
        disp(['fit: ', num2str(fit')]);
        disp(['Mean Absolute Error: ', num2str(mean(abs(error)))]);
        disp(['Root Mean Square Error (RMSE): ', num2str(sqrt(mean(error.^2)))]);

            % disp('System identification completed successfully.');
        end





        function [timeSignal1_new, signal1_new, signal2_new] = cut_downsample_data(obj, timeSignal1, timeSignal2, signal1, signal2)
            skip_ix = 1/obj.downSampleFreq;
        
            % Get max_t and min_t
            max_t = min(timeSignal1(end), timeSignal2(end));
            min_t = max(timeSignal1(1), timeSignal2(1));
        
            % Find the indices of timestamps_dvl and thruster2_in_t within the range [min_t, max_t]
            indicesDvl = (timeSignal1 <= max_t & timeSignal1 >= min_t);
            indicesThruster = (timeSignal2 <= max_t & timeSignal2 >= min_t);
        
            % Apply the indices to get the cut arrays
            timestamps_dvl_cut = timeSignal1(indicesDvl);
            thruster2_in_t_cut = timeSignal2(indicesThruster);
        
            % Assuming V_x and thruster2_in are your corresponding data arrays, you can do the same for them
            V_x_cut = signal1(indicesDvl);  % Cut V_x using the same indices as timestamps_dvl
            thruster2_in_cut = signal2(indicesThruster);  % Cut thruster2_in using the same indices as thruster2_in_t
        
            % Calculate the new sample rates for the cut arrays
            f_dvl_new = 1 / mean(diff(timestamps_dvl_cut));
            f_thruster_new = 1 / mean(diff(thruster2_in_t_cut));

            disp('f_dvl_new')
            disp(f_dvl_new)
            disp('f_thruster_new')
            disp(f_thruster_new)
            timeSignal1_new = timestamps_dvl_cut;
            signal1_new = V_x_cut;
            signal2_new = thruster2_in_cut;
        
            % % Define a new time vector with 1-second intervals
            % timeSignal1_new = min(timestamps_dvl_cut):skip_ix:max(timestamps_dvl_cut);
            % 
            % 
            % % Resample V_x_cut and thruster2_in_cut using linear interpolation
            % signal1_new = interp1(timestamps_dvl_cut, V_x_cut, timeSignal1_new, 'linear', 'extrap');
            % signal2_new = interp1(thruster2_in_t_cut, thruster2_in_cut, timeSignal1_new, 'linear', 'extrap');
        
            % The function returns the resampled data and the new sample rates
        end

        function [timeSignal1_new, signal1_new, signal2_new] = cut_downsample_sonar_data(obj, timeSignal1, timeSignal2, signal1, signal2)
        
            skip_ix = 1/obj.downSampleFreq;
            % Get max_t and min_t
            max_t = min(timeSignal1(end), timeSignal2(end));
            min_t = max(timeSignal1(1), timeSignal2(1));
        
            % Find the indices of timestamps_dvl and thruster2_in_t within the range [min_t, max_t]
            indicesDvl = (timeSignal1 <= max_t & timeSignal1 >= min_t);
            indicesThruster = (timeSignal2 <= max_t & timeSignal2 >= min_t);
        
            % Apply the indices to get the cut arrays
            timestamps_dvl_cut = timeSignal1(indicesDvl);
            thruster2_in_t_cut = timeSignal2(indicesThruster);
        
            % Assuming V_x and thruster2_in are your corresponding data arrays, you can do the same for them
            V_x_cut = signal1(indicesDvl,:);  % Cut V_x using the same indices as timestamps_dvl
            thruster2_in_cut = signal2(indicesThruster,:);  % Cut thruster2_in using the same indices as thruster2_in_t
        
            % Calculate the new sample rates for the cut arrays
            f_dvl_new = 1 / mean(diff(timestamps_dvl_cut));
            f_thruster_new = 1 / mean(diff(thruster2_in_t_cut));
        
            % Define a new time vector with 1-second intervals
            timeSignal1_new = min(timestamps_dvl_cut):skip_ix:max(timestamps_dvl_cut);
        
            % Resample timestamps_dvl_cut and thruster2_in_t_cut to 1 Hz
            timestamps_dvl_resampled = timeSignal1_new;  % Time vector is already defined as t_new
        
            % Resample V_x_cut and thruster2_in_cut using linear interpolation
            signal1_new = interp1(timestamps_dvl_cut, V_x_cut, timeSignal1_new, 'linear', 'extrap');
            signal2_new = interp1(thruster2_in_t_cut, thruster2_in_cut, timeSignal1_new, 'linear', 'extrap');
        
            % The function returns the resampled data and the new sample rates
        end
        function [common_time, interp_signal1, interp_signal2] = interpolate_signals2Input(obj, signal1_time, signal1, signal2_time, signal2)


            [signal1_time, unique_idx] = unique(signal1_time);
            signal1 = signal1(unique_idx);

            [signal2_time, unique_idx] = unique(signal2_time);
            signal2 = signal2(unique_idx);



            % Find the longest time vector
            all_times = {signal1_time, signal2_time};
            [~, longest_idx] = max(cellfun(@length, all_times));
            common_time = all_times{longest_idx};
            
            % Ensure the time vector is sorted
            common_time = sort(unique(common_time));
            
            % Interpolate each signal to match the common time vector
            interp_signal1 = interp1(signal1_time, signal1, common_time, 'linear', 'extrap');
            interp_signal2 = interp1(signal2_time, signal2, common_time, 'linear', 'extrap');
        end


    end
    methods (Access = private)
        function [trimmed_signal1_time, trimmed_signal1, trimmed_signal2_time, trimmed_signal2, trimmed_signal3_time, trimmed_signal3] = ...
                align_signals(obj, signal1_time, signal1, signal2_time, signal2, signal3_time, signal3)
            % Aligns three signals to start at the latest common start time and end at the earliest common end time.
            %
            % Inputs:
            %   signal1_time - Time array of first signal
            %   signal1      - Values corresponding to signal1_time
            %   signal2_time - Time array of second signal
            %   signal2      - Values corresponding to signal2_time
            %   signal3_time - Time array of third signal
            %   signal3      - Values corresponding to signal3_time
            %
            % Outputs:
            %   trimmed_signal1_time - Adjusted signal1_time within common time range
            %   trimmed_signal1      - Adjusted values corresponding to trimmed_signal1_time
            %   trimmed_signal2_time - Adjusted signal2_time within common time range
            %   trimmed_signal2      - Adjusted values corresponding to trimmed_signal2_time
            %   trimmed_signal3_time - Adjusted signal3_time within common time range
            %   trimmed_signal3      - Adjusted values corresponding to trimmed_signal3_time
        
            % Determine the latest common start time
            common_start_time = max([signal1_time(1), signal2_time(1), signal3_time(1)]);
        
            % Determine the earliest common end time
            common_end_time = min([signal1_time(end), signal2_time(end), signal3_time(end)]);
        
            % Find indices for start and end alignment
            idx1_start = find(signal1_time >= common_start_time, 1);
            idx2_start = find(signal2_time >= common_start_time, 1);
            idx3_start = find(signal3_time >= common_start_time, 1);
        
            idx1_end = find(signal1_time <= common_end_time, 1, 'last');
            idx2_end = find(signal2_time <= common_end_time, 1, 'last');
            idx3_end = find(signal3_time <= common_end_time, 1, 'last');
        
            trimmed_signal1_time    = signal1_time(idx1_start:idx1_end);
            trimmed_signal1         = signal1(idx1_start:idx1_end);
    
            trimmed_signal2_time = signal2_time(idx2_start:idx2_end);
            trimmed_signal2 = signal2(idx2_start:idx2_end);

            trimmed_signal3_time = signal3_time(idx3_start:idx3_end);
            trimmed_signal3 = signal3(idx3_start:idx3_end);


        end
        function [common_time, interp_signal1, interp_signal2, interp_signal3] = interpolate_signals(obj, signal1_time, signal1, signal2_time, signal2, signal3_time, signal3)
    
            % Find the longest time vector
            all_times = {signal1_time, signal2_time, signal3_time};
            [~, longest_idx] = max(cellfun(@length, all_times));
            common_time = all_times{longest_idx};
            
            % Ensure the time vector is sorted
            common_time = sort(unique(common_time));
            


            [signal1_time, unique_idx] = unique(signal1_time);
            signal1 = signal1(unique_idx);
            [signal2_time, unique_idx] = unique(signal2_time);
            signal2 = signal2(unique_idx);
            [signal3_time, unique_idx] = unique(signal3_time);
            signal3 = signal3(unique_idx);


            % Interpolate each signal to match the common time vector
            interp_signal1 = interp1(signal1_time, signal1, common_time, 'linear', 'extrap');
            interp_signal2 = interp1(signal2_time, signal2, common_time, 'linear', 'extrap');
            interp_signal3 = interp1(signal3_time, signal3, common_time, 'linear', 'extrap');
        end




    end

    
end
