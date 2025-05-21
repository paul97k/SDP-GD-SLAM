classdef classKalman < handle
    properties
        fullPath

        DataObjGlobal


        identifiedSystem

        EKF
        downSampleFreq
        map

        u
        timeId
        z_meas_val
        
        x_estimates


        noise

    end
    
    methods
        % Constructor to load the data
        function obj = classKalman(objData ,identification)
            obj.DataObjGlobal = objData;
            obj.identifiedSystem = identification;
        end



        function setInputAndMeasurement(obj)
            DataObj = obj.DataObjGlobal;

            id_time = DataObj.CMD.cmd_timestamps;
            V_x_new = DataObj.NED.V_x;
            V_y_new =  DataObj.NED.V_y;
            V_z_new =  DataObj.NED.V_z;
            
            rolls = DataObj.imu.rolls;
            pitches =  DataObj.imu.pitches;
            yaws =  DataObj.imu.yaws;
            
            % Ensure column vectors
            id_time = id_time(:);
            V_x_new = V_x_new(:);
            V_y_new = V_y_new(:);
            V_z_new = V_z_new(:);
            
            
            % 
            zeroInput = DataObj.CMD.cmd_linear_x*0 ;
            Input = [zeroInput zeroInput zeroInput  ...
                     DataObj.CMD.cmd_linear_x  DataObj.CMD.cmd_linear_y DataObj.CMD.cmd_linear_z];
            
            Output = [DataObj.positionIntegrated.X_intg DataObj.positionIntegrated.Y_intg DataObj.positionIntegrated.Z_intg];
            time = id_time;



            if size(Input,1) > size(Output,1)
                Input = Input(1:size(Output,1),:);
                time = time(1:size(Output,1));
            elseif size(Input,1) < size(Output,1)
                Output = Output(1:size(Input,1),:);
            end
            
            obj.timeId = time;
            obj.u = Input;
            obj.z_meas_val = Output;    
        end


        function PerformEKFOriginal(obj, u, z_meas, paramaters)
            P = paramaters.P;
            Q = paramaters.Q;
            R = paramaters.R;
            x_0 = paramaters.x_0;

            DataObj = obj.DataObjGlobal;
            
            dt = obj.identifiedSystem.dt;
            A = obj.identifiedSystem.A;
            B = obj.identifiedSystem.B;

            A1 = [eye(3)  eye(3)*dt zeros(3)];
            A2 = [zeros(size(A,2),3) A];
            F = [A1;A2];

            G =  [zeros(3, size(B,2)) ;B];
            
            H = [zeros(size(A,1),3) eye(size(A,1))];

            num_steps = length(u); % Number of simulation steps


            x_estimatesLoc = zeros(size(A,1) +3, num_steps);
            x_est = x_0;
            % EKF Loop
            for k = 2:num_steps
                
                x_pred = F * x_est + G* u(k,:)'; % Predicted state
                P_pred = F * P * F' + Q; % Predicted covariance
            
                % Update Step (DVL Measurement)
                z_k = z_meas(k,:)';
                
            
                % Compute Kalman Gain
                K = P_pred * H' / (H * P_pred * H' + R);
                
                % Update State and Covariance
                % disp(["Error=", num2str(mean(z_DVL - H * x_pred))])
                x_est = x_pred + K * (z_k - H * x_pred);
                % P = (eye(4) - K * H) * P_pred*(eye(4) - K * H)' + K*R*K';
                P = (eye(size(P)) - K * H) * P_pred*(eye(size(P)) - K * H)' + K*R*K';
            
                % Store for plotting
                x_estimatesLoc(:, k) = x_est;
            end
            
            obj.x_estimates = x_estimatesLoc;

                % obj.EKF = struct( ...
                %     'x_estimates', x_estimates ...
                % );
            
        end

        

        function PerformEKF(obj, u, z_meas, paramaters)
            P = paramaters.P;
            Q = paramaters.Q;
            R = paramaters.R;
            x_0 = paramaters.x_0;

            DataObj = obj.DataObjGlobal;
            
            dt = obj.identifiedSystem.dt;
            A = obj.identifiedSystem.A;
            B = obj.identifiedSystem.B;

            % A1 = [eye(3)  eye(3)*dt zeros(3)];
            % A2 = [zeros(size(A,2),3) A];
            F = A;

            G =  B;
            
            % H = [zeros(size(A,1),3) eye(size(A,1))];
            H = [eye(3, size(A,1))];

            num_steps = length(u); % Number of simulation steps


            x_estimatesLoc = zeros(size(A,1) , num_steps);
            x_est = x_0;
            % EKF Loop
            for k = 2:num_steps
                
                x_pred = F * x_est + G* u(k,:)'; % Predicted state
                P_pred = F * P * F' + Q; % Predicted covariance
            
                % Update Step (DVL Measurement)
                z_k = z_meas(k,:)';
                
            
                % Compute Kalman Gain
                K = P_pred * H' / (H * P_pred * H' + R);
                
                % Update State and Covariance
                % disp(["Error=", num2str(mean(z_DVL - H * x_pred))])
                x_est = x_pred + K * (z_k - H * x_pred);
                % P = (eye(4) - K * H) * P_pred*(eye(4) - K * H)' + K*R*K';
                P = (eye(size(P)) - K * H) * P_pred*(eye(size(P)) - K * H)' + K*R*K';
            
                % Store for plotting
                x_estimatesLoc(:, k) = x_est;
            end
            
            obj.x_estimates = x_estimatesLoc;

                % obj.EKF = struct( ...
                %     'x_estimates', x_estimates ...
                % );
            
        end

        function mapReturn = CreateKalmanMAP(obj, x_estimates, skip_i, skip_j)
            % Function to generate a 3D sonar map for ICP using Kalman filter estimates
            %
            % Parameters:
            % - obj: The object containing sonar and ground truth data
            % - x_estimates: Kalman filter estimated positions [2 x N]
            % - skip_i: Step size for time indexing (to reduce processing)
            % - skip_j: Step size for sonar indexing (to reduce noise)
            %
            % Returns:
            % - map: A 3×M matrix containing the generated 3D points for ICP
        

            DataObj = obj.DataObjGlobal;

            % Extract sonar positions
            time_sonar = DataObj.sonarPositions.timestamps;
            % mean(diff(time_sonar))
            y_sonar = DataObj.sonarPositions.y_positions;
            z_sonar = DataObj.sonarPositions.z_positions;
            
            % Initialize the map storage
            obj.map = [];
            
            iFactor = size(y_sonar,1)/length(x_estimates);
            if (1-abs(iFactor))<0.2
                iFactor=1;
            end
            middleIdx = round(size(z_sonar,2)/2);
            for i = 1:size(y_sonar,1)
                % i/iFactor
                xEst_i = max([1,round(i/iFactor)]);
                % Select every `skip_j`-th sonar point
                y_subset = y_sonar(i, middleIdx);
                z_subset = z_sonar(i, middleIdx);
                
                % Compute 3D points in world coordinates
                % x_values = x_estimates(1, xEst_i) * ones(size(y_subset));
                x_values = x_estimates(1, xEst_i) ;
                y_values = y_subset + x_estimates(2, xEst_i);
                z_values = z_subset;
                
                % Append new points to the map
                obj.map = [obj.map, [x_values; y_values; z_values]];
            end
            mapReturn = obj.map;
        end


        function [timeGT,timeLoc,...
                  map_pointsGT, ...
                  vxGT, vyGT, ...
                  X_intg, Y_intg, ...
                  xGT, yGT,  Z_AUV,...
                  Z_est, Z_nearest, Z_GT] = CutAndGetNearestDataFromMap(obj, skip_i, skip_j, x_estimates, map_points)

            endIdx= length(x_estimates);

            DataObj = obj.DataObjGlobal;
            x_estimatesLoc =x_estimates;
            timeLoc = obj.timeId(1:endIdx);
            

            X_intg = DataObj.positionIntegrated.X_intg(1:endIdx,:); 
            Y_intg = DataObj.positionIntegrated.Y_intg(1:endIdx,:);
            
            timeGT = DataObj.positionGT.timestamps;
            xGT = DataObj.positionGT.x_positions;
            yGT = DataObj.positionGT.y_positions;
            vxGT = DataObj.velocityGT.x_vel;
            vyGT = DataObj.velocityGT.y_vel;

                % -----------------------------------------------------------------    
                % -------------------create Map for estimated X and Ground truth-----------------------
                % -----------------------------------------------------------------      

            x_estimatesGTXY = zeros(9,length(xGT)); % the other 7 are just so that the function can work properly
            x_estimatesGTXY(1,:) =xGT';
            x_estimatesGTXY(2,:) =yGT';
            map_pointsGT  = obj.CreateKalmanMAP(x_estimatesGTXY, skip_i, skip_j);

            Z_est =     obj.findClosestZ(x_estimatesLoc(1,:),x_estimatesLoc(2,:), map_points);
            Z_nearest = obj.findClosestZ(X_intg             ,Y_intg             , map_points);
            Z_GT =      obj.findClosestZ(xGT                ,yGT                , map_pointsGT);
            
            Z_AUV = 0;



        end

        function plotResults(obj, ...
                timeGT,timeLoc,...
                z_meas_valLoc, x_estimatesLoc,  ...
                  map_points, map_pointsGT, ...
                  vxGT, vyGT, ...
                  X_intg, Y_intg, ...
                  xGT, yGT,  Z_AUV,...
                  Z_est, Z_nearest, Z_GT)


            plotSpecs = false;
            if plotSpecs
                figure;
                % set(gcf, 'Position', [0, 0, 2000, 1000]); % (x, y, width, height)
                % plot position
                    subplot(3,2,1);
                    plot(timeGT, xGT, 'b', 'LineWidth', 1.5); hold on;
                    plot(timeLoc, X_intg, 'g', 'LineWidth', 1.5); hold on;
                    plot(timeLoc, x_estimatesLoc(1,:), 'r--', 'LineWidth', 1.5);
                    xlabel('Time Step(k*dt)'); ylabel('X position');
                    legend('True Position','inegrate Position', 'EKF Estimate');
                    title('Position Estimation in X');
                    
                    subplot(3,2,2);
                    plot(timeGT, yGT, 'b', 'LineWidth', 1.5); hold on;
                    plot(timeLoc, Y_intg, 'g', 'LineWidth', 1.5); hold on;
                    plot(timeLoc, x_estimatesLoc(2,:), 'r--', 'LineWidth', 1.5);
                    xlabel('Time Step(k*dt)'); ylabel('Y position');
                    legend('True Position','inegrate Position', 'EKF Estimate');
                    title('Position Estimation in Y');
                
                % plot error
                    subplot(3,2,3);
                    plot(timeGT, abs(xGT-X_intg), 'g', 'LineWidth', 1.5); hold on;
                    plot(timeGT, abs(xGT-x_estimatesLoc(1,:)') , 'r--', 'LineWidth', 1.5);
                    xlabel('|Error Step|'); ylabel('X position');
                    legend('Error inegrate Position', 'Error  EKF Estimate');
                    title('Absolute Error in X');
                    
                    subplot(3,2,4);
                    plot(timeGT, abs(yGT-Y_intg), 'g', 'LineWidth', 1.5); hold on;
                    plot(timeGT, abs(yGT-x_estimatesLoc(2,:)'), 'r--', 'LineWidth', 1.5);
                    xlabel('Time Step(k*dt)'); ylabel('Y position');
                    legend('Error inegrate Position', 'Error EKF Estimate');
                    title('Absolute  Error in Y');
                
                % plot velocity  
                    subplot(3,2,5);
                    plot(timeGT, vxGT, 'b', 'LineWidth', 1.5); hold on;
                    plot(timeGT, z_meas_valLoc(:,1), 'g', 'LineWidth', 1.5); hold on;
                    plot(timeGT, x_estimatesLoc(5,:), 'r--', 'LineWidth', 1.5);
                    xlabel('Time Step(k*dt)'); ylabel('X velocity');
                    legend('GT Velocity', 'meas Velocity','EKF Estimate');
                    title('Velocity Estimation in X');
                    % xlim([50,100])
                    
                    subplot(3,2,6);
                    plot(timeGT, vyGT, 'b', 'LineWidth', 1.5); hold on;
                    plot(timeGT, z_meas_valLoc(:,2), 'g', 'LineWidth', 1.5); hold on;
                    plot(timeGT, x_estimatesLoc(6,:), 'r--', 'LineWidth', 1.5);
                    xlabel('Time Step(k*dt)'); ylabel('Y velocity');
                    legend('GT Velocity', 'meas Velocity', 'EKF Estimate');
                    title('Velocity Estimation in Y');
                    % xlim([50,100])

            end



                % 
                % 
                % 
                % % -----------------------------------------------------------------    
                % % -------------------create Map for estimated X and Ground truth-----------------------
                % % -----------------------------------------------------------------      
                % 
                % 
                % x_estimatesGTXY = zeros(9,length(xGT)); % the other 7 are just so that the function can work properly
                % x_estimatesGTXY(1,:) =xGT';
                % x_estimatesGTXY(2,:) =yGT';
                % 
                % map_points  = obj.CreateKalmanMAP(x_estimatesLoc, skip_i, skip_j);
                % map_pointsGT  = obj.CreateKalmanMAP(x_estimatesGTXY, skip_i, skip_j);
                % 
                % Z_est =     obj.findClosestZ(x_estimatesLoc(1,:)   ,x_estimatesLoc(2,:)   , map_points);
                % Z_nearest = obj.findClosestZ(X_intg             , Y_intg            , map_points);
                % Z_GT =      obj.findClosestZ(xGT                ,yGT                , map_pointsGT);
                % 
                % Z_AUV = 0;
                
                % Plot the results
                figure;
                scatter3(map_points(1,:), map_points(2,:), map_points(3,:), 5, 'b', 'filled'); 
                
                % scatter3(map_pointsGT(1,:), map_pointssGT(2,:), map_pointsGT(3,:), 5, 'r', 'filled'); 


                
                scatter3(x_estimatesLoc(1,:),x_estimatesLoc(2,:), Z_est, 20, 'y', 'filled'); % Red points for matched Y, Z
                hold on;
                scatter3(x_estimatesLoc(1,:),x_estimatesLoc(2,:), zeros(size(x_estimatesLoc(2,:))), 20, 'y', 'filled'); % Red points for matched Y, Z

                scatter3(X_intg, Y_intg, Z_nearest, 20, 'g', 'filled'); % Red points for matched Y, Z
                scatter3(X_intg, Y_intg,  zeros(size(Y_intg)), 20, 'g', 'filled'); % Red points for matched Y, Z

                scatter3(xGT, yGT, Z_GT, 20, 'r', 'filled'); % Red points for matched Y, Z
                scatter3(xGT, yGT, zeros(size(yGT)), 20, 'r', 'filled'); %
                % Red points for matched Y, Z
                xlabel('X'); ylabel('Y'); zlabel('Z');
                title('Closest Y and Z points from Map');
                grid on;
                % legend('Map Points', 'Ground truth Map Points', 'Postion using EKF', '.', 'Postion using integration', '.', 'Ground truth Postion', '.');
                legend( 'Postion using EKF', '.', 'Postion using integration', '.', 'Ground truth Postion', '.');

        end



        function [Z_closest] = findClosestZ(obj, X_intg, Y_intg, map_points)
            % Extract X, Y, Z from map
            X_map = map_points(1, :)';
            Y_map = map_points(2, :)';
            Z_map = map_points(3, :)';
            
            % Find nearest neighbor indices
            idx = knnsearch([X_map, Y_map], [X_intg(:), Y_intg(:)]);
        
            % Retrieve corresponding Z values
            Z_closest = Z_map(idx);
        end


        
    end
end
