%% Script Description
% This script takes in the Radar sensor readings along with the ego car global coordinates and calculates 
% the global x,y coordinates of the traffic cars then to be used to produce
% the .mat file to be fed into the CS-LSTM model to predict the trajectory of the target car 

close all; % close all the figures
clear;
% This section calculates the equation of the line the nose of the vehicle is pointing at a given time.
Speed = 10;
Dir = 'Right';
Sensor = 'Radar';
Road = 'US101';
Spd = num2str(Speed);

switch Dir
    case 'Right'
        folder = 'RLC';
    case 'Left'
        folder = 'LLC';
end    

switch Road
    case 'Straight'
        rd = 'StraightRd';
    case 'US101'
        rd = 'US101_original';
    case 'Curve'
        rd = 'Road Curve';
end

for a = 25:25:150
%     a = 50;
%     readPath = ['C:\Users\gamage_a\Documents\CM_Trail\SimOutput\WMGL241\',rd,'\Manoeuvre_Cut in_',Dir,'\',Sensor,'\Range\RelLonVel_',Spd,'kph\Scenario_',rd,'_',folder,'_4TS','_Range_',num2str(a),'.erg'];  
    readPath = ['C:\Users\gamage_a\Documents\CM_Trail\SimOutput\WMGL241\',rd,'\Manoeuvre_Cut in_',Dir,'\',Sensor,'\Range\RelLonVel_',Spd,'kph\Scenario_US101_LC_Right_Range_',num2str(a),'.erg'];
%     readPath = ['C:\Users\gamage_a\Documents\CM_Trail\SimOutput\WMGL241\',rd,'\Manoeuvre_Cut in_',Dir,'\',Sensor,'\Range\RelLonVel_',Spd,'kph\Scenario_',rd,'_',folder,'_2023_Range_',num2str(a),'.erg'];
    dFile = cmread(readPath);
    % Initial time-step removed to allow for radar model's cycle time
    ego_xFr1 = dFile.Car_Fr1_tx.data; % Ground truth x coordinate of car
    ego_yFr1 = dFile.Car_Fr1_ty.data; % Ground truth y coordinate of car
    Time = dFile.Time.data; % Initial time-step removed to alow for radar model's cycle time
    rdRefAng = asin(dFile.RdVector_y.data(2:end));
    yawAng = dFile.Car_Yaw.data; % Ground truth yaw angle of ego car
    yawAng = round(yawAng*100)/100; % Convert the yawangle to 2 decimal point representation
    radarPos = 4.2; % Position of the radar sensor on the vehicle
    ego_x = ego_xFr1 + radarPos.*cos(yawAng); % Ground truth x coordinate of radar position
    ego_y = ego_yFr1 + radarPos.*sin(yawAng); % Ground truth y coordinate of radar position      

    % Traffic car detection and ground truth data
    numTraffic = max(dFile.Sensor_Radar_Radar1_nObj.data); % Maximum number of objects detected by the radar sensor
    measuredData = cell(numTraffic,2); % Create a cell with 2 columns and a row for each object detected   
    tt = struct2cell(dFile); % Convert the dFile structure to a cell, to enable reading the strings 
     
    % This section extracts the required radar measurements and ground-truth data to subsequently calculate x, y coordinates 
    % of the traffic cars 
    for i = 1:numTraffic
        rad_Idx = ['Sensor.Radar.Radar1.Obj' num2str(i-1)]; % Auto-generate the common string elements for reading data 
        Trf_ID = [rad_Idx '.ObjId'];     
        meas_X = [rad_Idx '.DistX'];
        meas_Y = [rad_Idx '.DistY'];
        meas_Dist = [rad_Idx '.Dist'];
        Detect = [rad_Idx '.MeasStat'];     
        
       % Iterate through all the records to fill the cell with filtered data
       for q = 1:length(tt)
           % Fill the measuredData cell structure with the required data only.
           if convertCharsToStrings(tt{q}.name) == convertCharsToStrings(Trf_ID)
               objID = max(tt{q}.data-16000000);
               if and(objID>-1, objID<=numTraffic)
                   % Labels for reading ground truths
                   GT_traffic_X = ['Traffic.T0' num2str(objID) '.tx'];
                   GT_traffic_Y = ['Traffic.T0' num2str(objID) '.ty'];
                   laneID = ['Traffic.T0' num2str(objID) '.Lane.Act.LaneId'];
               end
               for k = 1:length(tt)
                   % Only consider data from the 2nd time frame to allow for missing data at initial 
                   % time frame due to cycle time of the Radar model.
                   if convertCharsToStrings(tt{k}.name) == convertCharsToStrings(GT_traffic_X)
                       measuredData{objID+1,2}(1,:) = tt{k}.data(2:end);                     
                   elseif convertCharsToStrings(tt{k}.name) == convertCharsToStrings(GT_traffic_Y)
                       measuredData{objID+1,2}(2,:) = tt{k}.data(2:end);               
                   elseif convertCharsToStrings(tt{k}.name) == convertCharsToStrings(laneID)
                       measuredData{objID+1,2}(3,:) = tt{k}.data(2:end);
                   elseif convertCharsToStrings(tt{k}.name) == convertCharsToStrings(meas_X)
                       measuredData{objID+1,1}(1,:) = tt{k}.data(2:end);
                   elseif convertCharsToStrings(tt{k}.name) == convertCharsToStrings(meas_Y)
                       measuredData{objID+1,1}(2,:) = tt{k}.data(2:end);
                   elseif convertCharsToStrings(tt{k}.name) == convertCharsToStrings(meas_Dist)
                       measuredData{objID+1,1}(3,:) = tt{k}.data(2:end);
                   elseif convertCharsToStrings(tt{k}.name) == convertCharsToStrings(Detect)
                       measuredData{objID+1,1}(4,:) = tt{k}.data(2:end);
                   end
               end
           end
       end
    end
    
    % Create a cell for the calculated x, y coordinates of the traffic cars 
    measurements = cell(numTraffic,2);

   for j = 1:numTraffic
       for i = 1:length(dFile.Car_Yaw.data)-1 % -1 is to remove the initial data point which is not captured by the radar
           if  ~isempty(measuredData{j,1})% Check if the vehicle is detected by the sensor
               if measuredData{j,1}(4,i) == 3 % checks if detected by radar at a given time step
                   if yawAng(i) == 0 % if the ego vehicle trajectory is straight inline with the road center line (since road center line is set to the x-axis of the earth's fixed system).
                       measured_x = ego_x(i) + measuredData{j,1}(1,i);
                       measured_y = ego_y(i) + measuredData{j,1}(2,i); % assuming the measurements to the right of the line are negetive
                   else
                       theta = atan(measuredData{j,1}(2,i)/ measuredData{j,1}(1,i)); % Angle between the distance measurement line and the direction the vehicle is facing
                       Dist = sqrt(measuredData{j,1}(1,i)^2 + measuredData{j,1}(2,i)^2);
                       alpha = yawAng(i) + theta;
                       del_x = cos(alpha)* Dist;
                       del_y = sin(alpha)* Dist;
                       measured_x = ego_x(i) + del_x;
                       measured_y = ego_y(i) + del_y;
                   end
               else
                   measured_x = 0;
                   measured_y = 0;
               end
           end           
           measurements{j,1} = [measurements{j,1}, measured_x]; % Append the newly calculated x, y coordinates to the measurements cell structure
           measurements{j,2} = [measurements{j,2}, measured_y];
       end

    %% Differences between the ground-truth coordinates Vs the calculated coordinates based on radar measurements of the target car
    if  ~isempty(measuredData{j,1})
        index = (measuredData{j,1}(4,:) == 3);    
        calcCoordinate_x = measurements{j,1}(index);
        gTruth_x = measuredData{j,2}(1,:);
        gTruth_x_filtered = gTruth_x(index);
        diff_x = gTruth_x_filtered - calcCoordinate_x;
        
        compare_x = [gTruth_x_filtered', calcCoordinate_x', (gTruth_x_filtered - calcCoordinate_x)'];
        
        calcCoordinate_y = measurements{j,2}(index);
        gTruth_y = measuredData{j,2}(2,:);
        gTruth_y_filtered= gTruth_y(index);
        diff_y = gTruth_y_filtered - calcCoordinate_y;
        
        compare_y = [gTruth_y_filtered', calcCoordinate_y', (gTruth_y_filtered - calcCoordinate_y)'];
        
        %% Plotting the X, Y co-ordinates from measurement-based Vs ground truths
        % close all;
        % Create a figure
        SurID = num2str(j);
        Title = ['Ground Truths Vs ',Sensor,' Measurements at Range ', num2str(a),'m for Surround Car_', SurID,' ',rd,'220323'];
        figMetrics = figure('Name',Title);
        % This is the maximum figure width that can be used for publishing without clipping the subplots
        maxWidth = 1150;
        pos = figMetrics.Position;
        width = pos(3);
        figMetrics.Position = [pos(1)-(maxWidth-width)/2 pos(2) maxWidth pos(4)];
        
        % plot the radar measurements-based x coordinates
        xCord_radar = subplot(1,2,1);
        plot(calcCoordinate_x,'.');
        hold on
        plot(gTruth_x_filtered, '.');
        hold off
        
        ylabel(xCord_radar,'x coordinate');
        xlabel(xCord_radar,'Point');
        title(xCord_radar,'Radar-based Vs Ground-truths for X coordinates');
        grid(xCord_radar,'on');
        legend ('Radar detections','GroundTruth','Location','northwest');
        
        % plot the radar measurements-based y coordinates
        yCord_radar = subplot(1,2,2);
        plot(calcCoordinate_y,'.');
        hold on
        plot(gTruth_y_filtered, '.');
        hold off
        ylabel(yCord_radar,'y coordinate');
        xlabel(yCord_radar,'Point');
        title(yCord_radar,'Radar-based Vs Groundtruths for Y coordinates');
        grid(yCord_radar,'on');
        legend ('Radar detections','GroundTruth','Location','northeast');
        
        % Define path for saving the figures
        FolderName = ['C:\Users\gamage_a\Documents\Python\conv-social-pooling-master\',rd,'\radarData\manoeuvre_',folder,'\Range\relVel_',Spd,'kph\graphs'];
        %[~, file]  = fileparts(filename);  % Remove extension
        saveas(gca, fullfile(FolderName, [Title, '.png']) ) % Append
        close all;
    end
  end

    %% Script for generating the traj data file for the CS pooling model using CM generated data from the Radar measurements.
   
    % Read the data from the saved data file from CM   
%     GT = 0; % a key to differentiate between groundtruth data Vs vision sensor based data
    surCars = [];
        for t = 1:numTraffic
            if  ~isempty(measuredData{t,1})% Check if the vehicle is detected
                trajTr = [10*Time(2:end)', measurements{t,2}', measurements{t,1}', measuredData{t,2}(3,:)'];% [time, local x, local y, lane id]correspond to the Target car (always given traffic name 'T00' in CM)
                vehID = ones(length(trajTr),1)*t+15; % Generate the global vehicle IDs as in CM by multiplying the ones array by 16(In CM:16000000)
                trajTr = single([vehID, trajTr]); %  Form the first 6 fields of the matrix and convert to single precision
                [~,ia,~] = unique(measuredData{t,2}(1:3,2:end)','rows', 'stable'); % Remove the time frames matching the static poisitional data records from groundtruth data records
                trajTr = trajTr(ia,:);
                if t==1 % This is the target car
                    trajTar = [trajTr, rdRefAng(ia)']; % Concatenate the road reference angle
                    non0start = find(trajTr(:,3) ~= 0, 1, 'first'); % index of first non-zero data recording
                    non0end = find(trajTr(:,3) ~= 0, 1, 'last'); % index of first non-zero data recording
                    trajtar = trajTr(non0start:non0end,:); % Time window the sensor had captured the target car's trajectory
                else
                surCars = [surCars;trajTr]; % For surround cars, impact of occlusions are accounted for since the no data points are not removed
                end
            end
        end
        
    % Create an array of all the trajectory data of the ego vehicle
    trajEgo = [ones(length(Time'),1)*15,10*Time', ego_yFr1', ego_xFr1', dFile.Car_Road_Lane_Act_LaneId.data']; % [vehID, time, local x, local y, landID]
    trajEgo = trajEgo(2:end,:);% Remove the first timeframe to align with the sensor-based dataset
    [~,ia,~] = unique(trajEgo(:,3:4),'rows', 'stable'); % Remove the static poisition data recording at the end of the scenario
    trajEgo = single(trajEgo(ia,:)); % Use 'ia' to index into and retrieve the rows that have unique combinations of elements in the 4th and 5th columns.
                                     % and convert the values into single precision.
    trajSur = [surCars;trajEgo];
    trajAll = [trajSur;trajtar];
    
    [traj, tracks] = CMinputs(trajTar, trajSur, trajAll);
    
    %% Save mat files:
    %disp('Saving mat files...')
    savePath = ['C:\Users\gamage_a\Documents\Python\conv-social-pooling-master\',rd,'\radarData\manoeuvre_',folder,'\Range\relVel_',Spd,'kph\matFiles\',rd,'_Range_',num2str(a),'_220323'];
    save(savePath,'traj','tracks')
end 

%% Script for generating the traj data file for the CS pooling model using CM based ground-truth data
% GT = 1;
surCarsGT = [];
for t = 1:numTraffic
    trajTrGT = [10*Time(2:end)', measuredData{t,2}(2,:)', measuredData{t,2}(1,:)', measuredData{t,2}(3,:)'];% correspond to the Target car (always given traffic name 'T00' in CM)
    vehID = ones(length(trajTrGT),1)*t+15; % Generate the global vehicle IDs as in CM by multiplying the ones array by 16(In CM:16000000)
    trajTrGT = single([vehID, trajTrGT]); %  Format the first 6 fields of the matrix and convert to single precision
    [~,ia,~] = unique(trajTrGT(:,3:4),'rows', 'stable'); % Remove the static poisition data recording at the end of the scenario
    trajTrGT = trajTrGT(ia,:);
    if t==1       
        trajTarGT = [trajTrGT, rdRefAng(ia)'];
    else
        surCarsGT = [surCarsGT;trajTrGT];
    end
end

trajSurGT = [surCarsGT; trajEgo]; % Combine the surround vehicles together
trajAllGT = [trajSurGT; trajTarGT(:,1:end-1)];

[traj, tracks] = CMinputs(trajTarGT, trajSurGT, trajAllGT);

%% Save mat files:
%disp('Saving mat files...')
savePath = ['C:\Users\gamage_a\Documents\Python\conv-social-pooling-master\',rd,'\radarData\manoeuvre_',folder,'\Range\relVel_',Spd,'kph\matFiles\',rd,'_groundTruth_220323'];
save(savePath,'traj','tracks');  

function [traj, tracks] = CMinputs(trajCM, trajTr, trajAll)

% Modified N.Deo's code to determine the lateral and longitudinal
% maneuvers: Same approach followed as implemented when using the NGSIM data.
for k = 1:length(trajCM(:,1))        
        time = trajCM(k,2);  
        lane = trajCM(k,5);
        ind = find(trajCM(:,2)==time); % Find the index of the vehicle trajectory where the frame ID (column 3) matches the 'time'.I.e. find the index of 
        % the exact track from the list of tracks for a vehicle: vehid
        % find(): Find indices of nonzero elements        
        
        % Get lateral maneuver:
        ub = min(size(trajCM,1),ind+40);% Upper bound is calculated by checking whether the index at each record has 40 
                                         %(0.5*8sec duration used for observation and prediction) more Frame_IDs to the future and taking the lowest timeFrame
                                         % size(X,1) returns the number of rows of X and size(X,[1 2]) returns a row vector containing the number of rows & columns.
        lb = max(1, ind-40);% Lower bound is calculated by checking whether the index at each record has 40 more Frame_IDs to the past and taking 
                                        % the highest Frame_ID
        if trajCM(ub,5)>trajCM(ind,5) || trajCM(ind,5)>trajCM(lb,5)% future lane Id > current lane Id OR current lane Id > past lane Id 
            trajCM(k,7) = 3;% Categorise as 'Left Lane-change' and adds it to a new column_7
        elseif trajCM(ub,5)<trajCM(ind,5) || trajCM(ind,5)<trajCM(lb,5)
            trajCM(k,7) = 2;% Right Lane-change
        else
            trajCM(k,7) = 1;% Lane Keep
        end        
        
        % Get longitudinal maneuver:
        ub = min(size(trajCM,1),ind+50);% Upper bound is calculated by checking whether the index at each record has 50 more frames(5s to the future)to 
                                         % the future and taking the lowest duration
        lb = max(1, ind-30);% Lower bound is calculated by checking whether the index at each record has 30 frames to the past or checking if at the start 
                            % of the recording
        if ub==ind || lb ==ind || trajCM(ub,4)==0 ||trajCM(lb,4)==0 % If current index is the start OR the end of the recording there's no longitudinal 
                                                                    % reading (no measurement from the sensor) available....
            trajCM(k,8) = 1;% longitudinal maneuver is categorised as 'Normal speed' and adds it to a new column_8
        else
            vHist = (trajCM(ind,4)-trajCM(lb,4))/(ind-lb);% Historical velocity calculated by dividing the longitudinal distance between 
                                                            % current and lower bound time frames
            vFut = (trajCM(ub,4)-trajCM(ind,4))/(ub-ind);% Future velocity calculated by dividing the longitudinal distance between 
                                                           % current and lower bound time frames
            if vFut/vHist < 0.8% vehicle to be performing a braking maneuver if it’s average speed over the prediction horizon is less 
                % than 0.8 times its speed at the time of the prediction
                trajCM(k,8) = 2;% Braking and adds it to a new column_8
            else
                trajCM(k,8) = 1;
            end
        end
        
        % Get grid locations:
        
        frameEgo = trajTr(and((trajTr(:,2) == time),(trajTr(:,5) == lane)),:);
        frameL = trajTr(and((trajTr(:,2) == time),(trajTr(:,5) == lane-1)),:);
        frameR = trajTr(and((trajTr(:,2) == time),(trajTr(:,5) == lane+1)),:);
        if ~isempty(frameL)
            for l = 1:size(frameL,1)
                y = frameL(l,4)-trajCM(k,4);
                if abs(y) < 27.432 % 90ft distance boundary
                    gridInd = 1+round((y+27.432)/4.572); % Filling the first column of the 13x3 spatial grid
                    trajCM(k,8+gridInd) = frameL(l,1);
                end
            end
        end
        for l = 1:size(frameEgo,1)
            y = frameEgo(l,4)- trajCM(k,4);
            if abs(y) < 27.432 && y~=0
                gridInd = 14+round((y+27.432)/4.572);% 14 come from 90ft/15ft to the front and back of the ego car +1 (13) allocated to the frameL earlier.
                trajCM(k,8+gridInd) = frameEgo(l,1);
            end
        end
        if ~isempty(frameR)
            for l = 1:size(frameR,1)
                y = frameR(l,4)-trajCM(k,4);
                if abs(y) < 27.432
                    gridInd = 27+round((y+27.432)/4.572);% 27 comes from (14 + 13) as above.
                    trajCM(k,8+gridInd) = frameR(l,1);
                end
            end
        end       
end

 tracksCM = {};
    %trajSet = trajTr(trajTr(:,1)==k,:); % I would question at this stage, why trajAll was created at the first place? Code would have been much 
                                        % simpler without. Here, it is seperating the Training trajectory set by the Dataset ID
    carIds = unique(trajAll(:,1)); % Create an array containing the unique vehicleIDs
    for l = 1:length(carIds)
        vehtrack = trajAll(trajAll(:,1) == carIds(l),2:4)'; % ***Need to take the transpose for my research. 
                                                           % Iterate over the unique vehicleIDs and get the frameID, localX and localY and 
                                                           % transpose the matrix to a 3 by length of frame numbers captured
        tracksCM{1,carIds(l)} = vehtrack; % create a cell with references; DatasetID as the row and the vehicle ID as the column 
    end
    
% % Remove only the leading and lagging zeros to the sensor captured data for the target car
% non0start = find(trajCM(:,3) ~= 0, 1, 'first'); % index of first non-zero data recording
% non0end = find(trajCM(:,3) ~= 0, 1, 'last'); % index of first non-zero data recording
% trajCM = trajCM(non0start:non0end,:); % Limit the predictions on the time window the sensor had captured the target car's trajectory including occlusions        

% Remove time steps with no data for x, y coordinates 
idx = find(trajCM(:,4));
trajCM = trajCM(idx,:);    
    
%% Filter edge cases: 
% Since the model uses 3 sec of trajectory history for prediction, the initial 3 seconds of each trajectory is not used for training/testing
%disp('Filtering edge cases...')
    indsCM = zeros(size(trajCM,1),1);
    for k = 1: size(trajCM,1)
        t = trajCM(k,2);
        if tracksCM{1,trajCM(k,1)}(1,31) <= t && tracksCM{1,trajCM(k,1)}(1,end)>t+1
            indsCM(k) = 1;
        end
    end
    trajCM = trajCM(find(indsCM),:);%  find(): Find indices of nonzero elements
    
% Add dataset Id column
traj = [ones(size(trajCM,1),1),trajCM];
tracks = tracksCM;
end



