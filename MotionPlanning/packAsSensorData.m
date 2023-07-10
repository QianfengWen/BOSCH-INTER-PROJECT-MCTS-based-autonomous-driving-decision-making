function sensorData = packAsSensorData(ptCloud, configs, time)
% Pack the sensor data as format required by the tracker
%
% ptCloud - cell array of pointCloud object
% configs - cell array of sensor configurations
% time    - Current simulation time

%The lidar simulation returns outputs as pointCloud objects. The Location
%property of the point cloud is used to extract x,y, and z locations of
%returns and pack them as structures with information required by a tracker.
sensorData = struct('SensorIndex',{},...
    'Time', {},...
    'Measurement', {},...
    'MeasurementParameters', {});

for i = 1:numel(ptCloud)
    % This sensor's point cloud
    thisPtCloud = ptCloud{i};
    
    % Allows mapping between data and configurations without forcing an
    % ordered input and requiring configuration input for static sensors.
    sensorData(i).SensorIndex = configs{i}.SensorIndex;
    
    % Current time
    sensorData(i).Time = time;
    
    % Exctract Measurement as a 3-by-N defining locations of points
    sensorData(i).Measurement = reshape(thisPtCloud.Location,[],3)';
    
    % Data is reported in the sensor coordinate frame and hence measurement
    % parameters are same as sensor transform parameters.
    sensorData(i).MeasurementParameters = configs{i}.SensorTransformParameters;
end

end

