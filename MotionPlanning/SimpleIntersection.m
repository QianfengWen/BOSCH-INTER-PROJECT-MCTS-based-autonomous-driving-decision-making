function [scenario, egoVehicle, sensors] = SimpleIntersection()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 12-Jun-2023 11:00:51

% Construct a drivingScenario object.
[scenario, egoVehicle] = createDrivingScenario;

sensors = createSensors(scenario);

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = lidarPointCloudGenerator('SensorIndex', 1, ...
    'SensorLocation', [3.7 0], ...
    'MaxRange', 100, ...
    'DetectionCoordinates', 'Sensor Cartesian', ...
    'HasOrganizedOutput', false, ...
    'HasEgoVehicle', false, ...
    'HasRoadsInputPort', false, ...
    'AzimuthLimits', [-45 45], ...
    'ElevationLimits', [-20 20], ...
    'ActorProfiles', profiles);

sensors{2} = lidarPointCloudGenerator('SensorIndex', 2, ...
    'SensorLocation', [2.8 0.9], ...
    'Yaw', 75, ...
    'MaxRange', 100, ...
    'DetectionCoordinates', 'Sensor Cartesian', ...
    'HasOrganizedOutput', false, ...
    'HasEgoVehicle', false, ...
    'HasRoadsInputPort', false, ...
    'AzimuthLimits', [-45 45], ...
    'ElevationLimits',[-20 20], ...
    'ActorProfiles', profiles);

sensors{3} = lidarPointCloudGenerator('SensorIndex', 3, ...
    'SensorLocation', [2.8 0.9], ...
    'Yaw', 135, ...
    'MaxRange', 100, ...
    'DetectionCoordinates', 'Sensor Cartesian',...
    'HasOrganizedOutput', false, ...
    'HasEgoVehicle', false, ...
    'HasRoadsInputPort', false, ...
    'AzimuthLimits', [-45 45], ...
    'ElevationLimits',[-20 20],...
    'ActorProfiles', profiles);

sensors{4} = lidarPointCloudGenerator('SensorIndex', 4, ...
    'SensorLocation', [-1 0], ...
    'Yaw', 180, ...
    'MaxRange', 100, ...
    'DetectionCoordinates', 'Sensor Cartesian', ...
    'HasOrganizedOutput', false, ...
    'HasEgoVehicle', false, ...
    'HasRoadsInputPort', false, ...
    'AzimuthLimits', [-45 45], ...
    'ElevationLimits',[-20 20],...
    'ActorProfiles', profiles);

sensors{5} = lidarPointCloudGenerator('SensorIndex', 5, ...
    'SensorLocation', [2.8 -0.9], ...
    'Yaw', -75, ...
    'MaxRange', 100, ...
    'DetectionCoordinates', 'Sensor Cartesian', ...
    'HasOrganizedOutput', false, ...
    'HasEgoVehicle', false, ...
    'HasRoadsInputPort', false, ...
    'AzimuthLimits', [-45 45], ...
    'ElevationLimits',[-20 20], ...
    'ActorProfiles', profiles);

sensors{6} = lidarPointCloudGenerator('SensorIndex', 6, ...
    'SensorLocation', [2.8 -0.9], ...
    'Yaw', -135, ...
    'MaxRange', 100, ...
    'DetectionCoordinates', 'Sensor Cartesian', ...
    'HasOrganizedOutput', false, ...
    'HasEgoVehicle', false, ...
    'HasRoadsInputPort', false, ...
    'AzimuthLimits', [-45 45], ...
    'ElevationLimits',[-20 20],...
    'ActorProfiles', profiles);

numSensors = 6;

for i = 1:numSensors
sensors{i}.AzimuthResolution = 0.16;
sensors{i}.Height = 0.4;
sensors{i}.HasRoadsInputPort = false;
sensors{i}.HasOrganizedOutput = false;
end

function [scenario, egoVehicle] = createDrivingScenario

% Construct a drivingScenario object.
scenario = drivingScenario('SampleTime', 0.1);
% Add all road segments
roadCenters = [93.8 -0.1 0;
    -34.5 -0.7 0];
laneSpecification = lanespec(4);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

roadCenters = [27.21 41.91 0;
    27.41 -84.79 0];
laneSpecification = lanespec(4);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road1');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-33.5067135424923 -2.78436555177144 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
% waypoints = [-33.5067135424923 -2.78436555177144 0;
%     21.9 -2.3 0;
%     38.3 -2.3 0;
%     69.2 -2.3 0;
%     91.4 -2.1 0];
% speed = [15;15;15;15;15];
% smoothTrajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [24.8474978485774 7.00641277265186 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [25.4674978485774 8.00641277265186 0;
    25.47 -3.98 0.01;
    25.43 -18.27 0.01;
    25.62 -35.4 0;
    25.82 -200.8 0];
speed = [0;15;15;15;15];
waittime = [1;0;0;0;0];
trajectory(car1, waypoints, speed, waittime);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [24.9209044977542 14.6855726184145 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [25.5509044977542 15.0555726184145 0.01;
    25.53 -4.23 0;
    25.53 -18.73 0;
    25.83 -36.23 0;
    25.73 -150.03 0];
speed = [0;15;15;15;15];
waittime = [1;0;0;0;0];
trajectory(car2, waypoints, speed, waittime);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [25.2166467501627 27.336012610553 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [25.7166467501627 27.336012610553 0.01;
    25.67 20.25 0.01;
    25.54 11.87 0.01;
    25.52 5.12 0.01;
    26.62 0.84 0.01;
    29 -1.6 0;
    32.7 -2.2 0;
    200.4 -2.2 0];
speed = [15;0;10;15;15;15;15;15];
waittime = [0;0.3;0;0;0;0;0;0];
trajectory(car3, waypoints, speed, waittime);
