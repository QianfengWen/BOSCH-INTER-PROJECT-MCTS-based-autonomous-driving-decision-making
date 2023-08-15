function [scenario, egoVehicle, egoWaypoints, allrefPaths, allStatus] = ds6_lanes_roadWith5CarsCuttingIn()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 14-Aug-2023 00:55:16

% Construct a drivingScenario object.
scenario = drivingScenario("SampleTime", 1.0, "StopTime", 15.0);
allrefPaths = cell(0);
allStatus = cell(0);
egoWaypoints =  [-49.1092824239394 -5.5684917295394 0;
    -5.7 -5.6 0;
    21.5 -5.6 0;
    52.4 -5.8 0;
    74.8 -5.6 0;
    96.9 -5.4 0];

% Add all road segments
roadCenters = [-68.9 -0.5 0;
    100.2 0 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [0.9 0.9 0.3])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(6, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

roadCenters = [8.1 70.8 0;
    7.2 -79.3 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [0.96 0.98 0.3])
    laneMarking('Dashed')
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(6, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road1');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-49.1092824239394 -5.5684917295394 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
% speed = [5;5;5;5;5;5];
% trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-22.8087304710501 -1.88342387308028 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [-22.8087304710501 -1.88342387308028 0;
    -5 -2.2 0;
    34.3 -2.5 0];
refPath = referencePathFrenet(waypoints);
allrefPaths{numel(allrefPaths) + 1} = refPath;
speed = [10;0;10];
waittime = [0;1;0];
yaw = [NaN;NaN;NaN];
car1Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car1Status;
trajectory(car1, waypoints, speed, waittime);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-38.6534945522769 -5.44076603001708 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car2');
waypoints = [-38.6534945522769 -5.44076603001708 0;
    -18.4 -5.8 0;
    18.2 -9 0;
    51.9 -9.3 0;
    87.3 -9.2 0];
refPath = referencePathFrenet(waypoints);
allrefPaths{numel(allrefPaths) + 1} = refPath;
speed = [10;10;10;10;10];
waittime = [0;0;0;0;0];
yaw = [NaN;NaN;NaN;NaN;NaN];
car2Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car2Status;
trajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-29.8 -9.6 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car3');
waypoints = [-29.8 -9.6 0;
    -6.2 -9.4 0;
    66.5 -9.2 0];
refPath = referencePathFrenet(waypoints);
allrefPaths{numel(allrefPaths) + 1} = refPath;
speed = [5;5;5];
waittime = [0;0;0];
yaw = [NaN;NaN;NaN];
car3Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car3Status;
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-43.5 -2.2 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car4');
waypoints = [-43.5 -2.2 0;
    -32.2 -2.2 0;
    -22.8 -2.2 0;
    9.1 -5.8 0;
    23.2 -5.8 0;
    37.6 -5.4 0];
refPath = referencePathFrenet(waypoints);
allrefPaths{numel(allrefPaths) + 1} = refPath;
speed = [5;5;5;5;5;5];
waittime = [0;0;0;0;0;0];
yaw =  [NaN;0;0;NaN;NaN;NaN];
car4Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car4Status;
trajectory(car4, waypoints, speed, 'Yaw', yaw);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-56.9 -5.8 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car5');
waypoints = [-56.9 -5.8 0;
    -34.6 -9.4 0;
    -5.4 -5.8 0];
refPath = referencePathFrenet(waypoints);
allrefPaths{numel(allrefPaths) + 1} = refPath;
speed = [5;5;5];
waittime = [0;0;0];
yaw = [NaN;NaN;NaN];
car5Status = struct('speed', speed, 'waittime', waittime, 'yaw', yaw);
allStatus{numel(allStatus) + 1} = car5Status;
trajectory(car5, waypoints, speed);

