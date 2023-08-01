function [scenario, egoVehicle] = Test1()
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Generated by MATLAB(R) 9.14 (R2023a) and Automated Driving Toolbox 3.7 (R2023a).
% Generated on: 27-Jul-2023 15:20:25

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [77.6 0 0;
    -9.1 -0.1 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('DoubleSolid')
    laneMarking('DoubleDashed')
    laneMarking('SolidDashed')
    laneMarking('DashedSolid')
    laneMarking('Solid')];
laneSpecification = lanespec(6, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-1.36844193519104 2.17611618287167 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [-1.36844193519104 2.17611618287167 0.01;
    11.48 -1.9 0.01];
speed = [15;15];
yaw =  [0;0];
trajectory(egoVehicle, waypoints, speed, 'Yaw', yaw);

% Add the non-ego actors
vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [7.64 2.06 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');