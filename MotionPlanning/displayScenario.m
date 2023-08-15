function displayScenario(AllPath, refPaths, profiles, allStatus)
[scenario, egoVehicle] = createScenario(AllPath, refPaths, profiles, allStatus);
plot(scenario);
chasePlot(egoVehicle);
while advance(scenario)
    pause(0.0001);
end
end

function [scenario, egoVehicle] = createScenario(AllPath, refPaths, profiles, allStatus)
scenario = drivingScenario;

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
    'Position', [-23.2079542936621 -5.86580669860298 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [AllPath{1}(1:3)];
speed = AllPath{1}(5);
waittime = 0;
for i = 2: numel(AllPath)
    newStatus = AllPath{i};
    if all(newStatus.state(1:3) == waypoints(numel(waypoints(:, 1)), :))
        if waittime(numel(waittime) - 1, 1) == 0
            waittime(numel(waittime), 1) = waittime(numel(waittime), 1) + newStatus.time;
        end
    else
        waittime = [waittime; 0];
        speed = [speed; newStatus.state(5)];
        waypoints = [waypoints; newStatus.state(1:2) 0];
    end
end
yaw = zeros(numel(waypoints(:, 1)), 1);
trajectory(egoVehicle, waypoints, speed, waittime, 'Yaw', yaw);

% Add the non-ego actors
for i = 1:numel(refPaths)
    if numel(refPaths{i}) == 6
        vehicle(scenario, ...
            'ClassID', profiles(i + 1).ClassID, ...
            'Position', [refPaths{i}(1:2) 0], ...
            'Mesh', driving.scenario.carMesh, ...
            'Name', strcat('car', num2str(i)));
    elseif numel(refPaths{i}.Waypoints(:, 1)) ~= 1
        tempCar =  vehicle(scenario, ...
            'ClassID', profiles(i + 1).ClassID, ...
            'Position', refPaths{i}.Waypoints(1, :), ...
            'Mesh', driving.scenario.carMesh, ...
            'Name', strcat('car', num2str(i)));
        % eval(['car',num2str(i),'=','tempCar',';']);
        waypoints = refPaths{i}.Waypoints;
        waypoints(:,3) = 0;
        speed = allStatus{i}.speed;
        waittime = allStatus{i}.waittime;
        yaw =  allStatus{i}.yaw;
        trajectory(tempCar, waypoints, speed, waittime,'Yaw', yaw);
    else
        vehicle(scenario, ...
            'ClassID', profiles(i + 1).ClassID, ...
            'Position', [refPaths{i}(1:2) 0], ...
            'Mesh', driving.scenario.carMesh, ...
            'Name', strcat('car', num2str(i)));
    end
end
% car1 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [24 1.3 0], ...
%     'Mesh', driving.scenario.carMesh, ...
%     'Name', 'Car1');
% waypoints = [24 1.3 0;
%     8.3 -1.9 0;
%     -1.1 -16.3 0;
%     -1.9 -28.9 0;
%     -2.3 -77.8 0];
% speed = [10;10;10;10;10];
% yaw =  [180;NaN;NaN;NaN;NaN];
% trajectory(car1, waypoints, speed, 'Yaw', yaw);
%
% car2 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [31 1.4 0], ...
%     'Mesh', driving.scenario.carMesh, ...
%     'Name', 'Car2');
% waypoints = [31 1.4 0;
%     19.1 1.5 0;
%     4.2 -6.1 0;
%     2.3 -19.1 0;
%     2.2 -34.4 0;
%     1.9 -67.2 0];
% speed = [10;10;10;10;10;10];
% yaw =  [180;NaN;NaN;NaN;NaN;NaN];
% trajectory(car2, waypoints, speed, 'Yaw', yaw);
%
% car3 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [49.8 1.7 0], ...
%     'Mesh', driving.scenario.carMesh, ...
%     'Name', 'Car3');
% waypoints = [49.8 1.7 0;
%     16.2 0.3 0;
%     8.1 -1.7 0;
%     0.2 -10.5 0;
%     -1.9 -22.4 0;
%     -2 -66.9 0];
% speed = [5;15;10;10;10;15];
% yaw =  [180;NaN;NaN;NaN;NaN;NaN];
% trajectory(car3, waypoints, speed,'Yaw', yaw);
%
% car4 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [63.4 1.5 0], ...
%     'Mesh', driving.scenario.carMesh, ...
%     'Name', 'Car4');
% waypoints = [63.4 1.5 0;
%     36.4 1.4 0;
%     17.3 -0.3 0;
%     7.2 -8.5 0;
%     5.6 -18.4 0;
%     5.8 -62.3 0];
% speed = [5;10;15;15;15;15];
% yaw =  [180;NaN;NaN;NaN;NaN;NaN];
% trajectory(car4, waypoints, speed, 'Yaw', yaw);
%
% car5 = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [73.9 1.7 0], ...
%     'Mesh', driving.scenario.carMesh, ...
%     'Name', 'Car5');
% waypoints = [73.9 1.7 0;
%     55 1.9 0;
%     28.9 2.1 0;
%     8.5 -5 0;
%     5.7 -17.5 0;
%     5.7 -53 0];
% speed = [5;7;8;10;10;10];
% waittime = [0;0;0;0;0;0];
% trajectory(car5, waypoints, speed, waittime);
%
end

