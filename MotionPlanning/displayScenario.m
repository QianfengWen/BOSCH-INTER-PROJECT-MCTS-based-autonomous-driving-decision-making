function displayScenario(AllPath, refPaths, profiles, allStatus, roadConfigs)
[scenario, egoVehicle] = createScenario(AllPath, refPaths, profiles, allStatus, roadConfigs);
plot(scenario);
chasePlot(egoVehicle);
while advance(scenario)
    pause(0.0001);
end
end

function [scenario, egoVehicle] = createScenario(AllPath, refPaths, profiles, allStatus, roadConfigs)
scenario = drivingScenario("StopTime",20);

% Add all road segments
for i = 1:numel(roadConfigs)
    tempRoad = roadConfigs{i};
    roadCenters = tempRoad.roadCenters;
    marking = tempRoad.marking;
    laneSpecification = tempRoad.laneSpecification;
    road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');
end

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
    if all(newStatus.state(1:2) == waypoints(numel(waypoints(:, 1)), 1:2))
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
end