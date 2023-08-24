function displayScenario(AllPath, actorWaypoints, profiles, allStatus, roadConfigs)
[scenario, egoVehicle, egoStatus] = createScenario(AllPath, actorWaypoints, profiles, allStatus, roadConfigs);
plot(scenario);
chasePlot(egoVehicle);
% while advance(scenario)
%     pause(0.0001);
% end
for i=1:int32((numel(AllPath) - 1) / 0.01)
    % egoStatus = struct('Position', egoVehicle.Position, ...
    % 'Velocity', egoVehicle.Velocity, ...
    % 'Yaw', egoVehicle.Yaw, ...
    % 'angularVelocity', egoVehicle.AngularVelocity);
    pause(0.0001);
    egoVehicle.Position = egoStatus.Position(i, :);
    egoVehicle.Velocity = egoStatus.Velocity(i, :);
    egoVehicle.Yaw = egoStatus.Yaw(i);
    egoVehicle.AngularVelocity = egoStatus.angularVelocity(i, :);
    advance(scenario);
end

end

function [scenario, egoVehicle, egoStatus] = createScenario(AllPath, actorWaypoints, profiles, allStatus, roadConfigs)
StopTime = numel(AllPath);
scenario = drivingScenario("StopTime", StopTime);
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
    'Position', [AllPath{1}(1:2) 0], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car');
waypoints = [AllPath{1}(1:2) 0];
speed = AllPath{1}(5);
% waittime = 0;
yaw = rad2deg(AllPath{1}(3));
orientation = [yaw 0 0];
toa = 0:numel(AllPath) - 1;
for i = 2: numel(AllPath)
    newStatus = AllPath{i};
    speed = [speed; newStatus.state(5)];
    yaw = [yaw; rad2deg(newStatus.state(3))];
    orientation = [orientation; yaw(i) 0 0];
    waypoints = [waypoints; newStatus.state(1:2) 0];
end
Orientation = quaternion(orientation, ...
                          "eulerd","ZYX","frame");
egotrajectory = waypointTrajectory(waypoints, ...
    TimeOfArrival=toa, ...
    Orientation=Orientation);
egoStatus = struct('Position', egoVehicle.Position, ...
    'Velocity', egoVehicle.Velocity, ...
    'Yaw', egoVehicle.Yaw, ...
    'angularVelocity', egoVehicle.AngularVelocity);
time = 0.01;
while time <= egotrajectory.TimeOfArrival(end)
    [position,orientation,velocity,~,angularVelocity] = lookupPose(egotrajectory,time);
    egoStatus.Position = [egoStatus.Position; position];
    egoStatus.Velocity = [egoStatus.Velocity; velocity];
    tempYaw = eulerd(orientation,'ZYX','frame');
    egoStatus.Yaw = [egoStatus.Yaw; tempYaw(1)];
    egoStatus.angularVelocity = [egoStatus.angularVelocity; rad2deg(angularVelocity)];
    time = time + 0.01;  
end
% for i = 2: numel(AllPath)
%     newStatus = AllPath{i};
%     if all(newStatus.state(1:2) == waypoints(numel(waypoints(:, 1)), 1:2)) || newStatus.state(5) == 0
%         if waittime(numel(waittime) - 1, 1) == 0
%             waittime(numel(waittime), 1) = waittime(numel(waittime), 1) + newStatus.time;
%         end
%     else
%         waittime = [waittime; 0];
%         speed = [speed; newStatus.state(5)];
%         yaw = [yaw; rad2deg(newStatus.state(3))];
%         waypoints = [waypoints; newStatus.state(1:2) 0];
%     end
% end
% trajectory(egoVehicle, waypoints, speed, waittime, 'Yaw', yaw);

% Add the non-ego actors
for i = 1:numel(actorWaypoints)
    if numel(actorWaypoints{i}) == 3
        vehicle(scenario, ...
            'ClassID', profiles(i + 1).ClassID, ...
            'Position', [actorWaypoints{i}(1:2) 0], ...
            'Mesh', driving.scenario.carMesh, ...
            'Name', strcat('car', num2str(i)));
    elseif numel(actorWaypoints{i}(:, 1)) ~= 1
        tempCar =  vehicle(scenario, ...
            'ClassID', profiles(i + 1).ClassID, ...
            'Position', [actorWaypoints{i}(1:2) 0], ...
            'Mesh', driving.scenario.carMesh, ...
            'Name', strcat('car', num2str(i)));
        % eval(['car',num2str(i),'=','tempCar',';']);
        waypoints = actorWaypoints{i};
        waypoints(:,3) = 0;
        speed = allStatus{i}.speed;
        waittime = allStatus{i}.waittime;
        yaw =  allStatus{i}.yaw;
        trajectory(tempCar, waypoints, speed, waittime,'Yaw', yaw);
    else
        vehicle(scenario, ...
            'ClassID', profiles(i + 1).ClassID, ...
            'Position', [actorWaypoints{i}(1:2) 0], ...
            'Mesh', driving.scenario.carMesh, ...
            'Name', strcat('car', num2str(i)));
    end
end
end