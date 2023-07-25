clear;
% Initially state

[egoVehicle, scenario, sensors] = ds2_lanes_roadWith2Cars;
sensorConfigs = cell(numel(sensors),1);

% This is for simpleIntersection scenario
% waypoints = [-33.5067135424923 -2.78436555177144 0;
%     21.9 -2.3 0;
%     38.3 -2.3 0;
%     69.2 -2.3 0;
%     91.4 -2.1 0];

% This is for stuckedCar scenario
waypoints = [10.9 -1.4 0;
    25 -1.4 0;
    34.3 -1.4 0;
    153.4 -1.4 0];

% Create a reference path using waypoints
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath,'TimeResolution',1.0);
% Visualize path regions for sampling strategy visualization
pathPoints = closestPoint(refPath, refPath.Waypoints(:,1:2)); % [x y theta kappa dkappa s]
roadS = pathPoints(:,end);
intersectionS = roadS(3,end);
intersectionBuffer = 20;

% Initial ego state for SimpleIntersection Scenario
% currentEgoState = [-33.5067135424923 -2.78436555177144 0 0 15 0];

% Initial ego state for stuckedCar Scenario
currentEgoState = [10.9 -1.4 0 0 5 0];
helperMoveEgoVehicleToState(egoVehicle, currentEgoState);

egoFrenetState = global2frenet(refPath, currentEgoState);

% Initialize basic configs
TIME = 0;
max_iter = 1000;
accMax = 5;
limitJerk = 15;
speedlimit = 15;
MaxTimeHorizon = 3.0;
TimeResolution = 1.0;
root.visits = 1;
root.time = TIME;
root.state = currentEgoState; % [x y theta kappa speed acc]
root.children = [0];
root.index = 1;
root.score = 0;
root.parent = 0;
root.UCB = inf;
root.egoFrenetState = egoFrenetState;% [s ds dss l dl dll]


AllPath = {currentEgoState};
AllTree = {0};

while advance(scenario)


    targetPosesEgoCoords = targetPoses(egoVehicle);
    actPoses = driving.scenario.targetsToScenario(targetPosesEgoCoords,egoVehicle); % use the targetPoses function to obtain all target actor poses in ego vehicle coordinates
    % tgtPoses = struct('Position', [40.7000 -1.4000 0], 'Velocity', [0 0 0], 'Yaw', 0, 'Pitch', 0, 'Roll', 0,'AngularVelocity', [0 0 0]);


    predictedActPoses = cell(0);
    % compute predicted positions for detected cars
    for i = 1:(MaxTimeHorizon / TimeResolution + 1)
        deltaT = i * TimeResolution;
        predictedActPoses{i} = actPoses;
        for j = 1:numel(actPoses)
            predictedActPoses{i}(j).Position(1) = actPoses(j).Position(1) + actPoses(j).Velocity(1) * cosd(actPoses(j).Yaw) * deltaT; % X position
            predictedActPoses{i}(j).Position(2) = actPoses(j).Position(2) + actPoses(j).Velocity(2) * sind(actPoses(j).Yaw) * deltaT; % Y position

            % Assume no change in Z position
            % tgtPoses(i).Position(3) = tgtPoses(i).Position(3);

            % Update orientation
            predictedActPoses{i}(j).Roll = actPoses(j).Roll + actPoses(j).AngularVelocity(1) * deltaT;
            predictedActPoses{i}(j).Pitch = actPoses(j).Pitch + actPoses(j).AngularVelocity(2) * deltaT;
            predictedActPoses{i}(j).Yaw = actPoses(j).Yaw + actPoses(j).AngularVelocity(3) * deltaT;
        end

    end

    curr_node = root;
    Tree = {};
    Tree{1} = root;

    %PLANNING SIMULATION
    % iterTime = root.visits;
    while (Tree{1}.visits < max_iter)
        curr_node = selection(Tree{1}, Tree);


        if curr_node.time < MaxTimeHorizon
            if numel(curr_node.children) == 1
                if curr_node.visits == 0
                    % rollout
                    cost = roll_out(curr_node, MaxTimeHorizon, TimeResolution, predictedActPoses, accMax, speedlimit, refPath, intersectionS, egoVehicle);
                    % back propagate
                    Tree = back_propagation(curr_node, cost, Tree);
                    Tree = updateUCB(Tree{1}, Tree);
                else
                    % expand
                    Tree = expand(Tree, curr_node, TimeResolution, accMax, speedlimit, refPath, predictedActPoses, egoVehicle);
                end
                curr_node = Tree{1};
            end
        else
            if curr_node.time - MaxTimeHorizon >= 0
                cost = roll_out(curr_node, MaxTimeHorizon, TimeResolution, predictedActPoses, accMax, speedlimit, refPath, intersectionS, egoVehicle);
                Tree = back_propagation(curr_node, cost, Tree);
                Tree = updateUCB(Tree{1}, Tree);
                curr_node = Tree{1};
            end
        end
    end
    % 1 is the index of the root node.
    root = Tree{1};
    if numel(Tree{1}.children) > 1

        expectedNode = Tree{root.children(2)};
        for i = 2:numel(root.children)
            if Tree{root.children(i)}.UCB >= expectedNode.UCB
                expectedNode = Tree{root.children(i)};
            end
        end
        expectedTrajectory = expectedNode.egoFrenetState;
    else
        stop = true;
        expectedNode = root;
        emergencyAcc = root.egoFrenetState(2) / TimeResolution;
        % Let's set the maximum deceleration to be 8m/s^2
        if emergencyAcc >= 8
            emergencyAcc = 8;
            stop = false;
        end
        emergencyJerkS = (-emergencyAcc - root.egoFrenetState(3)) / TimeResolution;
        [displacementEmergencyS, deltaSpeedEmergencyS, displacementEmergencyL, deltaSpeedEmergencyL] = getDisplacement(root, emergencyJerkS, 0, TimeResolution);

        expectedNode.egoFrenetState = root.egoFrenetState + [displacementEmergencyS, deltaSpeedEmergencyS, emergencyJerkS * TimeResolution, displacementEmergencyL, deltaSpeedEmergencyL, 0];
        expectedNode.state = frenet2global(refPath, expectedNode.egoFrenetState);

        disp("Tried emergency break.")
        AllPath{numel(AllPath) + 1} = expectedNode;
        AllTree{numel(AllTree) + 1} = Tree;
        if stop
            disp("There's obstacles forward, the car has stopped.")
            disp(checkCollision(expectedNode, actPoses, egoVehicle))
            break;
        end

    end

    AllPath{numel(AllPath) + 1} = expectedNode;
    AllTree{numel(AllTree) + 1} = Tree;
   
    % reset root properties for next iteration
    root.visits = 1;
    root.time = TIME;
    root.state = expectedNode.state; % [x y theta kappa speed acc]
    root.children = [0];
    root.index = 1;
    root.score = 0;
    root.parent = 0;
    root.UCB = inf;
    root.egoFrenetState = expectedNode.egoFrenetState;% [s ds dss l dl dll]

end



function Tree_ = expand(Tree, node, TimeResolution, accMax, speedlimit, refPath, predictedTgtPoses, egoVehicle)
% 加速度上限为accMax

Jerk3 = -node.egoFrenetState(3) / TimeResolution;
% Set the acceleration all to zero;

% 先添加匀速
newNode3 = struct('state', node.state, 'time', node.time + TimeResolution, ...
    'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState, 'avgScore', 0);
%在这种情况下，刚创建的newNode3的state, egoFrenetState均保持与node（其父）一致,除了加速度以外

[displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(node, Jerk3, 0, TimeResolution);


newNode3.egoFrenetState = newNode3.egoFrenetState + [displacementS3, deltaSpeedS3 , Jerk3 * TimeResolution, displacementL3, deltaSpeedL3, 0];

newNode3.state = frenet2global(refPath, newNode3.egoFrenetState);



if ~checkCollision(newNode3, predictedTgtPoses{int16((newNode3.time) / TimeResolution)}, egoVehicle) && newNode3.egoFrenetState(2) <= speedlimit && newNode3.egoFrenetState(2) >= 0
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode3.index;
    Tree{numel(Tree) + 1} = newNode3;

else
    Tree = back_propagation(Tree{node.index}, 0, Tree);
end


% 然后将加速度以1个分度来模拟，上限是limitJerk/TimeResoluion, 即deltaAcc
for nextAcc = 1:accMax

    % 减速, 先不考虑倒车的情况，因此我们在ds > 0(约等于静止)的时候才考虑减速。同时紧急制动加速度控制在-8m/s^2
    if node.egoFrenetState(2) > 0
        jerk1 = (-nextAcc - node.egoFrenetState(3)) / TimeResolution;
        newNode1 = struct('state', node.state, 'time', node.time + TimeResolution, ...
            'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState,'avgScore', 0);


        [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement(node, jerk1, 0, TimeResolution);

        newNode1.egoFrenetState = newNode1.egoFrenetState + [displacementS1, deltaSpeedS1, jerk1 * TimeResolution, displacementL1, deltaSpeedL1, 0];
        newNode1.state = frenet2global(refPath, newNode1.egoFrenetState);
        if ~checkCollision(newNode1, predictedTgtPoses{int16((newNode1.time) / TimeResolution)}, egoVehicle) && newNode1.egoFrenetState(2) > 0
            Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode1.index;
            Tree{numel(Tree) + 1} = newNode1;
        else
            Tree = back_propagation(Tree{node.index}, 0, Tree);
        end
    end

    % 加速的情况
    jerk2 = (nextAcc - node.egoFrenetState(3)) / TimeResolution;
    newNode2 = struct('state', node.state, 'time', node.time + TimeResolution, ...
        'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState,'avgScore', 0);

    [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(node, jerk2, 0, TimeResolution);



    newNode2.egoFrenetState = newNode2.egoFrenetState + [displacementS2, deltaSpeedS2, jerk2 * TimeResolution, displacementL2, deltaSpeedL2, 0];
    newNode2.state = frenet2global(refPath, newNode2.egoFrenetState);
    if ~checkCollision(newNode2, predictedTgtPoses{int16((newNode2.time) / TimeResolution)}, egoVehicle) && newNode2.egoFrenetState(3) <= 3
        Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode2.index;
        Tree{numel(Tree) + 1} = newNode2;
    else
        Tree = back_propagation(Tree{node.index}, 0, Tree);
    end


end

Tree_ = Tree;
end

function newNode = selection(node, Tree)
% 根据UCB选择最大的子节点
newNode = Tree{node.index};
while numel(newNode.children) ~= 1

    bestChild = Tree{newNode.children(2)};
    for i=2:length(newNode.children)
        if Tree{newNode.children(i)}.UCB >= bestChild.UCB
            bestChild = Tree{newNode.children(i)};
        end
    end
    newNode = bestChild;
end
end

function cost = roll_out(node, MaxTimeHorizon, TimeResolution, predicted, accMax, speedlimit, refPath, checkPoint, egoVehicle)
% 这个函数将执行模拟
cost = 0;  % 初始成本
currNode = node;
% termninal state就是 currTime > MaxTimeHorizon 或者碰撞了

while currNode.time < MaxTimeHorizon
    randomNum = rand();
    randomAcc = randi([1, accMax]);
    if randomNum <= 0.05
        % This situation is for slowing down, so randomAcc is a negative
        % number.

        % Compute Jerk first
        Jerk = (-randomAcc - currNode.egoFrenetState(3)) / TimeResolution;

        % As a sudden break may happen, so the limit jerk for slowing down
        % is allowed.

        % [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(node, deltaAccelerationS, deltaAccelerationL, TimeResolution)
        [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement (currNode, Jerk, 0, TimeResolution);


        % parentaCC stands for the acceleration of the parent node
        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS1, deltaSpeedS1 , Jerk * TimeResolution, displacementL1, deltaSpeedL1, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        if checkCollision(newNode, predicted{newNode.time / TimeResolution}, egoVehicle)
           cost = -100000;
           break;
        end
    elseif randomNum >= 0.95
        % This situation is for speeding up

        % If Acc is out of limit, then let restrain the acc to be a proper
        % number under the limit.
        if randomAcc >= 3
            % So currAcc = currNode.egoFrenetState(3)) - limitJerk * TimeResolution;
            randomAcc = 3;
        end

        % compute jerk.

        Jerk = (randomAcc - currNode.egoFrenetState(3)) / TimeResolution;

        [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(currNode, Jerk, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS2, deltaSpeedS2 , Jerk * TimeResolution, displacementL2, deltaSpeedL2, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        if checkCollision(newNode, predicted{newNode.time / TimeResolution}, egoVehicle)
           
           cost = -100000;
           break;
        end
    else
        % This situation is for keeping speed
        %

        Jerk = -currNode.egoFrenetState(3) / TimeResolution;

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);


        % if abs(Jerk) > limitJerk
        %     % So currAcc = currNode.egoFrenetState(3)) - limitJerk * TimeResolution;
        %     Jerk = limitJerk;
        % end

        [displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(currNode, Jerk, 0, TimeResolution);


        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS3, deltaSpeedS3, Jerk * TimeResolution, displacementL3, deltaSpeedL3, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        if checkCollision(newNode, predicted{newNode.time / TimeResolution}, egoVehicle)
           cost = -100000;
           break;
        end
    end
end

% special case for one time simulation for the terminal node.
if node.time >= MaxTimeHorizon
    randomNum = rand();
    randomAcc = randi([1, accMax]);
    if randomNum <= 0.05
        % This situation is for slowing down, so randomAcc is a negative
        % number.

        % Compute Jerk first
        Jerk = (-randomAcc - currNode.egoFrenetState(3)) / TimeResolution;

        % As a sudden break may happen, so the limit jerk for slowing down
        % is allowed.

        % [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(node, deltaAccelerationS, deltaAccelerationL, TimeResolution)
        [displacementS1, deltaSpeedS1, displacementL1, deltaSpeedL1] = getDisplacement (currNode, Jerk, 0, TimeResolution);


        % parentaCC stands for the acceleration of the parent node
        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS1, deltaSpeedS1 , Jerk * TimeResolution, displacementL1, deltaSpeedL1, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;

    elseif randomNum >= 0.95
        % This situation is for speeding up

        % If Acc is out of limit, then let restrain the acc to be a proper
        % number under the limit.
        if randomAcc >= 3
            % So currAcc = currNode.egoFrenetState(3)) - limitJerk * TimeResolution;
            randomAcc = 3;
        end

        % compute jerk.

        Jerk = (randomAcc - currNode.egoFrenetState(3)) / TimeResolution;

        [displacementS2, deltaSpeedS2, displacementL2, deltaSpeedL2] = getDisplacement(currNode, Jerk, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS2, deltaSpeedS2 , Jerk * TimeResolution, displacementL2, deltaSpeedL2, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        
    else
        % This situation is for keeping speed
        %

        Jerk = -currNode.egoFrenetState(3) / TimeResolution;

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);


        % if abs(Jerk) > limitJerk
        %     % So currAcc = currNode.egoFrenetState(3)) - limitJerk * TimeResolution;
        %     Jerk = limitJerk;
        % end

        [displacementS3, deltaSpeedS3, displacementL3, deltaSpeedL3] = getDisplacement(currNode, Jerk, 0, TimeResolution);


        newNode.egoFrenetState = newNode.egoFrenetState + [displacementS3, deltaSpeedS3, Jerk * TimeResolution, displacementL3, deltaSpeedL3, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
    end
end

cost2 = costFunction(newNode, newNode.egoFrenetState(3), checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit);

cost = cost + cost2(1);
disp("currNode.state:");
disp(currNode.state);
disp("currNode.egoFrenetState:");
disp(currNode.egoFrenetState);
disp("currNode's cost:")
disp("1. total_cost 2. cost_comfort 3. cost_safety 4. cost_pass 5. cost_stimulation")
disp(cost2);
disp("If passed intersection:")
disp(currNode.egoFrenetState(1) > checkPoint)

end

function tree_ = back_propagation(node, score, tree)
% 用给定的分数更新节点
while node.parent ~= 0
    tree{node.index}.score = node.score + score;
    tree{node.index}.visits = node.visits + 1;
    node = tree{node.parent};
end
tree{node.index}.score = node.score + score;
tree{node.index}.visits = node.visits + 1;
tree_ = tree;
end

function [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(node, jerkS, jerkL, TimeResolution)
% deltaSpeed = (2 * node.acc + deltaAcc) * 1/2 * TimeResolution, so we
% consider to use integral.
% (currSpeed + deltaSpeed) * TimeResolution = distance
% distance = currSpeed * TimeResolution + 1/2 * node.acc *
% TimeResolution^2 + 1/6 * deltaAcc) * TimeResolution ^3

deltaT = TimeResolution;
deltaSpeedS = node.egoFrenetState(3) * TimeResolution + 1/2 * jerkS * TimeResolution^2;
displacementS = node.egoFrenetState(2) * deltaT + 1/2 * node.egoFrenetState(3) * TimeResolution^2 + 1/6 * jerkS * TimeResolution^3;
deltaSpeedL = node.egoFrenetState(6) * TimeResolution + 1/2 * jerkL * TimeResolution^2;
displacementL = node.egoFrenetState(5) * deltaT + 1/2 * node.egoFrenetState(6) * TimeResolution^2 + 1/6 * jerkL * TimeResolution^3;
end

function tree_ = updateUCB(node, tree)
queue = {node};
while ~isempty(queue)
    currNode = queue{1};
    queue(1) = [];
    if currNode.visits == 0
        tree{currNode.index}.UCB = inf;
        tree{currNode.index}.avgScore = inf;
    else
        tree{currNode.index}.UCB = currNode.score / currNode.visits + 5 * sqrt(log(tree{1}.visits) / currNode.visits);
        tree{currNode.index}.avgScore = tree{currNode.index}.score / tree{currNode.index}.visits;
    end
    if numel(currNode.children) ~= 1
        for i = 1:(numel(currNode.children) - 1)
            queue{numel(queue) + 1} = tree{currNode.children(i + 1)};
        end
    end

end
tree_ = tree;

end

 function flag = checkCollision(node, OBJSTATE, egoVehicle)
flag = false;
for i = 1:numel(OBJSTATE)
    distance = sqrt((node.state(1) - OBJSTATE(i).Position(1)) ^ 2 + (node.state(2) - OBJSTATE(i).Position(2)) ^ 2);
    objCarDim = 0.5 * sqrt(egoVehicle.Length ^ 2 + egoVehicle.Width ^ 2);
    egoCarDim = 0.5 * sqrt(egoVehicle.Length ^ 2 + egoVehicle.Width ^ 2);
    if distance <= objCarDim + egoCarDim
        flag = true;
        break
    end
end
end


function cost = costFunction(node, acc, checkPoint, predictedTgtPoses, MaxTimeHorizon, TimeResolution, egoVehicle, speedlimit)
comfort = abs(acc);

% jerk calculation
jerk = (acc - node.parentaCC) / TimeResolution;

cost_comfort = calculateComfortCost(jerk, comfort, acc, node);
cost_pass = calculatePassibilityCost(node, checkPoint, MaxTimeHorizon);
cost_safety = calculateSafetyCost(node, predictedTgtPoses, TimeResolution, egoVehicle);

% stimulate the car to moveforward.
if node.egoFrenetState(2) < speedlimit
    % the standard level for cost_stimulation is 0.0, e.g.
    % speed ==  speedlimit, acc == 0
    cost_stimulation = (speedlimit - node.egoFrenetState(2)) - 0.2 * node.egoFrenetState(3);

elseif node.egoFrenetState(2) > speedlimit
    expectAcc = (node.egoFrenetState(2) - speedlimit) / TimeResolution;
    cost_stimulation = abs(acc - expectAcc);

else
    cost_stimulation = 0.0;
end

% calculate cost

cost = [-(cost_comfort + cost_safety + cost_pass + cost_stimulation), -cost_comfort, -cost_safety, -cost_pass, -cost_stimulation];

end

function cost_comfort = calculateComfortCost(jerk, comfort, acc, node)

cost_comfort_jerk = 2 / (1 + exp(-jerk));
cost_comfort_acc = 0;
cost_comfort_alter = 0;

if comfort < -2
    cost_comfort_acc = - 2 * comfort;
end

if acc * node.parentaCC < 0
    cost_comfort_alter = 5.0;
end

cost_comfort = cost_comfort_acc + cost_comfort_jerk + cost_comfort_alter;

% The comfort reward function is a sigmoid-like function.


end

function cost_pass = calculatePassibilityCost(node, checkPoint, MaxTimeHorizon)

% The reward should be higher if the vehicle is able to pass the intersection more smoothly
% For those node's time < MaxTimeHorizon, just let the reward be 0.
if node.time >= MaxTimeHorizon

    if node.egoFrenetState(1) > checkPoint
        cost_pass = 0.0;
    else
        cost_pass = 2.0 + abs(node.egoFrenetState(1) - checkPoint);
    end
else
    cost_pass = 0.0;
end
end

function cost_safety = calculateSafetyCost(node, predictedTgtPoses, TimeResolution, egoVehicle)

SAFE_DISTANCE = 5;
Emergency_Distance = 1;
cost_safety = 0;
speed = node.egoFrenetState(2);
acc = node.egoFrenetState(3);


predicted = predictedTgtPoses{int16((node.time) / TimeResolution)};
% Design a Piecewise function so that it returns a logorithmn decrease
% when the distance is smaller than SAFE_DISTANCE

for i = 1:numel(predictedTgtPoses{1})

    distance = sqrt((node.state(1) - predicted(i).Position(1)) ^ 2 + (node.state(2) - predicted(i).Position(2)) ^ 2);
    objCarDim = 0.5 * sqrt(egoVehicle.Length ^ 2 + egoVehicle.Width ^ 2);
    egoCarDim = 0.5 * sqrt(egoVehicle.Length ^ 2 + egoVehicle.Width ^ 2);

    total_distance = distance - objCarDim - egoCarDim;

    if total_distance <= Emergency_Distance
        % The growth of the cost function on this situation follows a linear trends
        % with a slope of -10.
        cost_safety_temp = -10 * (distance - Emergency_Distance) + 3.0 + 5 * speed ^2 + acc^3;

    elseif distance > Emergency_Distance && distance <= SAFE_DISTANCE

        cost_safety_temp = 1.0 * (distance - SAFE_DISTANCE) / (Emergency_Distance - SAFE_DISTANCE)  + 2.0 + speed ^2 + acc^3;


    else
        % Normally give a no more than 2.0 reward for the action that
        % keeps distance.
        cost_safety_temp = max(-1/100 * (distance - SAFE_DISTANCE) + 2.0, 0.0);
    end

    cost_safety = cost_safety + cost_safety_temp;
end

end

