clear;
% Initially state
[scenario, egoVehicle, lidars] = stuckedCar;
sensorConfigs = cell(numel(lidars),1);
% Fill in sensor configurations
for i = 1:numel(sensorConfigs)
    sensorConfigs{i} = helperGetLidarConfig(lidars{i},egoVehicle);
end
% This is for simpleIntersection scenario
% waypoints = [-33.5067135424923 -2.78436555177144 0;
%     21.9 -2.3 0;
%     38.3 -2.3 0;
%     69.2 -2.3 0;
%     91.4 -2.1 0];

% This is for stuckedCar scenario
waypoints = [20.5 -1.9 0;
    26.6 -1.7 0;
    36.7 -1.4 0;
    58 -1.4 0];

% Create a reference path using waypoints
refPath = referencePathFrenet(waypoints);
connector = trajectoryGeneratorFrenet(refPath,'TimeResolution',0.2);
% Visualize path regions for sampling strategy visualization
pathPoints = closestPoint(refPath, refPath.Waypoints(:,1:2)); % [x y theta kappa dkappa s]
roadS = pathPoints(:,end);
intersectionS = roadS(2,end);
intersectionBuffer = 20;

% Initial ego state for SimpleIntersection Scenario
% currentEgoState = [-33.5067135424923 -2.78436555177144 0 0 15 0];

% Initial ego state for stuckedCar Scenario
currentEgoState = [20.5 -1.9 0 0 15 0];
helperMoveEgoVehicleToState(egoVehicle, currentEgoState);

% Initial Configs
TIME = 0;
max_iter = 1000;
accMax = 15;
MaxTimeHorizon = 3.0;
TimeResolution = 0.2;
% tgtPoses = targetPoses(egoVehicle);
tgtPoses = struct('Position', [30.7000 -1.4000 0], 'Velocity', [0 0 0], 'Yaw', 0, 'Pitch', 0, 'Roll', 0,'AngularVelocity', [0 0 0]);
egoFrenetState = global2frenet(refPath, currentEgoState);
root.visits = 1;
root.time = TIME;
root.state = currentEgoState; % [x y theta kappa speed acc]
root.children = [0];
root.index = 1;
root.score = 0;
root.parent = 0;
root.UCB = inf;
root.egoFrenetState = egoFrenetState;% [s ds dss l dl dll]
predictedTgtPoses = cell(0);
checkPoint = [];
% compute predicted positions for detected cars
for i = 1:(MaxTimeHorizon / TimeResolution)
    deltaT = i * TimeResolution;
    predictedTgtPoses{i} = tgtPoses;
    for j = 1:numel(tgtPoses)
        predictedTgtPoses{i}(j).Position(1) = tgtPoses(j).Position(1) + tgtPoses(j).Velocity(1) * cosd(tgtPoses(j).Yaw) * deltaT; % X position
        predictedTgtPoses{i}(j).Position(2) = tgtPoses(j).Position(2) + tgtPoses(j).Velocity(2) * sind(tgtPoses(j).Yaw) * deltaT; % Y position

        % Assume no change in Z position
        % tgtPoses(i).Position(3) = tgtPoses(i).Position(3);

        % Update orientation
        predictedTgtPoses{i}(j).Roll = tgtPoses(j).Roll + tgtPoses(j).AngularVelocity(1) * deltaT;
        predictedTgtPoses{i}(j).Pitch = tgtPoses(j).Pitch + tgtPoses(j).AngularVelocity(2) * deltaT;
        predictedTgtPoses{i}(j).Yaw = tgtPoses(j).Yaw + tgtPoses(j).AngularVelocity(3) * deltaT;
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
                reward = roll_out(curr_node, MaxTimeHorizon, TimeResolution, predictedTgtPoses, egoVehicle, accMax, refPath, intersectionS);
                % back propagate
                Tree = back_propagation(curr_node, reward, Tree);
                Tree = updateUCB(Tree{1}, Tree);
            else
                % expand
                Tree = expand(Tree, curr_node, TimeResolution, accMax, refPath, predictedTgtPoses, egoVehicle);
            end
            curr_node = Tree{1};
        end
    else
        if curr_node.time - MaxTimeHorizon > 0
            reward = roll_out(curr_node, MaxTimeHorizon, TimeResolution, predictedTgtPoses, egoVehicle, accMax, refPath, intersectionS);
            Tree = back_propagation(curr_node, reward, Tree);
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

    expectedNode = Tree{root.index};
end

function Tree_ = expand(Tree, node, TimeResolution, accMax, refPath, predictedTgtPoses, egoVehicle)
% 假设加速度上限为accMax

% 先添加匀速
newNode3 = struct('state', node.state, 'time', node.time + TimeResolution, ...
    'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState);
[next_position_s3, next_position_l3] = getNextPosition(node, 0, 0, TimeResolution);

newNode3.egoFrenetState = newNode3.egoFrenetState + [next_position_s3, 0 , 0, next_position_l3, 0, 0];
newNode3.state = frenet2global(refPath, newNode3.egoFrenetState);

if ~checkCollision(newNode3, predictedTgtPoses(int16((newNode3.time) / TimeResolution)), egoVehicle)
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode3.index;
    Tree{numel(Tree) + 1} = newNode3;

else
        Tree{1}.visits = Tree{1}.visits + 1;
end

% 然后将加速度以2个分度来进行模拟
for i=1:2:accMax+1

    % 减速, 先不考虑倒车的情况，因此我们在ds > 0.05(约等于静止)的时候才考虑减速。
    if node.egoFrenetState(2) > 0.05
        newNode1 = struct('state', node.state, 'time', node.time + TimeResolution, ...
            'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState);
        [next_position_s1, next_position_l1] = getNextPosition(node, -i, 0, TimeResolution);

        newNode1.egoFrenetState = newNode1.egoFrenetState + [next_position_s1, (-i) * TimeResolution , -i, next_position_l1, 0, 0];
        newNode1.state = frenet2global(refPath, newNode1.egoFrenetState);
        if ~checkCollision(newNode1, predictedTgtPoses(int16((newNode1.time) / TimeResolution + 1)), egoVehicle)
            Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode1.index;
            Tree{numel(Tree) + 1} = newNode1;
        else
            Tree{1}.visits = Tree{1}.visits + 1;
        end
    end

    %加速
    newNode2 = struct('state', node.state, 'time', node.time + TimeResolution, ...
        'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState);
    [next_position_s2, next_position_l2] = getNextPosition(node, i, 0, TimeResolution);

    newNode2.egoFrenetState = newNode2.egoFrenetState + [next_position_s2, i * TimeResolution , i, next_position_l2, 0, 0];
    newNode2.state = frenet2global(refPath, newNode2.egoFrenetState);
    if ~checkCollision(newNode2, predictedTgtPoses(int16((newNode2.time) / TimeResolution + 1)), egoVehicle)
        Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode2.index;
        Tree{numel(Tree) + 1} = newNode2;
    else
        Tree{1}.visits = Tree{1}.visits + 1;
    end


end
Tree_ = Tree;
end

function newNode = selection(node, Tree)
% 根据UCB选择最好的子节点
newNode = Tree{node.index};
while numel(newNode.children) ~= 1
    % for i=2:length(newNode.children)
    %     if Tree{newNode.children(i)}.visits == 0
    %         Tree{newNode.children(i)}.UCB = inf;
    %     else
    %         Tree{newNode.children(i)}.UCB = newNode.score / newNode.visits + 2 * sqrt(log(Tree{1}.visits) / newNode.visits);
    %     end
    % end
    bestChild = Tree{newNode.children(2)};
    for i=2:length(newNode.children)
        if Tree{newNode.children(i)}.UCB >= bestChild.UCB
            bestChild = Tree{newNode.children(i)};
        end
    end
    newNode = bestChild;
end
end

function reward = roll_out(node, MaxTimeHorizon, TimeResolution, predicted, egoVehicle, accMax, refPath, checkPoint)
% 这个函数将执行模拟
reward = 0;  % 初始奖励
currNode = node;
% termninal state就是 currTime > MaxTimeHorizon 或者碰撞了

while currNode.time < MaxTimeHorizon
    randomNum = rand();
    randomAcc = randi([0, accMax]);
    if randomNum <= 0.20

        % [nextPositionS, nextPositionL] = getNextPosition(node, accelerationS, accelerationL, TimeResolution)
        [slowDownS1, slowDownL1] = getNextPosition(currNode, -randomAcc, 0, TimeResolution);


        % parentaCC stands for the acceleration of the parent node
        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [slowDownS1, (-randomAcc) * TimeResolution , -randomAcc, slowDownL1, 0, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        reward = reward + rewardFunction(newNode, randomAcc, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle);
    elseif randomNum >= 0.80
        [speedUpS1, speedUpL1] = getNextPosition(currNode, randomAcc, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [speedUpS1, (randomAcc) * TimeResolution , randomAcc, speedUpL1, 0, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        reward = reward + rewardFunction(newNode, randomAcc, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle);
    else
        [keepSpeedS1, keepSpeedL1] = getNextPosition(currNode, 0, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parentaCC', currNode.egoFrenetState(3), 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [keepSpeedS1, 0 , 0, keepSpeedL1, 0, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        reward = reward + rewardFunction(newNode, randomAcc, checkPoint, predicted, MaxTimeHorizon, TimeResolution, egoVehicle);
    end
end

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

function [nextPositionS, nextPositionL] = getNextPosition(node, accelerationS, accelerationL, TimeResolution)
deltaT = TimeResolution;
nextPositionS = node.egoFrenetState(2) * deltaT + 0.5 * accelerationS * deltaT^2;
nextPositionL = node.egoFrenetState(5) * deltaT + 0.5 * accelerationL * deltaT^2;
end

function tree_ = updateUCB(node, tree)
queue = {node};
while ~isempty(queue)
    currNode = queue{1};
    queue(1) = [];
    if currNode.visits == 0
        tree{currNode.index}.UCB = inf;
    else
        tree{currNode.index}.UCB = currNode.score / currNode.visits + 2 * sqrt(log(tree{1}.visits) / currNode.visits);
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
for i = 1:numel(OBJSTATE{1})
    distance = sqrt((node.state(1) - OBJSTATE{1}(i).Position(1)) ^ 2 + (node.state(2) - OBJSTATE{1}(i).Position(2)) ^ 2);
    objCarDim = sqrt(egoVehicle.Length ^ 2 + egoVehicle.Width ^ 2);
    egoCarDim = sqrt(egoVehicle.Length ^ 2 + egoVehicle.Width ^ 2);
    if distance <= objCarDim + egoCarDim
        flag = true;
        break
    end
end
end


function reward = rewardFunction(node, acc, checkPoint, predictedTgtPoses, MaxTimeHorizon, TimeResolution, egoVehicle)
    comfort = abs(acc);

    % jerk calculation
    jerk = (acc - node.parentaCC) / TimeResolution;

    % calculate rewards
    reward_comfort = calculateComfortReward(jerk, comfort);
    reward_pass = calculatePassibilityReward(node, checkPoint, MaxTimeHorizon);
    reward_safety = calculateSafetyReward(node, predictedTgtPoses, TimeResolution);

    % Combine rewards with different weights
    % Here the weights can be adjusted according to the requirement
    reward = 0.3 * reward_comfort + 0.3 * reward_pass + 0.4 * reward_safety;
end

function reward_comfort = calculateComfortReward(jerk, comfort)

    % Define a comfort score based on the jerk and comfort of the ride.
    % Assume the acceptable range of jerk is [-jerk_limit, jerk_limit], same for comfort
    jerk_limit = 3.0;
    comfort_limit = 1.5;

    jerk_normalized = min(max((jerk_limit - abs(jerk)) / jerk_limit, 0.0), 1.0);
    comfort_normalized = min(max((comfort_limit - abs(comfort)) / comfort_limit, 0.0), 1.0);

    % Higher the comfort or lower the jerk, higher the reward
    reward_comfort = 0.5 * jerk_normalized + 0.5 * comfort_normalized;
end

function reward_pass = calculatePassibilityReward(node, checkPoint, MaxTimeHorizon)

    % The reward should be higher if the vehicle is able to pass the intersection more smoothly
    reward_pass = 0; % For those node's time < MaxTimeHorizon, just let the reward be 0.
    if node.time >= MaxTimeHorizon

        if node.egoFrenetState(1) > checkPoint
            reward_pass = 10.0;
        else
            reward_pass = -10.0;
        end
    else
         if node.egoFrenetState(1) > checkPoint
            reward_pass = 10.0;
         end
    end
end

function reward_safety = calculateSafetyReward(node, predictedTgtPoses, TimeResolution)
    SAFE_DISTANCE = 30;
    reward_safety = 0;

    
    predicted = predictedTgtPoses(int16((node.time) / TimeResolution));
    % Design a Piecewise function so that it returns a logorithmn decrease
    % when the distance is smaller than SAFE_DISTANCE

    for i = 1:numel(predictedTgtPoses{1})
        distance = sqrt((node.state(1) - predicted{1}(i).Position(1)) ^ 2 + (node.state(2) - predicted{1}(i).Position(2)) ^ 2);
        if distance <= 0.9 * SAFE_DISTANCE
            % When the distance is smaller than the safe range, the
            % reward_safety decreases rapidly, to make it balanced, as
            % previously we set the reward of passing the intersectoin
            % successfully to 10.0, so as this rate, the distance of
            % 27 (under safe_distance = 30) would balance the
            % reward. (Avoiding the risk of accelerating to the
            % intersection in a possibility of crash)

            reward_safety = 100 * log(distance / SAFE_DISTANCE);

        else
            % Normally give a no more than 2.0 reward for the action that
            % keeps distance.
            reward_safety = min(0.2 * log(distance/ SAFE_DISTANCE), 2.0);
        end
    end
    
end

