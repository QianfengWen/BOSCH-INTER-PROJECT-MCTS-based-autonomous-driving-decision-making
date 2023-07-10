% Initially state
TIME = 0;
max_iter = 1000;
accMax = 15;
MaxTimeHorizon = 3;
TimeResolution = 0.2;
tgtPoses = targetPoses(egoVehicle);
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
    
    
    if curr_node.time < MaxTimeHorizon && ... 
        ~checkCollision(curr_node, predictedTgtPoses(int16((curr_node.time - TIME) / TimeResolution + 1)), egoVehicle)
        if numel(curr_node.children) == 1
            if curr_node.visits == 0
                % rollout
                reward = roll_out(curr_node, MaxTimeHorizon, TimeResolution, predictedTgtPoses, egoVehicle, accMax, refPath);
                % back propagate
                Tree = back_propagation(curr_node, reward, Tree);
                Tree = updateUCB(Tree{1}, Tree);
            else
                % expand+
                Tree = expand(Tree, curr_node, TimeResolution, accMax, refPath);
            end
            curr_node = Tree{1};
        end
    else
        if curr_node.time == MaxTimeHorizon
            reward = roll_out(curr_node, maxtimehorizon, TimeResolution);
            Tree = back_propagation(curr_node, reward, Tree);
            Tree = updateUCB(Tree{1}, Tree);
            curr_node = Tree{1};
        end
    end
end
root = Tree{1};
expectedNode = Tree{root.children(2)};
for i = 2:numel(root.children)
    if Tree{root.children(i)}.UCB >= expectedNode.UCB
            expectedNode = Tree{root.children(i)};
    end
end
expectedTrajectory = expectedNode.egoFrenetState;



% visulization

% Update ego state based on MCTS

% Update obj state based on inputs（since we assume perfect detection.）


% other functions we need, such collision detection, rollout simulation,
% kinematics constrain
function Tree_ = expand(Tree, node, TimeResolution, accMax, refPath)
% 假设加速度上限为accMax
for i=1:2:accMax+1
    % 减速
    newNode1 = struct('state', node.state, 'time', node.time + TimeResolution, ...
        'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState);
    [next_position_s1, next_position_l1] = getNextPosition(node, -i, 0, TimeResolution);

    newNode1.egoFrenetState = newNode1.egoFrenetState + [next_position_s1, (-i) * TimeResolution , -i, next_position_l1, 0, 0];
    newNode1.state = frenet2global(refPath, newNode1.egoFrenetState);
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode1.index;
    Tree{numel(Tree) + 1} = newNode1;

    %加速
    newNode2 = struct('state', node.state, 'time', node.time + TimeResolution, ...
        'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState);
    [next_position_s2, next_position_l2] = getNextPosition(node, i, 0, TimeResolution);

    newNode2.egoFrenetState = newNode2.egoFrenetState + [next_position_s2, i * TimeResolution , i, next_position_l2, 0, 0];
    newNode2.state = frenet2global(refPath, newNode2.egoFrenetState);
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode2.index;
    Tree{numel(Tree) + 1} = newNode2;

    %匀速
    newNode3 = struct('state', node.state, 'time', node.time + TimeResolution, ...
        'children', [0], 'visits', 0, 'score', 0, 'index', numel(Tree) + 1, 'parent', node.index, 'UCB', inf, 'egoFrenetState', node.egoFrenetState);
    [next_position_s3, next_position_l3] = getNextPosition(node, 0, 0, TimeResolution);

    newNode3.egoFrenetState = newNode3.egoFrenetState + [next_position_s3, 0 , 0, next_position_l3, 0, 0];
    newNode3.state = frenet2global(refPath, newNode3.egoFrenetState);
    Tree{node.index}.children(numel(Tree{node.index}.children) + 1) = newNode3.index;
    Tree{numel(Tree) + 1} = newNode3;
    

end
Tree_ = Tree;
end

function newNode = selection(node, Tree)
% 根据UCB选择最好的子节点
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

function reward = roll_out(node, MaxTimeHorizon, TimeResolution, predicted, egoVehicle, accMax, refPath)
% 这个函数将执行模拟
reward = 0;  % 初始奖励
currNode = node;
% termninal state就是 currTime > MaxTimeHorizon 或者碰撞了
while ~checkCollision(currNode, predicted(int32(currNode.time / TimeResolution)), egoVehicle) && currNode.time < MaxTimeHorizon
    randomNum = rand();
    randomAcc = randi([0, accMax]);
    if randomNum <= 0.20

        % [nextPositionS, nextPositionL] = getNextPosition(node, accelerationS, accelerationL, TimeResolution)
        [slowDownS1, slowDownL1] = getNextPosition(currNode, -randomAcc, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parent', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [slowDownS1, (-randomAcc) * TimeResolution , -randomAcc, slowDownL1, 0, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        reward = reward + 0.5;
    elseif randomNum >= 0.80
        [speedUpS1, speedUpL1] = getNextPosition(currNode, randomAcc, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parent', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [speedUpS1, (randomAcc) * TimeResolution , randomAcc, speedUpL1, 0, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        reward = reward + 0.5;
    else
        [keepSpeedS1, keepSpeedL1] = getNextPosition(currNode, 0, 0, TimeResolution);

        newNode = struct('state', currNode.state, 'time', currNode.time + ...
            TimeResolution, 'children', [0], 'visits', 0, 'score', 0, 'parent', 0, 'UCB', 0, 'egoFrenetState', currNode.egoFrenetState);
        newNode.egoFrenetState = newNode.egoFrenetState + [keepSpeedS1, 0 , 0, keepSpeedL1, 0, 0];
        newNode.state = frenet2global(refPath, newNode.egoFrenetState);
        currNode = newNode;
        reward = reward + 2.0;
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

% function reward = rewardFunction(state, acc, LANE_CENTER)
%     lane_deviation = abs(state - LANE_CENTER);
%     comfort = abs(acc);  % 驾驶员舒适度，假设与加速度的绝对值相关
% 
%     % 奖励函数是车道偏离程度和驾驶员舒适度的负权重和
%     reward = -0.5 * lane_deviation - 0.5 * comfort;
% end