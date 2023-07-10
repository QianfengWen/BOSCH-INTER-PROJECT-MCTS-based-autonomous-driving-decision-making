function [frenetTrajectory, globalTrajectory] = helperGenerateTrajectoryInMCTS(connector, refPath, currentEgoState, speedLimit, laneWidth, intersectionS, intersectionBuffer)

% Convert current global state of ego to frenet coordinate
egoFrenetState = global2frenet(refPath, currentEgoState);

% Create a node for the current state
currentNode = createNode(egoFrenetState);

% Perform MCTS
for i = 1:numIterations
    % Select a node to explore
    selectedNode = treePolicy(currentNode);

    % Simulate a trajectory from the selected node
    [finalFrenetState, reward] = defaultPolicy(selectedNode);

    % Update the values of nodes based on the simulated reward
    backup(selectedNode, reward);
end

% Choose the best action based on the updated values
bestNode = bestChild(currentNode);

% Connect the trajectories and obtain end state in global coordinates
[frenetTrajectory,globalTrajectory] = connect(connector,egoFrenetState,bestNode.frenetState,bestNode.timeStamp);

end

function node = createNode(frenetState)
    node.frenetState = frenetState;
    node.numVisits = 0;
    node.totalReward = 0;
    node.children = [];
end

function selectedNode = treePolicy(node)
    % Your code here: Choose a node to explore, balancing exploration and exploitation
end

function [finalFrenetState, reward] = defaultPolicy(node)
    % Your code here: Simulate a trajectory from the node and compute the resulting reward
end

function backup(node, reward)
    % Your code here: Update the values of nodes based on the simulated reward
end

function bestNode = bestChild(node)
    % Your code here: Choose the best child node based on the updated values
end
