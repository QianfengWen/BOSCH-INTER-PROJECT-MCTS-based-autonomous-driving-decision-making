classdef MctsTree
    properties
        root
        accMax
        accSpeed
        MaxTimeHorizon
        TimeResolution
    end
    
    methods
        function obj = MctsTree(egoCarProperties, rootState, accMax, accSpeed)
            obj.root = Node(egoCarProperties, rootState, 0.0, [], 1);
            obj.accMax = accMax;
            obj.accSpeed = accSpeed;
            obj.MaxTimeHorizon = 8.0;
            obj.TimeResolution = 0.1;
        end
        
        function nextNode = mcts(obj, numIterations)
            for i = 1:numIterations
                currNode = obj.root;
                while ~obj.isTerminal(currNode)
                    if isempty(currNode.children)
                        if currNode.visits == 0
                            reward = obj.rollOut(currNode);
                            obj.backPropagation(currNode, reward);
                            continue;
                        else
                            obj.expand(currNode);
                            continue;
                        end
                    else
                        currNode = obj.selection(currNode);
                    end
                end
            end
            
            obj.root.children = sort(obj.root.children, 'descend');
            nextNode = obj.root.children(end);
        end
        
        function selectedNode = selection(obj, node)
            nextNode = node.children(end);
            for i = 1:numel(node.children)
                child = node.children(i);
                if child.UCB >= nextNode.UCB
                    nextNode = child;
                end
            end
            selectedNode = nextNode;
        end
        
        function expand(obj, node)
            for i = 0:2:obj.accMax
                slowDownSL = obj.getNextPosition(node, -i, 0);
                newNode = Node(node.egoCarProperties, ["slow_down", [node.state[1][0] - i * self.TimeResolution,
                                                        i, 0, 0, slowDownSL]], node.currTimeState + obj.TimeResolution; node);
                node.addChild(newNode);
                
                speedUpSL = obj.getNextPosition(node, i, 0);
                newNode = Node(node.egoCarProperties, ["speed_up", [node.state[1][0] + i * self.TimeResolution,
                                                        i, 0, 0, speedUpSL]], node.currTimeState + obj.TimeResolution, node);
                node.addChild(newNode);
                
                keepSpeedSL = obj.getNextPosition(node, 0, 0);
                newNode = Node(node.egoCarProperties, ["keep_speed", [node.state[1][0],
                                                        i, 0, 0, keepSpeedSL]], node.currTimeState + obj.TimeResolution, node);
                node.addChild(newNode);
            end
        end
        
        function reward = rollOut(obj, node)
            startNode = node.copyLeaf();
            currNode = startNode;
            reward = 0.0;
            while ~obj.isTerminal(currNode) && ~obj.checkCollision(currNode) && currNode.currTimeState < obj.MaxTimeHorizon
                randomNum = rand();
                randomAcc = randi([0, obj.accMax]);
                if randomNum <= 0.20
                    slowDownSL = obj.getNextPosition(node, -randomAcc, 0);
                    newNode = Node(currNode.egoCarProperties, ["slow_down", [node.state(2)(1) - i * self.TimeResolution,
                                                        randomAcc, 0, 0, slowDownSL]], currNode.currTimeState + obj.TimeResolution, currNode);
                    reward = reward + obj.computeReward(newNode);
                    currNode = newNode;
                elseif randomNum >= 0.80
                    newNode = Node(currNode.egoCarProperties, ["speed_up", [node.state[1][0] + i * self.TimeResolution,
                                                        i, 0, 0, slowDownSL]], currNode.currTimeState + obj.TimeResolution, currNode);
                    reward = reward + obj.computeReward(newNode);
                    currNode = newNode;
                else
                    newNode = Node(currNode.egoCarProperties, ["keep_speed", [node.state[1][0],
                                                        i, 0, 0, keepSpeedSL]], currNode.currTimeState + obj.TimeResolution, currNode);
                    reward = reward + obj.computeReward(newNode);
                    currNode = newNode;
                end
            end
        end
        
        function backPropagation(obj, node, reward)
            while ~isempty(node)
                node.update(reward);
                node = node.parent;
            end
        end
        
        function nextSL = getNextPosition(obj, node, accelerationS, accelerationL)
            deltaT = obj.TimeResolution;
            nextPositionS = node.state{2}(1) * deltaT + 0.5 * accelerationS * deltaT^2;
            nextPositionL = node.state{2}(2) * deltaT + 0.5 * accelerationL * deltaT^2;
            nextSL = [nextPositionS, nextPositionL];
        end
        
        function terminal = isTerminal(obj, node)
            terminal = node.currTimeState >= obj.MaxTimeHorizon;
        end
        
        function collision = checkCollision(obj, node)
            collision = false;  % Implement collision check logic
        end
        
        function reward = computeReward(obj, node)
            % Implement reward calculation logic
            if node.state(1) == "keep_speed"
                reward = 1.0;
            elseif node.state(1) == "slow_down"
                reward = -0.5;  
            else
                reward = -0.5;
            end
            if self.check_collision(node)
                reward = -10.0;  
            elseif self.check_sudden_stop(node)
                reward = -2.0;  
            end
        end
    end
end
