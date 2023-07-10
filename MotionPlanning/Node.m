classdef Node
    properties
        egoCarProperties
        state
        currTimeState
        parent
        children
        score
        visits
        UCB
    end
    
    methods
        function obj = Node(egoCarProperties, state, currTimeState, parent, visits)
            obj.egoCarProperties = egoCarProperties;
            obj.state = state;
            obj.currTimeState = currTimeState;
            obj.parent = parent;
            obj.children = [];
            obj.score = 0.0;
            obj.visits = visits;
            obj.UCB = 0.0;
        end
        
        function addChild(obj, child)
            obj.children(nnz(obj.children) + 1) = child;
        end
        
        function update(obj, score)
            obj.score = obj.score + score;
            obj.visits = obj.visits + 1;
        end
        
        function newNode = copyLeaf(obj)
            newNode = Node(obj.egoCarProperties, obj.state, obj.currTimeState, obj.parent, obj.visits);
            newNode.score = obj.score;
        end

    end
end

