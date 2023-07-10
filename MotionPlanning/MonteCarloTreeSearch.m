classdef MonteCarloTreeSearch
    properties
        VehicleDimensions
        FutureCostMaps
        RectangularProfile
    end
    
    methods
        function obj = MonteCarloTreeSearch(vehicleDimensions, futureCostMaps, rectangularProfile)
            obj.VehicleDimensions = vehicleDimensions;
            obj.FutureCostMaps = futureCostMaps;
            obj.RectangularProfile = rectangularProfile;
        end
        
        function finalTrajectory = generateTrajectory(obj, currentEgoState, timeSteps, numIterations)
            % Generate trajectory using Monte Carlo Tree Search
            
            % Create the MCTS tree
            tree = MctsTree(obj.VehicleDimensions, currentEgoState, 5, 2);
            
            % Perform MCTS iterations
            node = tree.mcts(numIterations);
            trajectory = obj.assembleTrajectory(node, currentEgoState);
            cost = tree.computeReward;
            
            % Get the best trajectory
            finalTrajectory = obj.assembleTrajectory(tree.root.children(end), currentEgoState);
        end
        
        function trajectory = assembleTrajectory(obj, node, currentEgoState)
            % Assemble trajectory from the MCTS tree node
            
            % Initialize the trajectory
            trajectory = struct('Trajectory', [], 'Time', []);
            
            % Traverse the tree nodes to assemble the trajectory
            while ~isempty(node.parent)
                state = node.state;
                time = node.curr_time_state;
                trajectory.Trajectory = [state; trajectory.Trajectory];  % Updated to include all state variables
                trajectory.Time = [time; trajectory.Time];
                node = node.parent;
            end
            
            % Add the current ego state to the trajectory
            trajectory.Trajectory = [currentEgoState; trajectory.Trajectory];  % Updated to include all state variables
            trajectory.Time = [0; trajectory.Time];
        end
    end
        
end