function startEgoState = setStartEgoState(egoWaypoints, initialVelocity, initialAcc)
% set the starter state for the egoVehicle.
startEgoState = [egoWaypoints(1, 1:2) 0 0 initialVelocity initialAcc];
end

