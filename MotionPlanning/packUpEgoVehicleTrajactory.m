function egoVehicleTraj = packUpEgoVehicleTrajactory(node, nextNode, TimeResolution, refPath)
waypoints = [node.state(1:3); nextNode.state(1:3)];
jerk = (nextNode.state(end) - node.state(end)) / TimeResolution;
if waypoints(1,:) ~= waypoints(2,:)
    currNode = node;
    egoVehicleTraj = currNode.state;
    for i = 1: int32(TimeResolution / 0.2)
        currFrenetState = currNode.egoFrenetState;
        [displacementS, deltaSpeedS, displacementL, deltaSpeedL] = getDisplacement(currNode, jerk, nextNode.egoFrenetState(4) - node.egoFrenetState(4), 0.2);
        nextFrenetState = currFrenetState + [displacementS, deltaSpeedS , jerk * 0.2, double(displacementL / int32(TimeResolution / 0.2)), deltaSpeedL, 0];
        nextglobState = frenet2global(refPath, nextFrenetState);
        egoVehicleTraj = [egoVehicleTraj; nextglobState];
        currNode.state = nextglobState;
        currNode.egoFrenetState = nextFrenetState;
    end
else
    egoVehicleTraj  = repmat(node.state, int32(TimeResolution / 0.2));
end

end
