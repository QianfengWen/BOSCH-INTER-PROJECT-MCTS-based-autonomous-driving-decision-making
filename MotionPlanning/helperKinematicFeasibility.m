function isFeasible = helperKinematicFeasibility(frenetTrajectories, speedLimit, aMax)
% Check kinematic feasibility of trajectories
%
% frenetTrajectories - Array of trajectories in Frenet coordinates
% speedLimit - Speed limit (m/s)
% aMax - Maximum acceleration (m/s^2)

isFeasible = false(numel(frenetTrajectories),1);
for i = 1:numel(frenetTrajectories)
    % Speed of the trajectory
    speed = frenetTrajectories(i).Trajectory(:,2);
    
    % Acceleration of the trajectory
    acc = frenetTrajectories(i).Trajectory(:,3);
    
    % Is speed valid?
    isSpeedValid = ~any(speed < -0.1 | speed > speedLimit + 1);
    
    % Is acceleration valid?
    isAccelerationValid = ~any(abs(acc) > aMax);
    
    % Trajectory feasible if both speed and acc valid
    isFeasible(i) = isSpeedValid & isAccelerationValid;
end

end