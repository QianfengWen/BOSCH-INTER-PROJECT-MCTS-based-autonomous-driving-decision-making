function cost = helperCalculateTrajectoryCosts(frenetTrajectories, Pc, smax)
% Calculate cost for each trajectory.
%
% frenetTrajectories - Array of trajectories in Frenet coordinates
% Pc - Probability of collision for each trajectory calculated by validator

n = numel(frenetTrajectories);
Jd = zeros(n,1);
Js = zeros(n,1);
s = zeros(n,1);

for i = 1:n
    % Time
    time = frenetTrajectories(i).Times;
    
    % resolution
    dT = time(2) - time(1);
    
    % Jerk along the path
    dds = frenetTrajectories(i).Trajectory(:,3);
    Js(i) = sum(gradient(dds,time).^2)*dT;
    
    % Jerk perpendicular to path
    % d2L/dt2 = d/dt(dL/ds*ds/dt)
    ds = frenetTrajectories(i).Trajectory(:,2);
    ddL = frenetTrajectories(i).Trajectory(:,6).*(ds.^2) + frenetTrajectories(i).Trajectory(:,5).*dds;
    Jd(i) = sum(gradient(ddL,time).^2)*dT;
    
    s(i) = frenetTrajectories(i).Trajectory(end,2);
end

cost = Js + Jd + 1000*Pc(:) + 100*(s - smax).^2;

end