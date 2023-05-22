function traj = trajectoryDesign(param)

% Generate waypoint intercept times and position/velocity/acceleration data
[t, X] = generateWaypoints(param);

% Generate trajectory structure from waypoint data
traj = generateTrajectoryFromWaypoints(t, X);
