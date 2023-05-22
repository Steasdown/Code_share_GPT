function [t, X] = generateWaypoints(param)

nWaypoints = 17;
t = zeros(1, nWaypoints);
X = zeros(2, 2, nWaypoints);

% Waypoint 1 - start position
t(1) = 0;
X(:, :, 1) = [ ...
    1.1, 0;
    15, 0];

% Waypoint 2 - Orientate north and speed up from start pos
t(2) = 0.8;
X(:, :, 2) = [ ...
    1.5, 0.75;
    15, 0.05];

% Waypoint 3 - start of turn 1 to face W
t(3) = 3.45;
X(:, :, 3) = [ ...
    6, 0.75;
    15.1, -0.38];

% Waypoint 4 - end turn 1
t(4) = 4;
X(:, :, 4) = [ ...
    6.5, 0.42;
    14.65, -0.7];

% Waypoint 5 - start turn 2 to face S
t(5) = 6.5;
X(:, :, 5) = [ ...
    6.5, -0.45;
    12, -0.8];

% Waypoint 6 - end turn 2 
t(6) = 7.25;
X(:, :, 6) = [ ...
    6, -0.8;
    11.5, -0.4];

% Waypoint 7 - start turn 3 to face E
t(7) = 11;
X(:, :, 7) = [ ...
    1.4, -0.625;
    11.65, 0.25];

% Waypoint 8 - end turn 3
t(8) = 11.75;
X(:, :, 8) = [ ...
    1, -0.55;
    11.9, 0.45];


% Waypoint 9 - turn 4 start rotating toward NNW
t(9) = 13.75;
X(:, :, 9) = [ ...
    0.83, 0.325;
    13.3, 0.325];


% Waypoint 10 - end turn 4 
t(10) = 15.4;
X(:, :, 10) = [ ...
    1.45, 0.325;
    13.3, -0.325];


% Waypoint 11 - Adjust course to head N
t(11) = 17;
X(:, :, 11) = [ ...
    2.1, 0.775;
    11.85, -0.25];

% Waypoint 12 - start turn 6 to head W 
t(12) = 20.25;
X(:, :, 12) = [ ...
    6.1,    0.775;
    11.25, -0.455];

% Waypoint 13 - end turn 6
t(13) = 21;
X(:, :, 13) = [ ...
    6.65, 0.5;
    10.7, -0.8];

% Waypoint 14 - start turn 7 to head S towards charging pad
t(14) = 25.25;
X(:, :, 14) = [ ...
    6.85, -0.4;
    4.35, -0.75];

% Waypoint 15 - end turn 7
t(15) = 26.25;
X(:, :, 15) = [ ...
    6.22, -0.75;
    3.72, -0.45];

% Waypoint 16 - adjust course to head south
t(16) = 27.15;
X(:, :, 16) = [ ...
   5.45, -0.75;
    3.5, -0.1];

% Waypoint 17 - stopping at bin
t(17) = 30;
X(:, :, 17) = [ ...
    0.75, 0;
    3.3, 0];

