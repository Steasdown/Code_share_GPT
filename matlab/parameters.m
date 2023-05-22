% parameters.m
function params = parameters()
% Parameter settings.

% Body properties
params.mass = []; % Mass of the boat (kg)
params.dim = [NaN, NaN, NaN]; % Dimensions of the boat [length, width, height] (m)
params.I = [NaN, NaN, NaN]; % Moments of inertia [Ixx, Iyy, Izz] (kg.m^2)
params.cog = [NaN, NaN, NaN]; % Center of gravity [x, y, z] in body-fixed frame (m)

% Actuator properties
params.actuator1.position = [NaN, NaN, NaN]; % Position of actuator 1 [x, y, z] in body-fixed frame (m)
params.actuator2.position = [NaN, NaN, NaN]; % Position of actuator 2 [x, y, z] in body-fixed frame (m)

% Sensor properties
params.gyro.position = [NaN, NaN, NaN]; % Position of gyroscope [x, y, z] in body-fixed frame (m)
params.accelerometer.position = [NaN, NaN, NaN]; % Position of accelerometer [x, y, z] in body-fixed frame (m)
params.lidar.position = [NaN, NaN, NaN]; % Position of LIDAR [x, y, z] in body-fixed frame (m)

end