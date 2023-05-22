function param = controlParameters(scenario)

if nargin < 1
    scenario = 'track';
end


switch scenario
    case 'track'
        % Horizons
        param.nControlHorizon       = 10;            % Length of control horizon
        param.nPredictionHorizon    = 10;            % Length of prediction horizon
        % Cost penalties track
        param.qr                    = 1000;            % Position error penalty
        param.qpsi                  = 10;            % Heading error penalty
        param.ru                    = 0.005;            % Actuator penalty

    case 'dock'
        % Horizons
        param.nControlHorizon       = 10;            % Length of control horizon
        param.nPredictionHorizon    = 15;            % Length of prediction horizon
        % Cost penalties dock
        param.qr                    = 200;            % Position error penalty
        param.qpsi                  = 17;            % Heading error penalty
        param.ru                    = 0.0001;            % Actuator penalty

    otherwise
        error('Unknown mode');
end
param.nSubsteps             = 1;            % Number of RK substeps per time step

% Actuator bounds
param.uLowerBound           = [-6; -6];     % Lower actuator bounds at each time step
param.uUpperBound           = [ 6;  6];     % Upper actuator bounds at each time step

