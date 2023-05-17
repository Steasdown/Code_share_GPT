function param = controlParameters(scenario)

if nargin < 1
    scenario = 'track';
end


switch scenario
    case 'track'
        % Horizons
        param.nControlHorizon       = 8;            % Length of control horizon
        param.nPredictionHorizon    = 8;            % Length of prediction horizon
        % Cost penalties track - No State estimator
%         param.qr                    = 1000;            % Position error penalty
%         param.qpsi                  = 10;            % Heading error penalty
%         param.ru                    = 0.005;            % Actuator penalty

        % Cost penalties track - State estimator UKF
%         param.qr                    = 7000;            % Position error penalty
%         param.qpsi                  = 1450;            % Heading error penalty
%         param.ru                    = 1.5;            % Actuator penalty
        % Cost penalties track - State estimator EKF
        param.qr                    = 7000;            % Position error penalty
        param.qpsi                  = 1450;            % Heading error penalty
        param.ru                    = 1.3;            % Actuator penalty

    case 'dock'
        % Horizons
        param.nControlHorizon       = 10;            % Length of control horizon
        param.nPredictionHorizon    = 15;            % Length of prediction horizon
%         % Cost penalties dock - No State estimator
%         param.qr                    = 200;            % Position error penalty
%         param.qpsi                  = 17;            % Heading error penalty
%         param.ru                    = 0.0001;            % Actuator penalty

        % Cost penalties dock - State estimator UKF
%         param.qr                    = 700;            % Position error penalty
%         param.qpsi                  = 20;            % Heading error penalty
%         param.ru                    = 0.05;            % Actuator penalty
        % Cost penalties dock - State estimator EKF
        param.qr                    = 5000;            % Position error penalty
        param.qpsi                  = 100;            % Heading error penalty
        param.ru                    = 5;            % Actuator penalty

    otherwise
        error('Unknown mode');
end
param.nSubsteps             = 1;            % Number of RK substeps per time step

% Actuator bounds
param.uLowerBound           = [-6; -6];     % Lower actuator bounds at each time step
param.uUpperBound           = [ 6;  6];     % Upper actuator bounds at each time step

