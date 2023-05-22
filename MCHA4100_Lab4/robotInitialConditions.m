function x0 = robotInitialConditions(param)

switch param.scenario
    case 'track'
        x0 = [0; 0; 1; 15; 0];
    case 'dock'
        x0 = [0; 0; 0.65; 3.3; 0];
    otherwise
        error('Unknown scenario')
end

