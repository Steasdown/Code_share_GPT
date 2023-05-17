function [mu0,S0] = robotInitialState(param)

switch param.scenario
    case 'track'
        %mu0 = [0;0;1;15;0;0];
        mu0 = [0;0;1;15;0;deg2rad(5.72)];
        %S0 = zeros(6);
        S0 = diag([.01, .5,  .001, .001, 0.5e-20, 0.01]);
    case 'dock'
        mu0 = [0;0;0.65;3.3;0;0];
        %S0 = zeros(6);
        S0 = eye(6)*0.1;
    otherwise
        error('Unknown scenario')
end
assert(isequal(S0,triu(S0)),'Expect S0 to be upper triangular');
