%% Main function to generate tests
function tests = test_errorMPCJacobian
tests = functiontests(localfunctions);
end

%% Test Functions

function testerrorMPCJacobian(testCase)
t1 = 0.1;
x1 = [1; 1; 1; 1; 1];
u0 = [.1; .1];
param = robotParameters();
param.dt = 0.1;
param.ctrl = controlParameters();
param.ctrl.nControlHorizon = 3;
param.ctrl.nPredictionHorizon = 5;
U = 0.1*ones(2*param.ctrl.nControlHorizon,1);
U(3:end) = 0;

load('traj.mat','traj');
param.traj = traj;

[~,actual] = errorMPC(t1, x1, u0, U, param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(U) errorMPC(t1, x1, u0, U, param), U);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err, 1e-12), ...
    'Expected Jacobian to agree with numerical solution');
end

%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    addpath ../
end

function teardownOnce(testCase)  % do not change function name
    rmpath ../
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end
