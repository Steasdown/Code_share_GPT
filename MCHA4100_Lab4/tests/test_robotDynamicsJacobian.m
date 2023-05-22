%% Main function to generate tests
function tests = test_robotDynamicsJacobian
tests = functiontests(localfunctions);
end

%% Test Functions

function testRobotDynamicsJacobianInput(testCase)
t = 0;
x = [1; 1; 1; 1; 1];
u = [1; 1];
param = robotParameters();
[~, ~, actual] = robotDynamics(t, x, u, param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(u) robotDynamics(t, x, u, param), u);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err, 1e-12), ...
    'Expected input Jacobian to agree with numerical solution');
end

function testRobotDynamicsJacobianState(testCase)
t = 0;
x = [1; 1; 1; 1; 1];
u = [1; 1];
param = robotParameters();
[~, actual, ~] = robotDynamics(t, x, u, param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(x) robotDynamics(t, x, u, param), x);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err, 1e-12), ...
    'Expected state Jacobian to agree with numerical solution');
end

function testRobotDynamicsJacobianStateZeroMomentum(testCase)
t = 0;
x = [0; 0; 1; 1; 1];
u = [1; 1];
param = robotParameters();
[~, actual, ~] = robotDynamics(t, x, u, param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(x) robotDynamics(t, x, u, param), x);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err, 1e-12), ...
    'Expected input Jacobian to agree with numerical solution');
end

function testRobotDynamicsJacobianStateSmallMomentum(testCase)
t = 0;
x = [0.01; 0; 0.04; 0; 0];
u = [0; 0];
param = robotParameters();
[~, actual, ~] = robotDynamics(t, x, u, param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(x) robotDynamics(t, x, u, param), x);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err, 1e-12), ...
    'Expected input Jacobian to agree with numerical solution');
end

function testRobotDynamicsJacobianStateSmallMomentum2(testCase)
t = 0;
x = [0.07; 0; 0.04; 0; 0];
u = [0;0];
param = robotParameters();
[~, actual, ~] = robotDynamics(t, x, u, param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(x) robotDynamics(t, x, u, param), x);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err, 1e-12), ...
    'Expected input Jacobian to agree with numerical solution');
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
