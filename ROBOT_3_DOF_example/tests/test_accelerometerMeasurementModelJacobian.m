%% Main function to generate tests
function tests = test_accelerometerMeasurementModelJacobian
tests = functiontests(localfunctions);
end

%% Test Functions
function testaccelerometerMeasurementModelJacobian(testCase)
x = [1;1;1;1;1;0];
u = [1;1];
param = robotParameters();
[~,~,actual] = accelerometerMeasurementModel(x,u,param);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(x) accelerometerMeasurementModel(x,u,param), x);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err,1e-10), ...
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