%% Main function to generate tests
function tests = test_rigidBodyMassMatrix
tests = functiontests(localfunctions);
end

%% Test Functions
function testMassMatrixSymmetric(testCase)
    param.m = 42; 
    param.IBb = diag([4;5;6]);
    param.rCBb = [1;2;3]/100;
    MRB = rigidBodyMassMatrix(param);
    actual = MRB.';
    expected = MRB;
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testMassMatrixPositiveSemidefinite(testCase)
    param.m = 42; 
    param.IBb = diag([4;5;6]);
    param.rCBb = [1;2;3]/100;
    MRB = rigidBodyMassMatrix(param);
    actual = min(eig( (MRB + MRB.')/2 ));
    floor = -1e-10;
    assertGreaterThanOrEqual(testCase, actual, floor);
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