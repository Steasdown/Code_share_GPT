%% Main function to generate tests
function tests = test_trajectory
tests = functiontests(localfunctions);
end

%% Test Functions

function testLowerBoundaryCondition(testCase)
    t = testCase.TestData.traj.t(1);
    expected = testCase.TestData.traj.X(:,:,1);
    actual = trajectoryEval(testCase.TestData.traj, t);
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-10, ...
        'Expected q(t0) = q0, dq(t0) = dq0, and ddq(t0) = ddq0');
end

function testUpperBoundaryCondition(testCase)
    t = testCase.TestData.traj.t(2);
    expected = testCase.TestData.traj.X(:,:,2);
    actual = trajectoryEval(testCase.TestData.traj, t);
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-10, ...
        'Expected q(t1) = q1, dq(t1) = dq1, and ddq(t1) = ddq1');
end

function testOutOfLowerBound(testCase)
    t = testCase.TestData.traj.t(1) - 1.0;
    expected = [10 0 0; 10 0 0];
    actual = trajectoryEval(testCase.TestData.traj, t);
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-10, ...
        'Expected q(t) = q0, dq(t) = 0 and ddq(t) = 0 if t < t0');
end

function testOutOfUpperBound(testCase)
    t = testCase.TestData.traj.t(2) + 1.0;
    expected = [90 0 0; 90 0 0];
    actual = trajectoryEval(testCase.TestData.traj, t);
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-10, ...
        'Expected q(t) = q1, dq(t) = 0 and ddq(t) = 0 if t > t1');
end

function testInternalPoint(testCase)
    t = 2;
    expected = [50.0125, 74.125, -0.05; 50.0125, 74.125, -0.05];
    actual = trajectoryEval(testCase.TestData.traj, t);
    assertEqual(testCase, actual, expected, 'AbsTol', 1e-10);
end

%% Optional file fixtures
function setupOnce(testCase)  % do not change function name
    addpath ../
    t = [1 3];
    X = cat(3, ...
        [10 1 0.1; 10 1 0.1], ...
        [90 1 0.1; 90 1 0.1]);
    
    testCase.TestData.traj = generateTrajectoryFromWaypoints(t, X);
end

function teardownOnce(testCase)  % do not change function name
    rmpath ../
end

%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end
