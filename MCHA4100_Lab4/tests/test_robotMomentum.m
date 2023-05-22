%% Main function to generate tests
function tests = test_robotMomentum
tests = functiontests(localfunctions);
end

%% Test Functions
function testDontInvertMatrix(testCase)
    param = robotParameters();
    pr = [1;2];
    w = warning;
    warning('off','MATLAB:dispatcher:nameConflict')
    addpath('./mocks/','-begin');
    p = robotMomentum(pr,param);
    rmpath('./mocks/');
    warning(w);
    % No assertions needed since errors in the exploding fakes will fail the test
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