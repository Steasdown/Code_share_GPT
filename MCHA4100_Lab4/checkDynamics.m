% Robot parameters
param = robotParameters();

% Simulation parameters
param.inputDisturbance = false;
param.dt    = 0.1;                  % Evaluation time interval (simulation may internally use smaller steps) [s]
param.T     = 15;         	        % Total simulation time [s]
tHist       = 0:param.dt:param.T;   % Specify times the output is returned
options     = odeset('MaxStep',0.01);

% Set initial conditions
x0 = robotInitialConditions(param);

% Run test scenario
fprintf('Simulating test scenario...');
[~,xHistRef] = ode45(@(t,x) robotDynamicsOracleAdapter(t,x,inputVoltage(t,x),param),tHist,x0,options);
xHistRef = xHistRef.';

[~,xHistMeas] = ode45(@(t,x) robotDynamics(t,x,inputVoltage(t,x),param),tHist,x0,options);
xHistMeas = xHistMeas.';
fprintf('done\n');

seHist = sum((xHistMeas(3:4,:) - xHistRef(3:4,:)).^2,1); % (N - Nref)^2 + (E - Eref)^2
rmsError = realsqrt(trapz(tHist,seHist)/param.T);
finalError = realsqrt(seHist(end));

fprintf('\tRMS position error: %.3g metres\n', rmsError);
fprintf('\tFinal position error: %.3g metres\n\n', finalError);

%% Check performance
x = [1;1;1;1;1];
u = [1;1];
param = robotParameters();

fprintf('Benchmarking performance...');
tSlow = 0;
tFast = 0;
tMeas = 0;
for i = 1:100
    tSlow = tSlow + timeit(@() robotDynamicsOracleSlow(0, x, u, param));
    tFast = tFast + timeit(@() robotDynamicsOracleFast(0, x, u, param));
    tMeas = tMeas + timeit(@() robotDynamics(0, x, u, param));
end
fprintf('done\n');

performance = 100*(tSlow - tMeas)/(tSlow - tFast);
fprintf('\trobotDynamics performance score: %.1f%%\n', performance);
