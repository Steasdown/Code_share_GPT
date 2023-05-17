% Ensure no unit tests fail before continuing
results = runtests({ ...
    'tests/test_rigidBodyMassMatrix.m', ...
    'tests/test_robotConstraints.m', ...
    'tests/test_trajectory.m', ...
    });
assert(~any([results.Failed]));

%%
% Robot parameters
param = robotParameters();

% Scenario
%param.scenario = 'track';
param.scenario = 'dock';

% Run state estimator
runStateEstimator = true;
%runStateEstimator = false;
% Use state estimate for controller (true) instead of simulation state (false)
useStateEstimate = true; 
%useStateEstimate = false; 
% Use unscented transform or linearisation for estimator
transformMethod = 'affine';         % Use linearisation to implement EKF
%transformMethod = 'unscented';      % Use unscented transform to implement UKF

if strcmp(transformMethod, 'affine')
    results = runtests({ ...
        'tests/test_robotDynamicsJacobian.m', ...
        'tests/test_accelerometerMeasurementModelJacobian.m', ...
        'tests/test_beaconMeasurementModelJacobian.m', ...
        'tests/test_gyroscopeMeasurementModelJacobian.m', ...
        });
    assert(~any([results.Failed]));
end

% Controller parameters
param.ctrl = controlParameters(param.scenario);

% Use gradients to speed up optimisation
param.useGradientOpt = true;
if param.useGradientOpt
    switch param.scenario
        case 'track'
            results = runtests({ ...
                'tests/test_robotDynamicsJacobian.m', ...
                'tests/test_errorMPCJacobian.m', ...
                });
        case 'dock'
            results = runtests({ ...
                'tests/test_robotDynamicsJacobian.m', ...
                'tests/test_costMPCGradient.m', ...
                'tests/test_nonlconMPCJacobian.m', ...
                });
        otherwise
            error('Unknown scenario');
    end
    assert(~any([results.Failed]));
end

% Load habitat map
load('map.mat', 'map', 'Egridlines', 'Ngridlines');
param.map = map;
param.Egridlines = Egridlines;
param.Ngridlines = Ngridlines;

% Generate trajectory
if strcmp(param.scenario, 'track')
    param.traj = trajectoryDesign(param);
end

% Simulation parameters
switch param.scenario
    case 'track'
        param.dt    = 0.1;      	% Evaluation time interval (simulation may internally use smaller steps) [s]
        param.T     = 30;        	% Total simulation time [s]
    case 'dock'
        param.dt    = 0.1;      	% Evaluation time interval (simulation may internally use smaller steps) [s]
        param.T     = 6;      	    % Total simulation time [s]
    otherwise
        error('Unknown scenario');
end

options     = odeset('MaxStep', 0.005, 'Events', @(t, x) eventCollision(t, x, param));
tHist       = 0:param.dt:param.T;       % Specify times the output is returned
uHist       = nan(2, length(tHist));

% Set initial conditions
x0 = robotInitialConditions(param);

% Set initial state estimate
[mu0, S0] = robotInitialState(param);

% Run the simulation
param.inputDisturbance = true;
U = [];
u = [0; 0];                          % Initial stored control action
uHist(:, 1) = u;
xHist = nan(6, length(tHist));
xHist(1:5, 1) = x0;
xHist(6, :) = 0.1;   % true gyro bias
muHist = nan(6, length(tHist));
muHist(:, 1) = mu0;
SHist = nan(6, 6, length(tHist));
SHist(:, :, 1) = S0;

delete(findall(0, 'tag', 'TMWWaitbar'));  % Remove any stuck waitbars
wh = waitbar(0, getStatusMsg, ...
    'Name', 'MCHA4100 motivational waitbar', ...
    'CreateCancelBtn', 'setappdata(gcbf, ''cancelling'', 1)');
for k = 1:length(tHist)-1
    if getappdata(wh, 'cancelling')  % Check if waitbar cancel button clicked
        delete(wh);
        error('User cancelled operation.')
    end

    try % you can't succeed if you don't
        % Apply stored control action u
        % PRETENDING

        % Simulate one time step with ZOH input u
        func = @(t, x) robotDynamicsOracleAdapter(t, x, u, param);
        [~, xTemp, te, xe, ie] = ode45(func, [tHist(k) tHist(k+1)], xHist(1:5, k), options);
        xTemp = xTemp.';
        xHist(1:5, k+1) = xTemp(:, end);
        if ~isempty(te)
            warning('Collision detected');
            uHist(:, k+2:end) = [];
            xHist(:, k+2:end) = [];
            tHist(:, k+2:end) = [];
            muHist(:, k+2:end) = [];
            SHist(:, :, k+2:end) = [];
            break
        end

        if runStateEstimator
            % Take measurements from sensors (simulated)
            yacc = simulateMeasurement(@accelerometerMeasurementModel, xHist(:, k), u, param);
            ygyr = simulateMeasurement(@gyroscopeMeasurementModel, xHist(:, k), u, param);
            ybea = simulateMeasurement(@beaconMeasurementModel, xHist(:, k), u, param);
    
            % Correct state based on measurements
            [muHist(:, k), SHist(:, :, k)] = measurementUpdate(muHist(:, k), SHist(:, :, k), u, yacc, @accelerometerMeasurementModel, param, transformMethod);
            [muHist(:, k), SHist(:, :, k)] = measurementUpdate(muHist(:, k), SHist(:, :, k), u, ygyr, @gyroscopeMeasurementModel, param, transformMethod);
            [muHist(:, k), SHist(:, :, k)] = measurementUpdate(muHist(:, k), SHist(:, :, k), u, ybea, @beaconMeasurementModel, param, transformMethod);
    
            % Estimate next state
            [muHist(:, k+1), SHist(:, :, k+1)] = timeUpdateContinuous(muHist(:, k), SHist(:, :, k), u, @robotProcessModel, param, param.dt, 'RK4', transformMethod);
        end

        % Use mean estimate or true state (diagnostic only) for controller
        if useStateEstimate && runStateEstimator
            xctrl = muHist(1:5, k+1);
        else
            xctrl = xHist(1:5, k+1);
        end

        % Compute next control action
        U = controlMPC(tHist(k+1), xctrl, u, U, param);

        % Store next control action to apply
        u = U(1:2);

        uHist(:, k+1) = u; % Save for plotting
    catch hot_potato
        delete(wh);                 % Remove waitbar if error
        rethrow(hot_potato);        % Someone else's problem now
    end
    waitbar(k/length(tHist), wh);    % Update waitbar
end
delete(wh);                         % Remove waitbar if we complete successfully

%% Plot history
NHist   = xHist(3, :);
EHist   = xHist(4, :);
psiHist = xHist(5, :);
gHist   = nan(1, length(tHist));
HHist   = nan(1, length(tHist));
nu3Hist = nan(3, length(tHist));
mupHist = nan(6, length(tHist)); % mu + 1 stdev
mumHist = nan(6, length(tHist)); % mu - 1 stdev
Bc = robotConstraints(param);
for k = 1:length(tHist)
    nu3Hist(:, k) = robotVelocity(xHist(1:5, k), param);
    gHist(k) = Bc.'*nu3Hist(:, k);
    HHist(k) = robotEnergy(xHist(1:5, k), param);
    mupHist(:, k) = muHist(:, k) + realsqrt(diag(SHist(:, :, k).'*SHist(:, :, k)));
    mumHist(:, k) = muHist(:, k) - realsqrt(diag(SHist(:, :, k).'*SHist(:, :, k)));
end

if strcmp(param.scenario, 'track')
    Xtraj = nan(2, 2, length(tHist));
    for k = 1:length(tHist)
        Xtraj(:, :, k) = trajectoryEval(param.traj, tHist(k));
    end
    Ntraj = squeeze(Xtraj(1, 1, :)).';
    Etraj = squeeze(Xtraj(2, 1, :)).';
end

figure(1); clf

subplot(3, 4, 1)
hold on
plot(tHist, NHist, 'r')
plot(tHist, muHist(3, :), 'b')
plot(tHist, mupHist(3, :), 'b:')
plot(tHist, mumHist(3, :), 'b:')
hold off
grid on
legend('x', '\mu', '\mu \pm \sigma')
title('North position')
ylabel('N [m]')
subplot(3, 4, 5)
hold on
plot(tHist, EHist, 'r')
plot(tHist, muHist(4, :), 'b')
plot(tHist, mupHist(4, :), 'b:')
plot(tHist, mumHist(4, :), 'b:')
hold off
grid on
legend('x', '\mu', '\mu \pm \sigma')
title('East position')
ylabel('E [m]')
subplot(3, 4, 9)
hold on
plot(tHist, psiHist*180/pi, 'r')
plot(tHist, muHist(5, :)*180/pi, 'b')
plot(tHist, mupHist(5, :)*180/pi, 'b:')
plot(tHist, mumHist(5, :)*180/pi, 'b:')
hold off
grid on
legend('x', '\mu', '\mu \pm \sigma')
title('Yaw angle')
ylabel('\psi [\circ]')
xlabel('Time [s]')

subplot(3, 4, 2)
hold on
plot(tHist, xHist(1, :), 'r')
plot(tHist, muHist(1, :), 'b')
plot(tHist, mupHist(1, :), 'b:')
plot(tHist, mumHist(1, :), 'b:')
hold off
grid on
legend('x', '\mu', '\mu \pm \sigma')
title('Reduced momentum 1')
ylabel('p_{r,1} [-]')
subplot(3, 4, 6)
hold on
plot(tHist, xHist(2, :), 'r')
plot(tHist, muHist(2, :), 'b')
plot(tHist, mupHist(2, :), 'b:')
plot(tHist, mumHist(2, :), 'b:')
hold off
grid on
legend('x', '\mu', '\mu \pm \sigma')
title('Reduced momentum 2')
ylabel('p_{r,2} [-]')
subplot(3, 4, 10)
hold on
plot(tHist, xHist(6, :)*180/pi, 'r')
plot(tHist, muHist(6, :)*180/pi, 'b')
plot(tHist, mupHist(6, :)*180/pi, 'b:')
plot(tHist, mumHist(6, :)*180/pi, 'b:')
hold off
grid on
legend('x', '\mu', '\mu \pm \sigma')
title('Gyro bias')
ylabel('b_{gyr} [\circ/s]')
xlabel('Time [s]')

subplot(3, 4, 3)
plot(tHist, nu3Hist(1, :))
grid on
title('Surge velocity')
ylabel('u [m/s]')
subplot(3, 4, 7)
plot(tHist, nu3Hist(2, :))
grid on
title('Sway velocity')
ylabel('v [m/s]')
subplot(3, 4, 11)
plot(tHist, nu3Hist(3, :)*180/pi)
grid on
title('Yaw angular velocity')
ylabel('r [\circ/s]')
xlabel('Time [s]')

subplot(4, 4, 4)
plot(tHist, uHist(1, :))
grid on
title('Left motor voltage')
ylabel('u_L [V]')
subplot(4, 4, 8)
plot(tHist, uHist(2, :))
grid on
title('Right motor voltage')
ylabel('u_R [V]')
subplot(4, 4, 12)
plot(tHist, HHist)
grid on
title('Total energy')
ylabel('H [J]')
subplot(4, 4, 16)
plot(tHist, gHist)
grid on
title('Nonholonomic constraint violation')
ylabel('B_c^T \nu [-]')
xlabel('Time [s]')

%% Animation

outputVideo = false;

if outputVideo
    vid = VideoWriter(['lab5' param.scenario param.controller '.mp4'], 'MPEG-4');
    vid.FrameRate = 1/param.dt;
    vid.Quality = 100;
    open(vid);
end

fig     = 2;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf, 'FontSize', 14);
hold(ax, 'on');

[Egrid, Ngrid] = meshgrid(param.Egridlines, param.Ngridlines);
Cgrid = zeros(size(Egrid));
Cgrid(1:end-1, 1:end-1) = ~param.map;

colormap(ax, 'gray');
clim(ax, [0 1]);

hGrid   = pcolor(ax, Egrid, Ngrid, Cgrid);
hGrid.EdgeColor = 'none';

hDust = patch([12.5, 13, 13, 12.5],[0.5 0.5 1 1], 'red', 'EdgeColor', 'none');
if strcmp(param.scenario, 'track')
    hCharge = patch([3.05, 3.55, 3.55, 3.05], [0.5 0.5 1.0 1.0], 'red', 'EdgeColor', 'none');
else
    hCharge = patch([1.45, 1.95, 1.95, 1.45], [0.5 0.5 1.0 1.0], 'red', 'EdgeColor', 'none');
end

hLt     = plot(ax, nan(size(EHist)), nan(size(NHist)), 'r');
hRt     = plot(ax, nan(size(EHist)), nan(size(NHist)), 'g');
hSt     = plot(ax, nan(size(EHist)), nan(size(NHist)), 'b');
if strcmp(param.scenario, 'track')
    htraj   = plot(ax, Etraj, Ntraj, 'k:');
    hP      = plot(ax, nan, nan, 'ro');
end
hO      = plot(ax, nan, nan, 'k-');
hL      = plot(ax, 0, 0, 'r.');
hR      = plot(ax, 0, 0, 'g.');
hS      = plot(ax, 0, 0, 'b.');
hB      = plot(ax, 0, 0, 'k.');
hC      = plot(ax, 0, 0, 'k.');
tL      = text(ax, 0, 0, ' L', 'FontSize', 10, 'Color', 'r');
tR      = text(ax, 0, 0, ' R', 'FontSize', 10, 'Color', 'g');
tS      = text(ax, 0, 0, ' S', 'FontSize', 10, 'Color', 'b');
tB      = text(ax, 0, 0, ' B', 'FontSize', 10, 'Color', 'k');
tC      = text(ax, 0, 0, ' C', 'FontSize', 10, 'Color', 'k');

hold(ax, 'off');
axis(ax, 'equal');
if strcmp(param.scenario, 'track')
    axis(ax, [param.Egridlines(1), param.Egridlines(end), param.Ngridlines(1), param.Ngridlines(end)]);
else
    axis(ax, [param.Egridlines(1), 4.3, param.Ngridlines(1), 4]);
end
% grid(ax, 'on');
xlabel(ax, 'East [m]');
ylabel(ax, 'North [m]');

a = 0.15;
r = 0.2;
theta = linspace(3*pi/2, pi/2, 50);
rOCb = [ [0; r; 0], [a; r; 0], [a; -r; 0], [0; -r; 0], r*[cos(theta); sin(theta); zeros(size(theta))] ];
Se3  = skew([0; 0; 1]);

rBNn = nan(3, length(tHist));
rCNn = nan(3, length(tHist));
rLNn = nan(3, length(tHist));
rRNn = nan(3, length(tHist));
rSNn = nan(3, length(tHist));
rANn = nan(3, length(tHist));
for k = 1:length(tHist)
    Rnb = expm(psiHist(k)*Se3);
    rBNn(:, k) = [NHist(k); EHist(k); 0];
    rCNn(:, k) = rBNn(:, k) + Rnb*param.rCBb;
    rLNn(:, k) = rBNn(:, k) + Rnb*param.rLBb;
    rRNn(:, k) = rBNn(:, k) + Rnb*param.rRBb;
    rSNn(:, k) = rBNn(:, k) + Rnb*param.rSBb;
    
    rONn = rCNn(:, k) + Rnb*rOCb;
    hO.XData = rONn(2, :);
    hO.YData = rONn(1, :);
    
    hLt.XData = rLNn(2, :);
    hLt.YData = rLNn(1, :);
    hRt.XData = rRNn(2, :);
    hRt.YData = rRNn(1, :);
    hSt.XData = rSNn(2, :);
    hSt.YData = rSNn(1, :);
    
    if strcmp(param.scenario, 'track')
        hP.XData = Etraj(k);
        hP.YData = Ntraj(k);
    end
    
    hL.XData = rLNn(2, k);
    hL.YData = rLNn(1, k);
    tL.Position = [rLNn(2, k), rLNn(1, k), 0];
    hR.XData = rRNn(2, k);
    hR.YData = rRNn(1, k);
    tR.Position = [rRNn(2, k), rRNn(1, k), 0];
    hS.XData = rSNn(2, k);
    hS.YData = rSNn(1, k);
    tS.Position = [rSNn(2, k), rSNn(1, k), 0];
    hB.XData = rBNn(2, k);
    hB.YData = rBNn(1, k);
    tB.Position = [rBNn(2, k), rBNn(1, k), 0];
    hC.XData = rCNn(2, k);
    hC.YData = rCNn(1, k);
    tC.Position = [rCNn(2, k), rCNn(1, k), 0];
    
    if inpolygon(rCNn(2, k), rCNn(1, k), hDust.Vertices(:, 1), hDust.Vertices(:, 2))
        hDust.FaceColor = 'green';
    end
    if inpolygon(rCNn(2, k), rCNn(1, k), hCharge.Vertices(:, 1), hCharge.Vertices(:, 2))
        hCharge.FaceColor = 'green';
    end

    drawnow
    if outputVideo
        writeVideo(vid, getframe(hf));
    else
        pause(param.dt);
    end 

end

if outputVideo
    close(vid);
end

