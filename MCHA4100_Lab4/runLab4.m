% Ensure no unit tests fail before continuing
results = runtests({ ...
    'tests/test_dampingMatrix.m', ...
    'tests/test_rigidBodyMassMatrix.m', ...
    'tests/test_robotConstraints.m', ...
    'tests/test_robotMomentum.m', ...
    'tests/test_trajectory.m', ...
    });
assert(~any([results.Failed]));

%%
% Robot parameters
param = robotParameters();

% Controller type
%param.controller = 'none';
param.controller = 'MPC';

% Scenario
param.scenario = 'track';
%param.scenario = 'dock';

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

% Run the simulation
switch param.controller
    case 'none'
        param.inputDisturbance = false;
        u = [0; 0];
        func = @(t, x) robotDynamicsOracleAdapter(t, x, u, param);
        [tHist, xHist] = ode45(func, tHist, x0, options);
        tHist = tHist.';
        xHist = xHist.';
        for k = 1:length(tHist)
            uHist(:, k) = u;
        end
        uHist(:, length(tHist)+1:end) = [];
    case 'MPC'
        param.inputDisturbance = true;          % Enable small input disturbance
        U = [];
        u = [0; 0];                              % Initial stored control action
        uHist(:, 1) = u;
        xHist = zeros(5,  length(tHist));
        xHist(:, 1) = x0;
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
                % Pretend to apply stored control action u
                % PRETENDING
        
                % Simulate one time step with ZOH input u
                func = @(t, x) robotDynamicsOracleAdapter(t, x, u, param);
                [~, xTemp, te, xe, ie] = ode45(func, [tHist(k) tHist(k+1)], xHist(:, k), options);
                xTemp = xTemp.';
                xHist(:, k+1) = xTemp(:, end);
                if ~isempty(te)
                    warning('Collision detected');
                    uHist(:, k+2:end) = [];
                    xHist(:, k+2:end) = [];
                    tHist(:, k+2:end) = [];
                    break
                end
        
                % Pretend to take measurements from sensors
                % Pretend to estimate next state
                % PRETENDING INTENSIFIES
                xctrl = xHist(:, k+1);
                
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
    otherwise
        error('Unknown controller');
end

%% Plot history
NHist   = xHist(3, :);
EHist   = xHist(4, :);
psiHist = xHist(5, :);
gHist   = nan(1, length(tHist));
HHist   = nan(1, length(tHist));
nu3Hist = nan(3, length(tHist));
Bc = robotConstraints(param);
for k = 1:length(tHist)
    nu3Hist(:, k) = robotVelocity(xHist(:, k), param);
    gHist(k) = Bc.'*nu3Hist(:, k);
    HHist(k) = robotEnergy(xHist(:, k), param);
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

subplot(3, 3, 1)
plot(tHist, NHist)
grid on
title('North position')
ylabel('N [m]')
subplot(3, 3, 4)
plot(tHist, EHist)
grid on
title('East position')
ylabel('E [m]')
subplot(3, 3, 7)
plot(tHist, psiHist*180/pi)
grid on
title('Yaw angle')
ylabel('\psi [\circ]')
xlabel('Time [s]')

subplot(3, 3, 2)
plot(tHist, nu3Hist(1, :))
grid on
title('Surge velocity')
ylabel('u [m/s]')
subplot(3, 3, 5)
plot(tHist, nu3Hist(2, :))
grid on
title('Sway velocity')
ylabel('v [m/s]')
subplot(3, 3, 8)
plot(tHist, nu3Hist(3, :)*180/pi)
grid on
title('Yaw angular velocity')
ylabel('r [\circ/s]')
xlabel('Time [s]')

subplot(4, 3, 3)
plot(tHist, uHist(1, :))
grid on
title('Left motor voltage')
ylabel('u_L [V]')
subplot(4, 3, 6)
plot(tHist, uHist(2, :))
grid on
title('Right motor voltage')
ylabel('u_R [V]')
subplot(4, 3, 9)
plot(tHist, HHist)
grid on
title('Total energy')
ylabel('H [J]')
subplot(4, 3, 12)
plot(tHist, gHist)
grid on
title('Nonholonomic constraint violation')
ylabel('B_c^T \nu [-]')
xlabel('Time [s]')

%% Animation

outputVideo = false;

if outputVideo
    vid = VideoWriter(['lab4' param.scenario param.controller '.mp4'], 'MPEG-4');
    vid.FrameRate = 1/param.dt;
    vid.Quality = 100;
    open(vid);
end

fig     = 2;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf, 'FontSize', 14);
grid on;
hold(ax, 'on');

[Egrid, Ngrid] = meshgrid(param.Egridlines, param.Ngridlines);
Cgrid = zeros(size(Egrid));
Cgrid(1:end-1, 1:end-1) = ~param.map;

colormap(ax, 'gray');
clim(ax, [0 1]);

hGrid   = pcolor(ax, Egrid, Ngrid, Cgrid);
hGrid.EdgeColor = 'none';

hDust = patch([12.5, 13, 13, 12.5], [0.5 0.5 1 1], 'red', 'EdgeColor', 'none');
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
grid(ax, 'on');
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

