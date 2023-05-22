function [cineq, ceq, gcineq, gceq] = nonlconMPC(t1, x1, u0, U, param)

dt = param.dt;
Np = param.ctrl.nPredictionHorizon;
Nc = param.ctrl.nControlHorizon;
assert(Np >= Nc, 'Prediction horizon cannot be less than control horizon')

nx = length(x1);
nu = length(u0);

% Nonlinear equality constraints: ceq(x) == 0
ceq = []; % We don't have any for this problem, so leave it empty

% Nonlinear inequality constraints: cineq(x) <= 0
cineq = zeros(6*Np, 1);

t = t1;
x = x1;
f = @robotDynamics;

if nargout > 2
    gcineq = zeros(6*Np, nu*Nc);
    gceq = zeros(0, nu*Nc);
    dxdU = zeros(nx, nu*Nc);
    XX = [x, dxdU];
end

for k = 1:Np                 % for each step in the prediction horizon
    if k <= Nc
        u = U(nu*k-1:nu*k);
    else
        u = [0; 0];
    end
    
    % Do nSubsteps of Runge-Kutta integration using 3/8-rule
    for j = 1:param.ctrl.nSubsteps
        if nargout < 3
            f1 = f(t,          x,                           u, param);
            f2 = f(t + dt/3,   x + f1*dt/3,                 u, param);
            f3 = f(t + dt*2/3, x - f1*dt/3 + f2*dt,         u, param);
            f4 = f(t + dt,     x + f1*dt   - f2*dt + f3*dt, u, param);
            x = x + (f1 + 3*f2 + 3*f3 + f4)*dt/8;
        else
            dudU = zeros(nu, nu*Nc);
            if k <= Nc
                dudU(:, nu*(k-1)+1:nu*k) = eye(nu);
            end
            UU = [u, dudU];
            F = @(t,X,U,param) augmentGradients(f,t,X,U,param);
            F1 = F(t,          XX,                           UU, param);
            F2 = F(t + dt/3,   XX + F1*dt/3,                 UU, param);
            F3 = F(t + dt*2/3, XX - F1*dt/3 + F2*dt,         UU, param);
            F4 = F(t + dt,     XX + F1*dt   - F2*dt + F3*dt, UU, param);
            XX = XX + (F1 + 3*F2 + 3*F3 + F4)*dt/8;
            x = XX(:, 1);
            dxdU = XX(:, 2:end);
        end
        t = t + dt;
    end
    
q = x(3:5);
% obstacle variables
% Walls
NL = 0.4;
NU = 3.8;
EL = 1.1;
EU = 3.8;
% Obstacles
rO1Nn = [0.4; 2.5; 0];  % centre
r1 = 0.5;               % radius
rO2Nn = [1.75; 2.5; 0]; % centre
r2 = 0.25;              % radius
% Distance between C and furthest extent of robot chassis
L = 0.25;
% position variables
rCNn = [param.c*cos(x(5)); param.c*sin(x(5)); 0] + [x(3); x(4); 0]; % robot centre of mass position
e1 = [1; 0; 0]; % error1
e2 = [0; 1; 0]; % error2

c1 = r1 + L - norm(rCNn - rO1Nn);
c2 = r2 + L - norm(rCNn - rO2Nn);
c3 = -(NU - L) + e1.'*rCNn;
c4 = NL + L - e1.'*rCNn;
c5 =  EL + L - e2.'*rCNn;
c6 = -(EU - L) + e2.'*rCNn;
cineq((k-1)*6+1:k*6) = [c1; c2; c3; c4; c5; c6];
    
    if nargout > 2
        x1 = x(1);
        x2 = x(2);
        x3 = x(3);
        x4 = x(4);
        x5 = x(5);
        dc1dx = [0, 0, -(abs(x3 + cos(x5)/10 - 2/5)*(x3 + cos(conj(x5))/10 + conj(x3) + cos(x5)/10 - 4/5))/(2*(abs(x3 + cos(x5)/10 - 2/5)^2 + abs(x4 + sin(x5)/10 - 5/2)^2)^(1/2)*((x3 + cos(x5)/10 - 2/5)*(cos(conj(x5))/10 + conj(x3) - 2/5))^(1/2)), -(abs(x4 + sin(x5)/10 - 5/2)*(x4 + sin(conj(x5))/10 + conj(x4) + sin(x5)/10 - 5))/(2*(abs(x3 + cos(x5)/10 - 2/5)^2 + abs(x4 + sin(x5)/10 - 5/2)^2)^(1/2)*((x4 + sin(x5)/10 - 5/2)*(sin(conj(x5))/10 + conj(x4) - 5/2))^(1/2)), ((abs(x3 + cos(x5)/10 - 2/5)*((sin(x5)*(cos(conj(x5))/10 + conj(x3) - 2/5))/10 + (sin(conj(x5))*(x3 + cos(x5)/10 - 2/5))/10))/((x3 + cos(x5)/10 - 2/5)*(cos(conj(x5))/10 + conj(x3) - 2/5))^(1/2) - (abs(x4 + sin(x5)/10 - 5/2)*((cos(x5)*(sin(conj(x5))/10 + conj(x4) - 5/2))/10 + (cos(conj(x5))*(x4 + sin(x5)/10 - 5/2))/10))/((x4 + sin(x5)/10 - 5/2)*(sin(conj(x5))/10 + conj(x4) - 5/2))^(1/2))/(2*(abs(x3 + cos(x5)/10 - 2/5)^2 + abs(x4 + sin(x5)/10 - 5/2)^2)^(1/2))];
        dc2dx = [0, 0, -(abs(x3 + cos(x5)/10 - 7/4)*(x3 + cos(conj(x5))/10 + conj(x3) + cos(x5)/10 - 7/2))/(2*(abs(x3 + cos(x5)/10 - 7/4)^2 + abs(x4 + sin(x5)/10 - 5/2)^2)^(1/2)*((x3 + cos(x5)/10 - 7/4)*(cos(conj(x5))/10 + conj(x3) - 7/4))^(1/2)), -(abs(x4 + sin(x5)/10 - 5/2)*(x4 + sin(conj(x5))/10 + conj(x4) + sin(x5)/10 - 5))/(2*(abs(x3 + cos(x5)/10 - 7/4)^2 + abs(x4 + sin(x5)/10 - 5/2)^2)^(1/2)*((x4 + sin(x5)/10 - 5/2)*(sin(conj(x5))/10 + conj(x4) - 5/2))^(1/2)), ((abs(x3 + cos(x5)/10 - 7/4)*((sin(x5)*(cos(conj(x5))/10 + conj(x3) - 7/4))/10 + (sin(conj(x5))*(x3 + cos(x5)/10 - 7/4))/10))/((x3 + cos(x5)/10 - 7/4)*(cos(conj(x5))/10 + conj(x3) - 7/4))^(1/2) - (abs(x4 + sin(x5)/10 - 5/2)*((cos(x5)*(sin(conj(x5))/10 + conj(x4) - 5/2))/10 + (cos(conj(x5))*(x4 + sin(x5)/10 - 5/2))/10))/((x4 + sin(x5)/10 - 5/2)*(sin(conj(x5))/10 + conj(x4) - 5/2))^(1/2))/(2*(abs(x3 + cos(x5)/10 - 7/4)^2 + abs(x4 + sin(x5)/10 - 5/2)^2)^(1/2))];
        dc3dx = [0, 0,                                                                                                                                                                                                                               1,                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                   -sin(x5)/10];
        dc4dx = [0, 0,                                                                                                                                                                                                                              -1,                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                    sin(x5)/10];
        dc5dx = [0, 0,                                                                                                                                                                                                                               0,                                                                                                                                                                                                                            -1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                   -cos(x5)/10];
        dc6dx = [0, 0,                                                                                                                                                                                                                               0,                                                                                                                                                                                                                             1,                                                                                                                                                                                                                                                                                                                                                                                                                                                                    cos(x5)/10];
        gcineq((k-1)*6+1:k*6, :) = [dc1dx; dc2dx; dc3dx; dc4dx; dc5dx; dc6dx]*dxdU;

        %         dc1dx = [-((rCNn - rO1Nn).')/norm(rCNn - rO1Nn), 0, 0, 0, 0]; % gradient of obstacle 1 constraint w.r.t. x
%         dc2dx = [-((rCNn - rO2Nn).')/norm(rCNn - rO2Nn), 0, 0, 0, 0]; % gradient of obstacle 2 constraint w.r.t. x
%         dc3dx = [-e1.' zeros(1, 4)]; % gradient of north wall upper constraint w.r.t. x
%         dc4dx = [e1.' zeros(1, 4)]; % gradient of north wall lower constraint w.r.t. x
%         dc5dx = [0, 0, -e2.', 0, 0]; % gradient of East wall upper constraint w.r.t. x
%         dc6dx = [0, 0, e2.', 0, 0]; % gradient of East wall lower constraint w.r.t. x
    end
end

if nargout > 2
    gcineq = gcineq.';
    gceq = gceq.';
end
