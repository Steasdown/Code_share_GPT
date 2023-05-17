function [V, g] = costMPC(t1, x1, u0, U, param)

dt = param.dt/param.ctrl.nSubsteps;
Np = param.ctrl.nPredictionHorizon;
Nc = param.ctrl.nControlHorizon;
assert(Np >= Nc, 'Prediction horizon cannot be less than control horizon')

qr      = param.ctrl.qr;
qpsi    = param.ctrl.qpsi;
ru      = param.ctrl.ru;

nx = length(x1);
nu = length(u0);
e  = zeros(5*Np, 1);

t = t1;
x = x1;
f = @robotDynamics;

if nargout > 1
    J = zeros(nx*Nc, nu*Nc);
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
        if nargout < 2
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
            F = @(t, X, U, param) augmentGradients(f, t, X, U, param);
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
    rENn = [0.75;1.7;0];
    rCNn = [param.c*cos(x(5)); param.c*sin(x(5)); 0] + [x(3); x(4); 0];
    psistar = 0;
    rCEn = rCNn - rENn;
    if t>= 5
        er = sqrt(qr)*rCEn(1:2);
        epsi = sqrt(qpsi)*(x(5) - psistar);
    else                 
        er      = [0;0];
        epsi    = 0;   
    end
    eu = sqrt(ru)*(u);
    e((k-1)*5+1:k*5) = [er; epsi; eu];
    if nargout > 1
        if t>= 5
            der = [0, 0, sqrt(qr), 0, -sqrt(qr)*sin(x(5))*param.c;
                   0, 0, 0, sqrt(qr), qr^0.5*cos(x(5))*param.c]*dxdU;
            depsi = [0, 0, 0, 0, sqrt(qpsi)]*dxdU;
        else
            der = zeros(2,5)*dxdU;
            depsi = zeros(1,5)*dxdU;
        end
            deu = sqrt(ru)*dudU;
            J((k-1)*5+1:k*5,:) = [der;depsi;deu];
    end
end

V = e.'*e;
if nargout > 1
    g = 2*J.'*e;
end