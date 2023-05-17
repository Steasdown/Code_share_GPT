function [f, SQ, A] = robotProcessModel(x, u, param)

Km = param.Km;
Ng = param.Ng;
Ra = param.Ra;
d = param.d;
rw = param.rw;

if nargout > 2
    % Evaluate f(x, u) and its gradient A = df/dx
    [f, A] = robotDynamics(0, x, u, param);
else
    % Evaluate f(x,u)
    f = robotDynamics(0, x, u, param);
end

% Augment gyro bias dynamics: bdot = 0
f = [f; 0];
if nargout > 2
    A = blkdiag(A, 0);
end

% sigmau = std(u)^2;                 % Additive input voltage noise intensity [V.s^0.5]
% sigmab = 0;                 % Gyro bias drift intensity [rad.s^-1.5]

sigmau = 0.04;                 % Additive input voltage noise intensity [V.s^0.5]
sigmab = 0.00015;                 % Gyro bias drift intensity [rad.s^-1.5]

SQ = [sqrt((2*Km*Ng)/(Ra*rw)^2*sigmau^2), 0, 0, 0, 0;
    0, sqrt((Km*Ng*d)/(2*Ra*rw)^2*sigmau^2), 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0;
    0, 0, 0, 0, 0];
    % The 5x5 SQ from Problem 1

SQ = blkdiag(SQ, sigmab);   % Augment with gyro bias drift intensity (used in Problem 2 onwards)

assert(istriu(SQ), 'Expect SQ to be upper triangular');
