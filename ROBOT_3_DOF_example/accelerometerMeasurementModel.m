function [h, SR, C] = accelerometerMeasurementModel(x, u, param)

c = param.c;
m = param.m;
Iz = param.Iz;
l = param.l;
M3 = param.M3;
Bc = param.Bc;
Bcp = param.Bcp;
g = param.g;
Ra = param.Ra;
rw = param.rw;
Km = param.Km;
Ng = param.Ng;
d = param.d;
Rbn = [cos(x(5)), -sin(x(5)), 0;
        sin(x(5)), cos(x(5)), 0;
        0, 0, 1];
sigmar = 0.1;                 % Accelerometer channel stdev [m/s^2]
SR = sigmar*eye(2);
assert(istriu(SR), 'Expect SR to be upper triangular');

m1b = [0; 1; 0];
m2b = [1; 0; 0];
m3b = [0; 0;-1];

rMBb = [0.15; 0; 0];        % Position of accelerometer w.r.t B expressed in {b}
Rbm = [m1b m2b m3b];               % Rbm = [m1b, m2b, m3b], where mib is the ith accelerometer axis expressed in {b}

gn = [0 0 g]';

% Extract states
pr  = x(1:2);   % Reduced momentum
q   = x(3:5);   % Displacement
nu  = M3\([Bc' / M3; Bcp]\[0; pr]);  % velocity matrix
wBNb = [0; 0; nu(3)];   % angular velocity matrix for the body in reference to the world, body coordinates
vBNb = [nu(1); nu(2); 0];   % velocity matrix for the body in reference to the world, body coordinates

% Get time derivatives of state
if nargout > 2
    [xdot, dxdotdx] = robotDynamics(0, x(1:5), u, param);
    dxdotdx = blkdiag(dxdotdx, 0);
else
    xdot = robotDynamics(0, x(1:5), u, param);
end
xdot = [xdot; 0];

nuDot = M3\([Bc' / M3; Bcp]\[0; xdot(1); xdot(2)]); % acceleration matrix (integral of velocity)
wBNbDot = [0; 0; nuDot(3)]; % angular acceleration for the body in reference to the world, body coordinates
vBNbDot = [nuDot(1); nuDot(2); 0];  % acceleration matrix for the body in reference to the world, body coordinates

aMNb = -skew(rMBb)*wBNbDot + vBNbDot + skew(wBNb)*skew(wBNb)*rMBb + skew(wBNb)*vBNb;  % generalised acceleration for point M, , body coordinates

h = [1 0 0; 0 1 0]*Rbm*(aMNb - Rbn*gn); % construcitng h matrix 

if nargout > 2
    % Jacobian of h w.r.t. x
    %C = zeros(2, 6);
    x1 = x(1);
    x2 = x(2);
        C =[x2/(m*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) + (2*Ra*c*rw^2*x2*conj(l) - 2*Ra*l*rw^2*x2*conj(l))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) - (3*(2*Ra*c*rw^2*x2 - 2*Ra*l*rw^2*x2))/(40*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))), x1/(m*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) - (3*(Km^2*Ng^2*d^2 + 2*Ra*c*rw^2*x1 - 2*Ra*l*rw^2*x1))/(40*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) + (Km^2*Ng^2*d^2*conj(l) + 2*Ra*c*rw^2*x1*conj(l) - 2*Ra*l*rw^2*x1*conj(l))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))), 0, 0, 0, 0;
-(2*Iz^2*Km^2*Ng^2 - 8*Iz*Km^2*Ng^2*c*l*m + 4*Iz*Km^2*Ng^2*l^2*m + 8*Km^2*Ng^2*c^2*l^2*m^2 - 8*Km^2*Ng^2*c*l^3*m^2 + 2*Km^2*Ng^2*l^4*m^2)/(Ra*m^2*rw^2*(m*l^2 - 2*c*m*l + Iz)^2), (2*x2*conj(l))/(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))^2 - (3*x2)/(10*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))^2) + (2*Ra*c*m^2*rw^2*x2 - 2*Ra*l*m^2*rw^2*x2)/(Ra*m^2*rw^2*(m*l^2 - 2*c*m*l + Iz)^2), 0, 0, 0, 0];

%     C =[x2/(m*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) + (2*Ra*c*rw^2*x2*conj(l) - 2*Ra*l*rw^2*x2*conj(l))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) - (3*(2*Ra*c*rw^2*x2 - 2*Ra*l*rw^2*x2))/(40*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))), x1/(m*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) - (3*(Km^2*Ng^2*d^2 + 2*Ra*c*rw^2*x1 - 2*Ra*l*rw^2*x1))/(40*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) + (Km^2*Ng^2*d^2*conj(l) + 2*Ra*c*rw^2*x1*conj(l) - 2*Ra*l*rw^2*x1*conj(l))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))), 0, x1/(m*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) - (3*(Km^2*Ng^2*d^2 + 2*Ra*c*rw^2*x1 - 2*Ra*l*rw^2*x1))/(40*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))) + (Km^2*Ng^2*d^2*conj(l) + 2*Ra*c*rw^2*x1*conj(l) - 2*Ra*l*rw^2*x1*conj(l))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))), 0, 0;
% -(2*Iz^2*Km^2*Ng^2 - 8*Iz*Km^2*Ng^2*c*l*m + 4*Iz*Km^2*Ng^2*l^2*m + 8*Km^2*Ng^2*c^2*l^2*m^2 - 8*Km^2*Ng^2*c*l^3*m^2 + 2*Km^2*Ng^2*l^4*m^2)/(Ra*m^2*rw^2*(m*l^2 - 2*c*m*l + Iz)^2), (2*x2*conj(l))/(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))^2 - (3*x2)/(10*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))^2) + (2*Ra*c*m^2*rw^2*x2 - 2*Ra*l*m^2*rw^2*x2)/(Ra*m^2*rw^2*(m*l^2 - 2*c*m*l + Iz)^2), 0,(2*x2*conj(l))/(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))^2 - (3*x2)/(10*(Iz + l*m*conj(l) - c*l*m - c*m*conj(l))^2) + (2*Ra*c*m^2*rw^2*x2 - 2*Ra*l*m^2*rw^2*x2)/(Ra*m^2*rw^2*(m*l^2 - 2*c*m*l + Iz)^2), 0, 0];
 


end
