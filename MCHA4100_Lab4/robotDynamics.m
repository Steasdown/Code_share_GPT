function [dx, Jx, Ju] = robotDynamics(t, x, u, param)
pr  = x(1:2);   % Reduced momentum
q   = x(3:5);   % Displacement
Km = param.Km;
Ng = param.Ng;
Ra = param.Ra;
rw = param.rw;
c = param.c;
Iz = param.Iz;
l = param.l;
d = param.d;
m = param.m;

dx = [(-(2*Km^2*Ng^2)/(Ra*m*rw^2))*x(1) + ((m*(c - l))/(m*l^2 - 2*c*m*l + Iz)^2)*x(2)^2 + (Km*Ng*u(1))/(Ra*rw) + (Km*Ng*u(2))/(Ra*rw);
(-(c - l)/(m*l^2 - 2*c*m*l + Iz))*x(1)*x(2) + (((2*Km^2*Ng^2*l^2)/(Ra*rw^2) - (Km^2*Ng^2*(d^2/2 + 2*l^2))/(Ra*rw^2))/(m*l^2 - 2*c*m*l + Iz))*x(2) + (Km*Ng*d*u(1))/(2*Ra*rw) - (Km*Ng*d*u(2))/(2*Ra*rw);
 (cos(x(5))/m)*x(1) + ((l*sin(x(5)))/(m*l^2 - 2*c*m*l + Iz))*x(2);
(sin(x(5))/m)*x(1) + (-(l*cos(x(5)))/(m*l^2 - 2*c*m*l + Iz))*x(2);
x(2)/(m*l^2 - 2*c*m*l + Iz)];

if nargout >= 2
    % Reserved for future lab
    Jx = [-(2*Km^2*Ng^2)/(Ra*m*rw^2), (2*m*x(2)*(c - l))/(m*l^2 - 2*c*m*l + Iz)^2, 0, 0, 0; 
        -(x(2)*(2*Ra*c*rw^2 - 2*Ra*l*rw^2))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)), - (x(1)*(2*Ra*c*rw^2 - 2*Ra*l*rw^2))/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)) - (Km^2*Ng^2*d^2)/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz)), 0, 0, 0;
         (cos(x(5))/m), ((l*sin(x(5)))/(m*l^2 - 2*c*m*l + Iz)), 0, 0, ((l*x(2)*cos(x(5)))/(m*l^2 - 2*c*m*l + Iz)) - (x(1)*sin(x(5))/m);
         sin(x(5))/m, -(l*cos(x(5)))/(m*l^2 - 2*c*m*l + Iz), 0, 0, (x(1)*cos(x(5)))/m + (l*x(2)*sin(x(5)))/(m*l^2 - 2*c*m*l + Iz);
         0, 1/(m*l^2 - 2*c*m*l + Iz), 0, 0, 0];
    
    
    %Jx = zeros(5,5);
end

if nargout >= 3
    %Reserved for future lab
    Ju =     [    (Km*Ng)/(Ra*rw),      (Km*Ng)/(Ra*rw);
                (Km*Ng*d)/(2*Ra*rw), -(Km*Ng*d)/(2*Ra*rw);
                      0,                    0;
                      0,                    0;
                      0,                    0];
    %Ju = zeros(5,2);
end

% Extract states

