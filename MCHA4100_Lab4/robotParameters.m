function param = robotParameters()

param.l     = 0.2;          % Longitudinal distance from B to wheels [m]
param.d     = 0.3;          % Distance between wheels [m]
param.c     = 0.1;          % Longitudinal distance from B to C [m]
param.s     = 0.05;         % Longitudinal distance from B to S [m]
param.rSBb  = [-param.s; 0; 0];             % skid position in relation to B
% 1a
param.rLBb  = [param.l; -param.d/2; 0];  % Left wheel position in relation to B
param.rRBb  = [param.l;  param.d/2; 0];   % Right wheel position in relation to B
param.rCBb  = [param.c; 0; 0];              % center of mass position, relation to B

param.Km    = 0.014;        % Motor constant [N.m/A] or [V.s/rad]
param.Ra    = 0.657;        % Motor armature resistance [Ohm]
param.Ng    = 5;            % Motor gear ratio [-]
param.rw    = 0.04;         % Wheel radius [m]

param.m     = 3;            % Robot mass [kg]
param.Iz    = 3*0.15^2;     % Yaw inertia [kg.m^2]
param.g     = 3.71;         % Acceleration due to gravity on Mars [m/s^2]
param.IBb   = diag([0,0,param.Iz]);

param.dofIdx = [1 2 6];     % N, E, psi

Km = param.Km;
Ng = param.Ng;
Ra = param.Ra;
rw = param.rw;
c = param.c;
Iz = param.Iz;
l = param.l;
d = param.d;
m = param.m;

param.Bc = [0; 1; l];
param.Bcp = [1, 0, 0;     % left annilhilator  
            0, -l, 1];


%M6 = rigidBodyMassMatrix(param);            % create full mass matrix
%param.M3 = M6(param.dofIdx,param.dofIdx); 

param.M3 = [param.m, 0, 0;
            0, param.m, param.c*param.m;
            0, param.c*param.m, param.Iz];

Ba6 = actuatorConfiguration(param);
param.Ba3 = Ba6(param.dofIdx,:);

param.G = [param.Bcp*param.Ba3; zeros(3,2)];

const = (m*l^2 - 2*c*m*l + Iz);

% a11 = -(2*Km^2*Ng^2);
% a12 = (Ra*m*rw^2);
% Ax1 = (a11/a12);
% a21 = (l*m*(c - l));
% a22 = const^2;
% Ax2 = a21/a22;
% a31 = (-(c-l));
% a32 = (l*const);
% Ax3 = a31/a32;
% a41 = -(Km^2*Ng^2*d^2);
% a42 = (2*Ra*rw^2*const);
% Ax4 = (a41/a42);
% Ax5 = (l*m);
% Ax6 = l/const; 
% %a71 = (m*l^2 - 2*c*m*l + Iz);
% param.A1 = Ax1; % -(2*Km^2*Ng^2)/(Ra*m*rw^2)
% param.A2 = Ax2; % (l*m*(c - l))/(m*l^2 - 2*c*m*l + Iz)^2
% param.A3 = Ax3; % (-(c-l))/(2*Ra*l*rw^2*(m*l^2 - 2*c*m*l + Iz))
% param.A4 = Ax4; % (-(Km^2*Ng^2*d^2)/(2*Ra*rw^2*(m*l^2 - 2*c*m*l + Iz))
% param.A5 = Ax5; % 1/l*m
% param.A6 = Ax6; % 1/(m*l^2 - 2*c*m*l + Iz)
% param.A7 = 1/const; % (m*l^2 - 2*c*m*l + Iz)
% param.B1 = (Km*Ng*l)/(Ra*rw);
% param.B2 = (Km*Ng*d)/(2*Ra*rw);
