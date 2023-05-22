function D = dampingMatrix(nu, param)

vBNb = nu(1:3);
omegaBNb = nu(4:6);

%
% Damping due to motor back-emf
%

Km      = param.Km;
Ra      = param.Ra;
Ng      = param.Ng;
rw      = param.rw;

SrLBb   = skew(param.rLBb);
SrRBb   = skew(param.rRBb);

% calculations for Matrix
Z = (1/Ra)*((Ng*Km/rw)^2);
A = eye(3)+eye(3);
B = -(SrLBb+SrRBb);
C = SrLBb+SrRBb;
E = -(SrLBb^2 + SrRBb^2);

% construct Damping matrix
D = Z*[A, B; 
        C, E];
