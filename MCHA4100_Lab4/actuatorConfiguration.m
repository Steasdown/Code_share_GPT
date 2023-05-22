function Ba = actuatorConfiguration(param)
%ACTUATORCONFIGURATION Actuator configuration matrix maps actuator forces
%into body forces
%
% tau = Ba * u
%

Km      = param.Km;
Ra      = param.Ra;
Ng      = param.Ng;
rw      = param.rw;

rLBb    = param.rLBb;
rRBb    = param.rRBb;
e1      = [1; 0; 0];

% matrix calculations
A = Ng*Km/(Ra*rw)*e1;
B = Ng*Km/(Ra*rw)*e1;
C = Ng*Km/(Ra*rw)*skew(rLBb)*e1;
D = Ng*Km/(Ra*rw)*skew(rRBb)*e1;

% construct actuator matrix
Ba = [A, B;
      C, D];

end