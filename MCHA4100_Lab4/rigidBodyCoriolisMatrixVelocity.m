function C = rigidBodyCoriolisMatrixVelocity(nu,param)

omegaBNb    = nu(4:6);
SomegaBNb   = skew(omegaBNb);
m           = param.m;
IBb         = param.IBb;
SrCBb       = skew(param.rCBb);


% Coriolis matrix in terms of body-fixed velocity
C = [m*SomegaBNb, -m*SomegaBNb*SrCBb;
    m*SrCBb*SomegaBNb, -skew(IBb*omegaBNb)];


end
