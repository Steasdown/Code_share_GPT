function nu3 = robotVelocity(x,param)

% Get p from pr
p3 = robotMomentum(x,param);

% Obtain rigid body mass matrix for DOF of interest
%M6 = rigidBodyMassMatrix(param);
%M3 = M6(param.dofIdx,param.dofIdx);
M3 = [param.m, 0, 0;
            0, param.m, param.c*param.m;
            0, param.c*param.m, param.Iz];
% Get nu from p
nu3 = M3\p3;