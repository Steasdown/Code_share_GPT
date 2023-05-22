function nu3 = robotVelocity(x, param)

% Get p from pr
p3 = robotMomentum(x, param);

% Obtain rigid body mass matrix for DOF of interest
M6 = rigidBodyMassMatrix(param);
M3 = M6(param.dofIdx, param.dofIdx);

% Get nu from p
nu3 = M3\p3;
