function H = robotEnergy(x,param)

% Get p from pr
p3 = robotMomentum(x,param);

% Obtain rigid body mass matrix for DOF of interest
M6 = rigidBodyMassMatrix(param);
M3 = M6(param.dofIdx,param.dofIdx);

% Compute Hamiltonian
H = 0.5*p3.'*(M3\p3);