function [h, SR, C] = gyroscopeMeasurementModel(x, u, param)

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

m1b = [0; 1; 0];
m2b = [1; 0; 0];
m3b = [0; 0;-1];

sigmar = deg2rad(1);                 % Gyroscope channel stdev [rad/s]
SR = sigmar;
assert(istriu(SR), 'Expect SR to be upper triangular');

Rbm = [m1b m2b m3b];   % Rbm = [m1b, m2b, m3b], where mib is the ith accelerometer axis expressed in {b}

% Extract states
pr  = x(1:2);   % Reduced momentum
b = x(6);
%b   = sigmab;     % Gyro bias

nu  = ([Bc.'/M3; Bcp]*M3\[0; pr]);
wBNb = [0; 0; nu(3)];

h = [0 0 1]*Rbm*wBNb + b;

if nargout > 2
    % Jacobian of h w.r.t. x
    % C = zeros(6,1)
    C = [0, -1/(m*l^2 - 2*c*m*l + Iz), 0, 0, 0, 1];

end
