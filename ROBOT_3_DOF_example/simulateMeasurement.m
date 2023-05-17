function y = simulateMeasurement(measurementModel,x,u,param)

% Measurement likelihood
%   p(y|x,u) = N(y;h(x,u),R)

[h,SR] = measurementModel(x,u,param);

% SR is an upper triangular matrix such that SR.'*SR = R
% Let z ~ N(0,I) = c*exp(-0.5*z.'*z) be a standard normally distributed sample.
% Then, z = SR.'\(y - h(x,u))
% Therefore y = h(x,u) + SR.'*z

y = h + SR.'*randn(size(h,1),1);