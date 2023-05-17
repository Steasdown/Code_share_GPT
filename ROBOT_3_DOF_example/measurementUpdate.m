function [mu, S] = measurementUpdate(mu, S, u, y, measurementModel, param, transformMethod)

%
% Condition the Gaussian pdf p(x) on measurement y to obtain p(x|y)
%

%
% Input arguments
%
% mu:	state mean
% S:    upper triangular matrix such that S.'*S is the covariance of the state
% u:    input
% y:    measurement
% measurementModel:     function handle to measurement model
% param:                parameters to pass to measurement model
% transformMethod:      'affine' or 'unscented' transform
%
% Output arguments
%
% mu:   state mean conditioned on measurement
% S:    upper triangular matrix such that S.'*S is the covariance of the state conditioned on the measurement
%

switch transformMethod
    case 'affine'
        transform = @affineTransformWithAdditiveNoise;
    case 'unscented'
        transform = @unscentedTransformWithAdditiveNoise;
    otherwise
        error('Expecting transformMethod to be ''affine'' or ''unscented''');
end

% Apply transform through the measurement model augmented with
% the identity function
%
% (mux,Pxx) |---> ([ muy ] [Pyy+R Pyx])
%                 ([ mux ],[Pxy   Pxx])
%
jointFunc = @(x) augmentIdentityAdapter(measurementModel, x, u, param);
[muyx, Syx] = transform(mu, S, jointFunc);

% Condition measurement and state prediction joint Gaussian on current
% measurement
[mu, S] = conditionGaussianOnMarginal(muyx, Syx, y);
