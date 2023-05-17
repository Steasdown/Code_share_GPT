function [muy, Syy] = affineTransformWithAdditiveNoise(mux, Sxx, h)

%
% Square Root Affine Transform of y = h(x) + v, where v ~ N(0,R)
%

%
% Input arguments
%
% mux: mean of x
% Sxx: upper triangular matrix such that Sxx.'*Sxx is the covariance of x
% h:   function handle for model with the prototype [y, SR, C] = h(x),
%      where x is the input, y is the output, C = dh/dx is the Jacobian at x
%      and SR is an upper-triangular matrix such that SR.'*SR = R is the
%      covariance of the additive noise
%
% Output arguments
%
% muy: mean of y
% Syy: upper triangular matrix such that Syy.'*Syy is the covariance of y
%

assert(istriu(Sxx), 'Expected Sxx to be an upper triangular matrix');

% Evaluate model at x = mux
[muy, SR, C] = h(mux);
assert(istriu(SR), 'Expected SR to be an upper triangular matrix');
ny = length(muy);

R = triu(qr([Sxx*C.'; SR], 0));
Syy = R(1:ny, :);
