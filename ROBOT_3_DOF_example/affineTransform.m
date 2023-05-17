function [muy, Syy] = affineTransform(mux, Sxx, h)

%
% Square Root Affine Transform of y = h(x)
%

%
% Input arguments
%
% mux: mean of x
% Sxx: upper triangular matrix such that Sxx.'*Sxx is the covariance of x
% h:   function handle for model with the prototype [y, C] = h(x),
%      where x is the input, y is the output and C = dh/dx is the Jacobian at x
%
% Output arguments
%
% muy: mean of y
% Syy: upper triangular matrix such that Syy.'*Syy is the covariance of y
%

assert(istriu(Sxx), 'Expected Sxx to be an upper triangular matrix');

% Evaluate model at x = mux
[muy, C] = h(mux);
ny = length(muy);

R = triu(qr(Sxx*C.', 0));
Syy = R(1:ny, :);
