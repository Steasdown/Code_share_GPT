function [yx, SRR, CI] = augmentIdentityAdapter(h, x, u, param)

%
% Emulate the model augmented with an identity function
%
% [ y ] = [ h(x) ] + [ v ]
% [ x ]   [   x  ]   [ 0 ]
%
% where v ~ N(0,R)
%

%
% Input arguments
%
% h:        function handle for model
% x:        state
% u:        input
% param:   	parameters to pass to model
%
% Output arguments
%
% yx:       augmented output
% SRR:      upper triangular matrix such that SRR.'*SRR is the covariance of the augmented additive noise [v; 0]
% CI:       Jacobian matrix of the augmented model
% 

nx = length(x);
if nargout < 3
    [y, SR] = h(x, u, param);
else
    [y, SR, C] = h(x, u, param);
    CI = [C; eye(nx)];
end

yx = [y; x];

ny = length(y);
SRR = zeros(nx+ny);
SRR(1:ny, 1:ny) = SR;