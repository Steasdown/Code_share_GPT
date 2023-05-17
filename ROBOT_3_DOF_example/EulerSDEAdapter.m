function [xnext, SQQ, AA] = EulerSDEAdapter(f, x, u, param, dt)

% Perform one step of forward Euler integration for the continuous-time SDE
%   dx = f(x, u)*dt + dw, where dw ~ (0, Q*dt),
% and emulate the interface for a discrete-time process model
%   x[k+1] = x[k] + f(x[k], u[k])*dt + dw[k]
%

%
% Input arguments
%
% f:        function handle for continuous-time model
% x:        state at time index k
% u:        input
% param:   	parameters to pass to model
% dt:       time step
%
% Output arguments
%
% xnext:   	state at time index k+1
% SQQ:      upper triangular matrix such that SQQ.'*SQQ is the discrete-time process noise covariance
% AA:       Jacobian matrix
% 

if nargout < 3
    [fx, SQ] = f(x, u, param);
else
    [fx, SQ, A] = f(x, u, param);
    nx = length(x);
    AA = eye(nx) + A*dt; % Warning: If dt is too small, this can round off to I
end
xnext = x + fx*dt;

SQQ = realsqrt(dt)*SQ;
