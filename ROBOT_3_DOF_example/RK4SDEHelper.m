function [xnext, J] = RK4SDEHelper(f , xdw, u, param, dt)

% This helper function evaluates the map
% [ x[k] ] |---> [ x[k+1] ]
% [  dw  ]
% for one step of RK4 integration so that an affine or unscented transform can
% be used to map
% (mu[k], S[k]) |---> (mu[k+1], S[k+1])
%

%
% Input arguments
%
% f:        function handle for continuous-time model
% xdw:      state augmented with noise increment [x; dw]
% u:        input
% param:   	parameters to pass to model
% dt:       time step
%
% Output arguments
%
% xnext:   	state at time index k+1
% J:        Jacobian matrix
% 

assert(mod(length(xdw), 2) == 0)
nx = length(xdw)/2;
x = xdw(1:nx);
dw = xdw(nx+1:2*nx);

if nargout < 2
    % Assume the input, u, is constant over the time step (ZOH)
    f1 = f(x,                  u, param);
    f2 = f(x + (f1*dt + dw)/2, u, param);
    f3 = f(x + (f2*dt + dw)/2, u, param);
    f4 = f(x + f3*dt + dw,     u, param);
    xnext = x + (f1 + 2*f2 + 2*f3 + f4)*dt/6 + dw;
else
    X = [x, eye(nx), zeros(nx)];
    dW = [dw, zeros(nx), eye(nx)];
    F = @(X, u, param) augmentGradients(f, X, u, param);
    % Assume the input, u, is constant over the time step (ZOH)
    F1 = F(X,                  u, param);
    F2 = F(X + (F1*dt + dW)/2, u, param);
    F3 = F(X + (F2*dt + dW)/2, u, param);
    F4 = F(X +  F3*dt + dW,    u, param);
    Xnext = X + (F1 + 2*F2 + 2*F3 + F4)*dt/6 + dW;
    xnext = Xnext(:, 1);
    J = Xnext(:, 2:end);
end

function dX = augmentGradients(f, X, u, param)
x = X(:, 1);
[dx, ~, Jx] = f(x, u, param);
dX = [dx, Jx*X(:, 2:end)];

