function [muy, Syy] = unscentedTransform(mux, Sxx, h)

%
% Square Root Unscented Transform of y = h(x)
% using 2*n + 1 sigma points
%

%
% Input arguments
%
% mux: mean of x
% Sxx: upper triangular matrix such that Sxx.'*Sxx is the covariance of x
% h:   function handle for model with the prototype [y, C] = h(x)
%
% Output arguments
%
% muy: mean of y
% Syy: upper triangular matrix such that Syy.'*Syy is the covariance of y
%

assert(istriu(Sxx), 'Expected Sxx to be an upper triangular matrix');

n = length(mux);          	% Length of input vector
nsigma = 2*n+1;             % Number of sigma points

% Unscented transform parameters
alpha = 1;
kappa = 0;
lambda = alpha^2*(n + kappa) - n;
gamma = realsqrt(n + lambda);
beta = 2;

% Mean weights
Wm = [repmat(1/(2*(n+lambda)), 1, 2*n), lambda/(n+lambda)];

% Covariance weights
Wc = [repmat(1/(2*(n+lambda)), 1, 2*n), lambda/(n+lambda) + (1-alpha^2+beta)];

% Generate sigma points
xsigma = zeros(n, nsigma);   % Input sigma points
for i = 1:n
    xsigma(:, i)   = mux + gamma*Sxx(i, :).';
    xsigma(:, i+n) = mux - gamma*Sxx(i, :).';
end
xsigma(:, 2*n+1) = mux;

% Transform the sigma points through the function
muy = h(xsigma(:, nsigma));             % Use function eval at mean to
ny = length(muy);                       % determine output dimension and
ysigma = zeros(ny, nsigma);             % initialise output sigma points
ysigma(:, nsigma) = muy;
for i = 1:nsigma-1
    ysigma(:, i) = h(xsigma(:, i));
end

% Unscented mean
muy = sum(Wm.*ysigma, 2);

% Compute conditional mean and sqrt covariance
dysigma = realsqrt(Wc).*(ysigma - muy);
R = triu(qr(dysigma.', 0));
Syy = R(1:ny, :);
