function dX = augmentGradients(f, t, X, U, param)

dX = zeros(size(X));

x = X(:, 1);
u = U(:, 1);
[dx, Jx, Ju] = f(t, x, u, param);

dX(:,1) = dx;
dX(:, 2:end) = Jx*X(:, 2:end) + Ju*U(:, 2:end);
