function X = trajectoryEval(traj, t) %#codegen

X = nan(traj.nDim, (traj.nOrder+1)/2, length(t));

for i = 1:length(t)
    if t(i) < traj.t(1)
        X(:, :, i) = traj.X(:, :, 1);
        X(:, 2:end, i) = 0;
        continue
    end
    if t(i) > traj.t(end)
        X(:, :, i) = traj.X(:, :, end);
        X(:, 2:end, i) = 0;
        continue
    end

    if t(i) == traj.t(end)
        X(:, :, i) = traj.X(:, :, end);
        continue
    end

    T = zeros(traj.nOrder+1, (traj.nOrder+1)/2);
    T(:, 1) = t(i).^(0:traj.nOrder).';
    for j = 1:(traj.nOrder-1)/2
        T(:, j+1) = traj.D*T(:, j);
    end

    k = find(traj.t <= t(i), 1, 'last');
    X(:, :, i) = traj.C(:, :, k)*T;
end
