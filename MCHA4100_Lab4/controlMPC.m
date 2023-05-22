function U = controlMPC(t1,x1,u0,U,param)

m = size(u0, 1);

if isempty(U)
    U = [u0; zeros(m*(param.ctrl.nControlHorizon-1), 1)];
else
    % warm start
    U = [U(m+1:end); zeros(m,1)];
end

switch param.scenario
    case 'track'
        options = optimoptions('lsqnonlin', ...
                    'Display','iter', ... % Use 'none' to disable console spam
                    'SpecifyObjectiveGradient', param.useGradientOpt, ...
                    'CheckGradients', false, ...
                    'MaxFunctionEvaluations', 1e5, ...
                    'MaxIterations', 1000);
        err     = @(U) errorMPC(t1, x1, u0, U, param);
        lb      = repmat(param.ctrl.uLowerBound, param.ctrl.nControlHorizon, 1);
        ub      = repmat(param.ctrl.uUpperBound, param.ctrl.nControlHorizon, 1);
        U       = lsqnonlin(err, U, lb, ub, options);
    case 'dock'
        options = optimoptions('fmincon',...
            'Algorithm', 'sqp', ...
            'Display', 'iter', ...
            'SpecifyObjectiveGradient', param.useGradientOpt, ...
            'SpecifyConstraintGradient', param.useGradientOpt, ...
            'CheckGradients', false, ...
            'MaxFunctionEvaluations', 1e5,...
            'MaxIterations', 1000);
        cost    = @(U) costMPC(t1, x1, u0, U, param);
        Aineq   = [];
        bineq   = [];
        Aeq     = [];
        beq     = [];
        lb      = repmat(param.ctrl.uLowerBound, param.ctrl.nControlHorizon, 1);
        ub      = repmat(param.ctrl.uUpperBound, param.ctrl.nControlHorizon, 1);
        nonlcon = @(U) nonlconMPC(t1, x1, u0, U, param);
        U       = fmincon(cost, U, Aineq, bineq, Aeq, beq, lb, ub, nonlcon, options);
    otherwise
        error('Unknown mode');
end
