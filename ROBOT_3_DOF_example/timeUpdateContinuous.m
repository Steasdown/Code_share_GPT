function [mu, S] = timeUpdateContinuous(mu, S, u, processModel, param, timestep, integrationMethod, transformMethod)

%
% Integrate a continuous-time process over one time step
%

%
% Input arguments
%
% mu: state mean at time index k-1
% S:  upper triangular matrix such that S.'*S is the covariance of the state at time index k-1
% u:  input (ZOH over the time step)
% processModel:         function handle to continuous-time process model
% param:                parameters to pass to process model
% timestep:             Time step for integration
% integrationMethod:    'Euler' or 'RK4' integration method
% transformMethod:      'affine' or 'unscented' transform
%
% Output arguments
%
% mu: state mean at time index k
% S:  upper triangular matrix such that S.'*S is the covariance of the state at time index k
%



% Perform one step of integration for the SDE
%   dx = f(x)*dt + dw, where dw ~ (0, Q*dt)
% to map (mu[k],P[k]) |---> (mu[k+1],P[k+1])
switch integrationMethod
    case 'Euler'
        % Since Euler SDE integration remains affine in the noise increment, we
        % can build a discrete-time model directly from the continuous-time model
        % using an adapter function.
        processDiscreteFunc = @(x) EulerSDEAdapter(processModel,x,u,param,timestep);
        switch transformMethod
            case 'affine'
                [mu, S] = affineTransformWithAdditiveNoise(mu,S,processDiscreteFunc);
            case 'unscented'
                [mu, S] = unscentedTransformWithAdditiveNoise(mu, S, processDiscreteFunc);
            otherwise
                error('Expecting transformMethod to be ''affine'' or ''unscented''');
        end
    case 'RK4'
        % Since the RK4 SDE integration is not affine in the noise increment, we
        % augment the state with the noise increment and transform this jointly
        % through the RK4 method.
        RK4Func = @(xdw) RK4SDEHelper(processModel, xdw, u, param, timestep);
        [~,SQ] = processModel(mu, u, param);
        muxdw = [mu; zeros(size(mu))];
        Sxdw = blkdiag(S, SQ*realsqrt(timestep));
        switch transformMethod
            case 'affine'
                [mu, S] = affineTransform(muxdw, Sxdw, RK4Func);
            case 'unscented'
                [mu, S] = unscentedTransform(muxdw, Sxdw, RK4Func);
            otherwise
                error('Expecting transformMethod to be ''affine'' or ''unscented''');
        end
    otherwise
        error('Expecting integrationMethod to be ''Euler'' or ''RK4''');
end

end



