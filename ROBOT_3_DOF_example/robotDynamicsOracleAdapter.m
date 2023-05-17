function dx = robotDynamicsOracleAdapter(t, x, u, param)

persistent T
if isempty(T)
    [~,Bcp_student] = robotConstraints(param);
    load('K.mat','K');
    T = Bcp_student*K;
end

if param.inputDisturbance
    u = u + inputDisturbance(t);
end

% Transform student momentum state to internal momentum state
x(1:2) = T\x(1:2);

dx = robotDynamicsOracleFast(t,x,u,param);

% Transform internal momentum state to student momentum state
dx(1:2) = T*dx(1:2);

