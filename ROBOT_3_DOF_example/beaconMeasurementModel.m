function [h, SR, C] = beaconMeasurementModel(x, u, param)

% rZNn_all = [ ...
%     7.5, 7.5, 6, 3;
%     3.5, 15, 9, 9;
%     -3, -3, -3, -3];
% 
% 
sigmar = 0.1;                 % Range beacon stdev [m]
% Extract states
%q   = x(3:5);   % Displacement

SR = sigmar*eye(4);
assert(istriu(SR), 'Expect SR to be upper triangular');

Rbn = [cos(x(5)), -sin(x(5)), 0;
        sin(x(5)), cos(x(5)), 0;
        0, 0, 1];

rQBb = [-0.05; 0; 0];

rBNn = [x(3); x(4); 0];

rQNn = rBNn + Rbn*rQBb;

rZ1Nn = [7.3; 3.5; -3];
rZ2Nn = [7.5;  15; -3];
rZ3Nn = [6;     9; -3];
rZ4Nn = [3;     9; -3];


h = [norm(rQNn - rZ1Nn);
    norm(rQNn - rZ2Nn);
    norm(rQNn - rZ3Nn);
    norm(rQNn - rZ4Nn)];

if nargout > 2
    % Jacobian of h w.r.t. x
    % C = zeros(4, 6);
    % Compute partial derivatives of rQNn
    drQNn_dx3 = [1; 0; 0];
    drQNn_dx4 = [0; 1; 0];
    
    dRbn_dx5 = [-sin(x(5)), -cos(x(5)), 0;
                 cos(x(5)), -sin(x(5)), 0;
                 0, 0, 0];
    drQNn_dx5 = dRbn_dx5 * rQBb;
    
    % Compute partial derivatives of h
    C = zeros(4, 6);
    
    for i = 1:4
        rZ_n = eval(['rZ', num2str(i), 'Nn']);
        diff_rQNn_rZ = rQNn - rZ_n;
        norm_diff = norm(diff_rQNn_rZ);
        
        % constuct C matrix
        C(i, 3) = diff_rQNn_rZ' * drQNn_dx3 / norm_diff;
        C(i, 4) = diff_rQNn_rZ' * drQNn_dx4 / norm_diff;
        C(i, 5) = diff_rQNn_rZ' * drQNn_dx5 / norm_diff;
    end
end
